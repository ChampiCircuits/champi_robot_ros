import time
import threading
import struct
import math
import logging
import enum
import socket
import can
import common
from common import MotionState

logging.getLogger("can").setLevel(logging.WARNING)
logging.basicConfig(format='[%(relativeCreated).3d] %(message)s', level=logging.INFO)
log = logging.getLogger("MotorBoard")

#IF SIM
import common_sim
from multiprocessing.connection import Client
conn = Client(('localhost', 6001))
log.info("IPC connected to sim controller")
simu_data: common_sim.SIM_Controller2Motor = None
simu_cond = threading.Condition()

def ipc_read_thread():
    global simu_data

    while True:
        data: common_sim.SIM_Controller2Motor = conn.recv()
        simu_data = data
        with simu_cond:
            simu_cond.notify()

teleplot_addr = ("localhost", 47269)
teleplot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_telemetry(name, value):
    now = time.time() * 1000
    msg = name+":"+str(now)+":"+str(value)
    teleplot_sock.sendto(msg.encode(), teleplot_addr)
#ENDIF SIM

class PID:
    def __init__(self, kp, ki, kd):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        self.last_error: float = 0
        self.integrator: float = 0
    
    def process(self, error: float) -> float:
        self.integrator += error

        output = self.kp * error
        output += self.ki * self.integrator
        output += self.kd * (error - self.last_error)

        self.last_error = error

        return output

class Odometry:
    def __init__(self, control_loop_period: float, wheel_perimeter: float, ticks_per_rev: int, wheel_spacing: float):
        """
        :param control_loop_period: in seconds
        :param wheel_perimeter: odometry wheel perimeter, in mm
        :param ticks_per_rev: in ticks
        :param wheel_spacing: spacing between the two odometry wheels, in mm
        """
        self.control_loop_period = control_loop_period
        self.wheel_spacing = wheel_spacing
        self.k_wheel = wheel_perimeter / ticks_per_rev

        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0
        self.vel_distance_mm = 0.0
        self.vel_theta_rad = 0.0

        self.last_ticks_left = 0
        self.last_ticks_right = 0
    
    def compute(self, ticks_left: int, ticks_right: int):
        delta_left: float = (ticks_left - self.last_ticks_left) * self.k_wheel
        delta_right: float = (ticks_right - self.last_ticks_right) * self.k_wheel

        delta_distance: float = (delta_right + delta_left) / 2.0
        delta_theta: float = (delta_right - delta_left) / self.wheel_spacing

        self.distance_mm += delta_distance
        self.theta_rad += delta_theta

        self.vel_distance_mm = delta_distance / self.control_loop_period
        self.vel_theta_rad = delta_theta / self.control_loop_period

        self.x_mm += delta_distance * math.sin(self.theta_rad)
        self.y_mm += delta_distance * math.cos(self.theta_rad)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

    def get_x(self):
        return self.x_mm

    def get_y(self):
        return self.y_mm
    
    def get_theta_rad(self):
        return self.theta_rad

    def get_theta_deg(self):
        return self.theta_rad * 180.0 / math.pi
    
    def get_velocity_theta(self):
        """
        :returns: theta velocity in deg/s
        """
        return self.vel_theta_rad * 180.0 / math.pi

    def get_distance(self):
        return self.distance_mm

    def get_velocity_distance(self):
        return self.vel_distance_mm

class Setpoints:
    def __init__(self, distance_mm: int, theta_deg: int):
        self.distance_mm = distance_mm
        self.theta_deg = theta_deg

class TrapezoidProfileType(enum.Enum):
    TRAPZEOID = 0
    TRIANGLE = 1

class TrapezoidProfileState(enum.Enum):
    READY_TO_START = 0
    ACCELERATION = 1
    MAX_VELOCITY = 2
    DECELERATION = 3
    FINISHED = 4

class TrapezoidProfile:
    def __init__(self):
        self.acceleration = 0.0
        self.max_velocity = 0.0
        self.distance_sign = 1 # 1 if positive distance, else -1

        self.type = TrapezoidProfileType.TRAPZEOID
        self.accel_time = 0.0
        self.accel_dist = 0.0
        self.maxvel_time = 0.0
        self.maxvel_dist = 0.0

        self.force_brake_ = False
        self.state = TrapezoidProfileState.FINISHED
        self.start_time = 0.0
        self.last_velocity = 0.0
        self.last_position = 0.0

    def plan(self, acceleration: float, max_velocity: float, current_distance: float, distance: float):
        """
        :param acceleration: in in mm/s^-2 or deg/s^-2
        :param max_velocity: in mm/s or deg/s
        :distance: in mm or deg
        """
        self.type = TrapezoidProfileType.TRAPZEOID
        self.accel_time = max_velocity / acceleration
        self.accel_dist = 0.5 * acceleration * self.accel_time ** 2
        self.maxvel_dist = math.fabs(distance) - self.accel_dist * 2
        self.maxvel_time = self.maxvel_dist / max_velocity

        # if max_velocity can't be reached, recalculate as TRIANGLE profile
        if self.accel_dist * 2 > math.fabs(distance):
            self.type = TrapezoidProfileType.TRIANGLE
            self.accel_time = math.sqrt((2.0 * math.fabs(distance)) / (2.0 * acceleration))
            self.accel_dist = 0.5 * acceleration * self.accel_time ** 2
            self.maxvel_dist = 0.0
            self.maxvel_time = 0.0

        self.acceleration = acceleration
        self.max_velocity = max_velocity
        self.distance_sign = 1 if distance > 0.0 else -1
        self.state = TrapezoidProfileState.READY_TO_START
        self.force_brake_ = False
        self.last_velocity = 0.0
        self.last_position = current_distance

        log.debug("---- TrapezoidProfile plan ----")
        log.debug("type:%f", self.type.name)
        log.debug("distance:%f", distance)
        log.debug("accel_time:%f", self.accel_time)
        log.debug("accel_dist:%f", self.accel_dist)
        log.debug("maxvel_time:%f", self.maxvel_time)
        log.debug("maxvel_dist:%f", self.maxvel_dist)

    def process(self, current_time: float):
        """
        :param time_: global timer in seconds
        """
        if self.state == TrapezoidProfileState.READY_TO_START:
            self.start_time = current_time

        t = current_time -  self.start_time

        if t < self.accel_time:
            self.state = TrapezoidProfileState.ACCELERATION
        elif t < self.accel_time + self.maxvel_time:
            self.state = TrapezoidProfileState.MAX_VELOCITY
        elif t < self.accel_time + self.maxvel_time + self.accel_time:
            self.state = TrapezoidProfileState.DECELERATION
        else:
            self.state = TrapezoidProfileState.FINISHED
            self.force_brake_ = False

        if self.force_brake_:
            self.state = TrapezoidProfileState.DECELERATION

        velocity = 0.0

        match self.state:
            case TrapezoidProfileState.ACCELERATION:
                velocity = self.last_velocity + self.acceleration * CONTROL_LOOP_PERIOD
                velocity = common.clip(velocity, 0.0, self.max_velocity)
            case TrapezoidProfileState.MAX_VELOCITY:
                velocity = self.max_velocity
            case TrapezoidProfileState.DECELERATION:
                velocity = self.last_velocity - self.acceleration * CONTROL_LOOP_PERIOD
                velocity = common.clip(velocity, 0.0, self.max_velocity)
            case TrapezoidProfileState.FINISHED:
                velocity = 0.0

        self.last_velocity = velocity
        velocity *= self.distance_sign

        position = self.last_position + velocity * CONTROL_LOOP_PERIOD
        self.last_position = position

        return (position, velocity)

    def force_brake(self):
        self.force_brake_ = True

    def get_state(self):
        return self.state

    def is_finished(self):
        return self.state == TrapezoidProfileState.FINISHED

def can_alive_thread():
    while True:
        msg = can.Message(arbitration_id=common.CANID_MOTOR_ALIVE)
        bus.send(msg)
        time.sleep(3)

def can_read_thread():
    global motionstate

    while True:
        for msg in bus:
            if msg.arbitration_id == common.CANID_RASPI_ALIVE:
                log.debug("RaspiBoard IsAlive")

            elif msg.arbitration_id == common.CANID_MOTOR_LINE:
                if motionstate != MotionState.STAY_AT_POSITION:
                    log.warning("huh oh, asked for LINE but currently not STAY_AT_POSITION")
                    continue

                with motionstate_lock:
                    motionstate = MotionState.LINE

                distance = struct.unpack(">i", msg.data)[0]

                trapezoidprofile.plan(LINE_ACCEL, LINE_MAX_VELOCITY, setpoints.distance_mm, distance)
                setpoints.distance_mm += distance # setpoints should be locked too ?


                log.info("starting line distance:%d", distance)

            elif msg.arbitration_id == common.CANID_MOTOR_ROTATE:
                if motionstate != MotionState.STAY_AT_POSITION:
                    log.warning("huh oh, asked for ROTATE but currently not STAY_AT_POSITION")
                    continue

                with motionstate_lock:
                    motionstate = MotionState.ROTATE

                theta_deg = struct.unpack(">i", msg.data)[0]
                
                trapezoidprofile.plan(ROT_ACCEL, ROT_MAX_VELOCITY, setpoints.theta_deg, theta_deg)
                setpoints.theta_deg += theta_deg # setpoints should be locked too ?

                log.info("starting rotate theta_deg:%d", theta_deg)
        time.sleep(0.01)

def can_status_thread():
    """
    send odometry status (x, y, theta) on CAN at 10Hz
    """
    while True:
        data = bytes(struct.pack(">hhi", int(odo.get_x()), int(odo.get_y()), int(odo.get_theta_deg()*10.0)))
        msg = can.Message(arbitration_id=common.CANID_MOTOR_STATUS, data=data)
        bus.send(msg)
        time.sleep(0.1)

CONTROL_LOOP_PERIOD = 0.01 # be sure to match this with the basicTimeStep from world.wbt !
odo = Odometry(CONTROL_LOOP_PERIOD, common.ODO_WHEEL_PERIMETER, common.ODO_TICKS_PER_REV, common.ODO_WHEEL_SPACING)
motionstate = MotionState.STAY_AT_POSITION
motionstate_lock = threading.Lock()
trapezoidprofile = TrapezoidProfile()
LINE_MAX_VELOCITY = 1000 # in mm/s
LINE_ACCEL = 1500 # in mm/s^-2
ROT_MAX_VELOCITY = 200 # in rad/s
ROT_ACCEL = 400 # in rad/s^-2
setpoints = Setpoints(0, 0)
pid_dist = PID(0.01, 0, 0)
pid_theta = PID(0.3, 0, 0)

bus = can.ThreadSafeBus(interface='socketcan', channel='vcan0', bitrate=500000)

def main():
    global motionstate

    log.info("started MotorBoard")
    
    #IF SIM
    t_ipc = threading.Thread(target=ipc_read_thread)
    t_ipc.start()
    #ENDIF SIM

    t_can_alive = threading.Thread(target=can_alive_thread)
    t_can_alive.start()

    t_can_read = threading.Thread(target=can_read_thread)
    t_can_read.start()

    t_can_status = threading.Thread(target=can_status_thread)
    t_can_status.start()

    counter_atgoal = 0

    while True:
        # IF NOT SIM read odometry ticks from encoders here

        # IF SIM
        # It's necessary to wait for the simulation data because webots doesn't guarantee a fixed calculation time, but is dependent on the complexity of the scene.
        with simu_cond:
            simu_cond.wait()
        ticks_left = simu_data.odo_left
        ticks_right = simu_data.odo_right
        # ENDIF SIM

        # compute odometry
        odo.compute(ticks_left, ticks_right)

        # compute distance/theta errors
        dist_error = 0.0
        theta_error = 0.0

        match motionstate:
            case MotionState.STAY_AT_POSITION:
                dist_error = setpoints.distance_mm - odo.get_distance()
                theta_error = setpoints.theta_deg - odo.get_theta_deg()
            case MotionState.LINE:
                (pos, vel) = trapezoidprofile.process(simu_data.sim_time)
                dist_error = pos - odo.get_distance()
                theta_error = setpoints.theta_deg - odo.get_theta_deg()
            case MotionState.ROTATE:
                (pos, vel) = trapezoidprofile.process(simu_data.sim_time)
                dist_error = setpoints.distance_mm - odo.get_distance()
                theta_error = pos - odo.get_theta_deg()

        if motionstate != MotionState.STAY_AT_POSITION:
            send_telemetry("ramp_position", pos)
            send_telemetry("ramp_velocity", vel)

        send_telemetry("theta_odo_position", odo.get_theta_deg())
        send_telemetry("theta_odo_velocity", odo.get_velocity_theta())

        send_telemetry("dist_odo_position", odo.get_distance())
        send_telemetry("dist_odo_velocity", odo.get_velocity_distance())

        # velocity control test
        # dist_error = 500 - odo.get_velocity_distance()
        # send_telemetry("get_velocity_distance", odo.get_velocity_distance())

        # compute PIDs
        dist_pwm = 0.0
        theta_pwm = 0.0

        dist_pwm = pid_dist.process(dist_error)
        theta_pwm = pid_theta.process(theta_error)

        # compute left and right pwm
        left_pwm = -dist_pwm + theta_pwm
        right_pwm = dist_pwm + theta_pwm

        # IF NOT SIM set motors PWM here

        # IF SIM
        conn.send(common_sim.SIM_Motor2Controller(left_pwm, right_pwm))
        # ENDIF SIM

        # if motion finished, send CAN msg
        if trapezoidprofile.is_finished():
            if motionstate == MotionState.LINE:
                if math.fabs(dist_error) < 1.0 and odo.get_velocity_distance() < 2:
                    counter_atgoal += 1
                else:
                    counter_atgoal = 0

                if counter_atgoal > 10:
                    log.info("LINE arrived at dest!, err:%f", dist_error)

                    with motionstate_lock:
                        motionstate = MotionState.STAY_AT_POSITION

                    counter_atgoal = 0

                    msg = can.Message(arbitration_id=common.CANID_MOTOR_DONE)
                    bus.send(msg)

            elif motionstate == MotionState.ROTATE:
                if math.fabs(theta_error) < 0.2 and odo.get_velocity_theta() < 2:
                    counter_atgoal += 1
                else:
                    counter_atgoal = 0

                if counter_atgoal > 10:
                    log.info("ROTATE arrived at dest!, err:%f", theta_error)

                    with motionstate_lock:
                        motionstate = MotionState.STAY_AT_POSITION

                    counter_atgoal = 0

                    msg = can.Message(arbitration_id=common.CANID_MOTOR_DONE)
                    bus.send(msg)

if __name__ == "__main__":
    main()
