from multiprocessing.connection import Listener
import queue
import threading
import logging
import math
from controller import Display, Supervisor
import common
import common_sim

logging.basicConfig(format='[%(relativeCreated).3d] %(message)s', level=logging.DEBUG)
log = logging.getLogger("VRACbotController")

log.info("started VRACbotController")

supervisor = Supervisor()

log.info("connecting to motorboard IPC...")
listener_motorboard = Listener(('localhost', 6001))
conn_motor = listener_motorboard.accept()
log.info("IPC connected to motorboard %s", listener_motorboard.last_accepted)

data_motor: common_sim.SIM_Motor2Controller = None
data_motor_cond = threading.Condition()

log.info("connecting to raspiboard IPC...")
listener_raspiboard = Listener(('localhost', 6000))
conn_raspi = listener_raspiboard.accept()
log.info("IPC connected to raspiboard %s", listener_raspiboard.last_accepted)

ipc_raspi_queue = queue.Queue()
ipc_raspi_queue_lock = threading.Lock()

TIME_STEP = int(supervisor.getBasicTimeStep())

mot_left = supervisor.getDevice("left-wheel-motor")
mot_right = supervisor.getDevice("right-wheel-motor")
mot_left_enc = supervisor.getDevice("left-wheel-encoder")
mot_right_enc = supervisor.getDevice("right-wheel-encoder")
odo_left = supervisor.getDevice("left-odometry-encoder")
odo_right = supervisor.getDevice("right-odometry-encoder")
mot_left.setPosition(float('inf'))
mot_right.setPosition(float('inf'))
mot_left.setVelocity(0.0)
mot_right.setVelocity(0.0)
mot_left_enc.enable(TIME_STEP)
mot_right_enc.enable(TIME_STEP)
odo_left.enable(TIME_STEP)
odo_right.enable(TIME_STEP)

lidar = supervisor.getDevice("lidar")
lidar_hres = lidar.getHorizontalResolution()
lidardisplay = supervisor.getDevice("lidar-display")
lidardisplay_width = lidardisplay.getWidth()
lidardisplay_height = lidardisplay.getHeight()
lidar.enable(TIME_STEP)
WHITE: int = 0x00FFFFFF
BLACK: int = 0x00000000

camera = supervisor.getDevice("camera")
camera.enable(TIME_STEP)

def ipc_motor_thread():
    global data_motor
    while True:
        data_motor = conn_motor.recv()
        with data_motor_cond:
            data_motor_cond.notify()

def ipc_raspi_thread():
    while True:
        ipc_raspi_queue.put(conn_raspi.recv())

def main():
    log.info("starting controller")

    t_ipc_motor = threading.Thread(target=ipc_motor_thread)
    t_ipc_motor.start()

    t_ipc_raspi = threading.Thread(target=ipc_raspi_thread)
    t_ipc_raspi.start()

    while supervisor.step(TIME_STEP) != -1:
        # grab camera image
        camera.getImage()

        # display lidar
        points = lidar.getRangeImage()
        angle = 0.0
        lidardisplay.setColor(BLACK)
        lidardisplay.fillRectangle(0, 0, lidardisplay_width, lidardisplay_height)
        lidardisplay.setColor(WHITE)
        for p in points:
            if p != float('inf'):
                x: int = (p * math.sin(angle)) * (lidardisplay_width/2)
                y: int = (p * math.cos(angle)) * (lidardisplay_height/2)
                x += lidardisplay_width/2
                y += lidardisplay_height/2
                lidardisplay.drawPixel(x, y)
            angle += (2.0*math.pi)/lidar_hres

        # pass simulation lidar to raspiboard
        points: list[common_sim.LidarDataPoint] = []
        angle = 0.0
        for p in lidar.getRangeImage():
            angle += (2.0*math.pi)/lidar_hres
            points.append(common_sim.LidarDataPoint(p, angle, 1.0))
        conn_raspi.send(common_sim.SIM_Controller2Raspi(points))

        # pass simulation motors and encoders to motorboard
        odo_left_ticks = int((odo_left.getValue() / (2.0*math.pi)) * common.ODO_TICKS_PER_REV)
        odo_right_ticks = int((odo_right.getValue() / (2.0*math.pi)) * common.ODO_TICKS_PER_REV)
        mot_left_ticks = int((mot_left_enc.getValue() / (2.0*math.pi)) * common.MOT_TICKS_PER_REV)
        mot_right_ticks = int((mot_right_enc.getValue() / (2.0*math.pi)) * common.MOT_TICKS_PER_REV)
        conn_motor.send(common_sim.SIM_Controller2Motor(odo_left_ticks, odo_right_ticks, mot_left_ticks, mot_right_ticks, supervisor.getTime()))

        # pass motorboard to simulation
        with data_motor_cond:
            data_motor_cond.wait()
        vel_left = -common.clip(data_motor.pwm_left, -1.0, 1.0) * common.MOT_MAX_VEL
        vel_right = common.clip(data_motor.pwm_right, -1.0, 1.0) * common.MOT_MAX_VEL
        mot_left.setVelocity(vel_left)
        mot_right.setVelocity(vel_right)

if __name__ == "__main__":
    main()
