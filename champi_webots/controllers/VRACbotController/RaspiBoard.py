import time
import threading
import struct
import logging
import queue
import can
import common
import common_sim
from common import MotionState

logging.getLogger("can").setLevel(logging.WARNING)
logging.basicConfig(format='[%(relativeCreated).3d] %(message)s', level=logging.DEBUG)
log = logging.getLogger("RaspiBoard")

#IF SIM
from multiprocessing.connection import Client
conn = Client(('localhost', 6000))
log.info("IPC connected to sim controller")
last_sim_state: common_sim.SIM_Controller2Motor = None
last_sim_state_lock = threading.Lock()
#ENDIF SIM

bus = can.ThreadSafeBus(interface='socketcan', channel='vcan0', bitrate=500000)
can_queue = queue.Queue()

motionstate = MotionState.STAY_AT_POSITION
motionstate_lock = threading.Lock()

#IF SIM
def ipc_read_thread():
    global last_sim_state

    while True:
        data: common_sim.SIM_Controller2Raspi = conn.recv()
        with last_sim_state_lock:
            last_sim_state = data
#ENDIF SIM

def lidar_thread(): # at 10Hz
    while True:
        #IF SIM
        with last_sim_state_lock:
            if last_sim_state:
                #print("received lidar state! len:", len(last_sim_state.points))
                noninfpoints = 0
                for p in last_sim_state.points:
                    if p.distance != float('inf'):
                        noninfpoints += 1
                #print("LIDAR non inf: ", noninfpoints)
        #ENDIF SIM

        time.sleep(0.1)

def can_alive_thread():
    while True:
        msg = can.Message(arbitration_id=common.CANID_RASPI_ALIVE)
        bus.send(msg)
        time.sleep(3)

def can_read_thread():
    global motionstate
    while True:
        for msg in bus:
            if msg.arbitration_id == common.CANID_MOTOR_ALIVE:
                log.debug("MotorBoard IsAlive")

            elif msg.arbitration_id == common.CANID_MOTOR_STATUS:
                x, y, t = struct.unpack(">hhi", msg.data)
                t = t/10.0
                log.debug("MotorBoard Status {x:%04d y:%04d t:%.1f}", x, y, t)
            
            elif msg.arbitration_id == common.CANID_MOTOR_DONE:
                with motionstate_lock:
                    motionstate = MotionState.STAY_AT_POSITION
                log.debug("MotorBoard Done")

def line(distance: int):
    global motionstate

    print("doin line ", distance)
    data = bytearray(struct.pack(">i", distance))
    msg = can.Message(arbitration_id=common.CANID_MOTOR_LINE, data=data)
    bus.send(msg)

    with motionstate_lock:
        motionstate = MotionState.LINE

def rotate(degree: int):
    global motionstate

    print("doin rotate ", degree)
    data = bytearray(struct.pack(">i", degree))
    msg = can.Message(arbitration_id=common.CANID_MOTOR_ROTATE, data=data)
    bus.send(msg)

    with motionstate_lock:
        motionstate = MotionState.ROTATE

def wait_until_motion_finished():
    while True:
        with motionstate_lock:
            if motionstate == MotionState.STAY_AT_POSITION:
                return

        time.sleep(0.1)

def main():
    print("started RaspiBoard")

    #IF SIM
    t_ipc = threading.Thread(target=ipc_read_thread)
    t_ipc.start()
    #ENDIF

    t_can_alive = threading.Thread(target=can_alive_thread)
    t_can_alive.start()

    t_can_read = threading.Thread(target=can_read_thread)
    t_can_read.start()

    t_lidar = threading.Thread(target=lidar_thread)
    t_lidar.start()

    print("GO!")

    while True:
        rotate(45)
        wait_until_motion_finished()
    
        line(650)
        wait_until_motion_finished()

        rotate(-45)
        wait_until_motion_finished()

        line(800)
        wait_until_motion_finished()

        rotate(-90)
        wait_until_motion_finished()

        line(360)
        wait_until_motion_finished()

        rotate(-90)
        wait_until_motion_finished()

        line(950)
        wait_until_motion_finished()

        line(-300)
        wait_until_motion_finished()

        rotate(150)
        wait_until_motion_finished()

        line(900)
        wait_until_motion_finished()

        rotate(30)
        wait_until_motion_finished()

        line(600)
        wait_until_motion_finished()

        rotate(-90)
        wait_until_motion_finished()

        line(280)
        wait_until_motion_finished()

        line(-280)
        wait_until_motion_finished()

        rotate(165)
        wait_until_motion_finished()

        line(1400)
        wait_until_motion_finished()

        while True:
            time.sleep(1)

if __name__ == "__main__":
    main()
