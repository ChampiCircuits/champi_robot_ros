import math
from controller import Supervisor

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

def clip(val, min_, max_):
    if val < min_:
        return min_
    if val > max_:
        return max_
    return val

supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())

mot1 = supervisor.getDevice("frontleft-omni-motor")
mot2 = supervisor.getDevice("frontright-omni-motor")
mot3 = supervisor.getDevice("backleft-omni-motor")
mot4 = supervisor.getDevice("backright-omni-motor")
mot1.setPosition(float('inf'))
mot2.setPosition(float('inf'))
mot3.setPosition(float('inf'))
mot4.setPosition(float('inf'))
mot1.setVelocity(0.0)
mot2.setVelocity(0.0)
mot3.setVelocity(0.0)
mot4.setVelocity(0.0)

enc1 = supervisor.getDevice("frontleft-omni-encoder")
enc2 = supervisor.getDevice("frontright-omni-encoder")
enc3 = supervisor.getDevice("backleft-omni-encoder")
enc4 = supervisor.getDevice("backright-omni-encoder")
enc1.enable(TIME_STEP)
enc2.enable(TIME_STEP)
enc3.enable(TIME_STEP)
enc4.enable(TIME_STEP)

gyro = supervisor.getDevice("gyro")
gyro.enable(TIME_STEP)

CENTER_TO_WHEEL = 0.130 # [m]
WHEEL_RADIUS = 0.030 # [m]
WHEEL_PERIMETER = 2.0 * math.pi * WHEEL_RADIUS # [m]
MAX_VEL = 0.1 # in [m/s]

def main():
    last_gt_translation = supervisor.getSelf().getField("translation").getSFVec3f()
    last_gt_rotation = supervisor.getSelf().getField("rotation").getSFVec3f()

    last_enc1 = 0
    last_enc2 = 0
    last_enc3 = 0
    last_enc4 = 0

    gyro_theta = 0.0
    odo_x = 0.0
    odo_y = 0.0
    odo_theta = 0.0

    TEST_POINTS1 = [
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (0.0, 0.5, 0.0),
        (-0.5, 0.0, 0.0),
        (-0.5, -0.5, 0.0),
        (0.5, -0.5, 0.0),
        (0.5, 0.5, 0.0),
    ]
    TEST_POINTS2 = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi/4.0),
        (0.5, 0.5,  0.0),
        (0.0, -0.5, -math.pi/4.0),
    ]
    POINTS = TEST_POINTS1
    current_point = 0

    pid_x = PID(5.0, 0.0, 0.0)
    pid_y = PID(5.0, 0.0, 0.0)
    pid_t = PID(10.0, 0.0, 0.0)

    timestep = TIME_STEP/1000.0 # in [s]

    i = 0

    while supervisor.step(TIME_STEP) != -1:
        # 0. GROUND TRUTH FROM SIMULATION =============================
        gt_translation = supervisor.getSelf().getField("translation").getSFVec3f()
        gt_rotation = supervisor.getSelf().getField("rotation").getSFVec3f()

        gt_dx = gt_translation[0] - last_gt_translation[0]
        gt_dy = gt_translation[1] - last_gt_translation[1]
        gt_dtheta = gt_rotation[3] - last_gt_rotation[3]

        gt_vx = gt_dx / timestep
        gt_vy = gt_dy / timestep
        gt_omega = gt_dtheta / timestep

        last_gt_translation = gt_translation
        last_gt_rotation = gt_rotation

        #print(f"GROUND TRUTH: {gt_vx} {gt_vy} {gt_omega}")



        # 1. ODOMETRY =============================
        # get gyro
        gyro_theta += gyro.getValues()[2] * timestep

        # get encoders
        enc1_val = enc1.getValue() / (2.0 * math.pi)
        enc2_val = enc2.getValue() / (2.0 * math.pi)
        enc3_val = enc3.getValue() / (2.0 * math.pi)
        enc4_val = enc4.getValue() / (2.0 * math.pi)

        # encoders difference from last state
        denc1 = (enc1_val - last_enc1) / timestep
        denc2 = (enc2_val - last_enc2) / timestep
        denc3 = (enc3_val - last_enc3) / timestep
        denc4 = (enc4_val - last_enc4) / timestep

        last_enc1 = enc1_val
        last_enc2 = enc2_val
        last_enc3 = enc3_val
        last_enc4 = enc4_val

        # inverse kinematics
        odo_vx = denc1 - denc2 + denc3 - denc4
        odo_vy = -denc1 - denc2 + denc3 + denc4
        odo_omega = -denc1 - denc2 - denc3 - denc4

        # magic constants, got them empirically from simulation
        odo_vx *= 0.0666463292934
        odo_vy *= 0.0666463292934
        odo_omega *= 0.331001127047

        odo_dx = odo_vx * timestep
        odo_dy = odo_vy * timestep
        odo_dtheta = odo_omega * timestep

        #odo_theta += odo_dtheta
        odo_theta = gyro_theta
        odo_x += odo_dx * math.cos(odo_theta) - odo_dy * math.sin(odo_theta)
        odo_y += odo_dx * math.sin(odo_theta) + odo_dy * math.cos(odo_theta)

        #print(f"ODO: {odo_x} {odo_y} {odo_theta}")



        # 2. PID =================
        err_x = POINTS[current_point][0] - odo_x
        err_y = POINTS[current_point][1] - odo_y
        err_t = POINTS[current_point][2] - odo_theta

        if math.fabs(err_x) < 0.001 and math.fabs(err_y) < 0.001 and (math.fabs(err_t) * 180.0 / math.pi) < 0.5:
            gt_err_x = round(1000.0 * (gt_translation[0] - POINTS[current_point][0]), 1) # in mm
            gt_err_y = round(1000.0 * (gt_translation[1] - POINTS[current_point][1]), 1) # in mm
            gt_err_t = round((gt_rotation[3] - POINTS[current_point][2]) * 180.0 / math.pi, 1) # in deg
            current_point = (current_point + 1) % len(POINTS)
            print(f"DONE (ground truth err x:{gt_err_x} y:{gt_err_y} t:{gt_err_t}), next goal i:{current_point} pos:{POINTS[current_point]}")

            if current_point == 0:
                i += 1
                if i == 10:
                    print("FINISHED 10 seq")
                    break

        px = pid_x.process(err_x)
        py = pid_y.process(err_y)
        pt = pid_t.process(err_t)

        px = clip(px, -MAX_VEL, MAX_VEL)
        py = clip(py, -MAX_VEL, MAX_VEL)
        pt = clip(pt, -MAX_VEL*2.0, MAX_VEL*2.0)



        # 3. MOTOR CONTROL =============================
        vx = px # [m/s]
        vy = py # [m/s]
        omega = pt # [rad/s]

        # magic constants
        vx *= (1.0 / WHEEL_RADIUS) * math.sqrt(2)
        vy *= (1.0 / WHEEL_RADIUS) * math.sqrt(2)
        omega *= 4.75446059161 # TODO find formula for this constant, got this empirically from simulation

        # forward kinematics
        v1 = vx - vy - omega
        v2 = -vx - vy - omega
        v3 = vx + vy - omega
        v4 = -vx + vy - omega

        # set motors velocities
        mot1.setVelocity(v1)
        mot2.setVelocity(v2)
        mot3.setVelocity(v3)
        mot4.setVelocity(v4)

    mot1.setVelocity(0.0)
    mot2.setVelocity(0.0)
    mot3.setVelocity(0.0)
    mot4.setVelocity(0.0)

if __name__ == "__main__":
    main()
