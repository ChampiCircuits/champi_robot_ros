from enum import Enum
import time

class TrapezoidalVelocityProfile:

    class ProfileState(Enum):
        ACCELERATION = 1
        FLAT = 2
        DECELERATION = 3


    def __init__(self, max_speed, max_acceleration):
        self.v_max = max_speed
        self.a_max = max_acceleration

        self.start_pos = None
        self.end_pos = None

        self.state = None

        self.t_start = None

        self.t_end_acc = None
        self.t_end_flat = None
        self.t_end_dec = None

        self.error_negative = False


    def set_new_goal(self, start_pos, goal_pos, v_init = 0):
        # print("current speed",current_speed)
        # ic(current_speed)
        self.start_pos = start_pos
        self.end_pos = goal_pos

        if self.start_pos > self.end_pos:
            self.error_negative = True

        tf = 2 * sqrt(abs(self.end_pos - self.start_pos) / self.a_max)

        midpoint_vel = self.a_max * tf / 2

        if midpoint_vel <= self.v_max: # 2 segments case
            self.t_end_acc = (self.v_max-v_init) /self.a_max
            self.t_end_flat = None
            t_dec = (self.v_max-0) /self.a_max
            self.t_end_dec = self.t_end_acc + t_dec

            print("\n"*2)
            print("2 segments case")

        else: # 3 segments case
            V = self.v_max
            A = self.a_max

            self.t_end_acc = (V-v_init) / A
            self.t_end_dec = (V-0) / A
            deltaQ = self.end_pos - self.start_pos
            Tstar = deltaQ/V + (pow(V-v_init,2) + pow(V-0,2))/(2*A*V) # total minimum time
            self.t_end_flat = Tstar - self.t_end_acc - self.t_end_dec

            print("\n"*2)
            print("3 segments case")


        # if current_speed!=0:
        #     self.t_end_acc += (time.time()-self.t_start)
        if v_init == 0: # TODO OUI NON?
            self.t_start = time.time()

        print(self.t_start-time.time())
        print(self.t_end_acc)
        print(self.t_end_flat)
        print(self.t_end_dec)

    def compute_vel(self, pos_current): # TODO SERT A RIEN?

        t = time.time() - self.t_start

        # Update profile state
        if t < self.t_end_acc:
            self.state = self.ProfileState.ACCELERATION
        elif self.t_end_flat is not None and t < self.t_end_flat:
            self.state = self.ProfileState.FLAT
        elif t < self.t_end_dec:
            self.state = self.ProfileState.DECELERATION
        else:
            return 0

        # Update cmd
        cmd = 0
        if self.state == self.ProfileState.ACCELERATION:
            cmd = self.a_max * t
        elif self.state == self.ProfileState.FLAT:
            cmd = self.v_max
        else:
            if self.t_end_flat is not None:
                cmd = self.v_max - self.a_max * (t - self.t_end_flat)
            else:
                midpoint_vel = self.a_max * self.t_end_dec / 2
                cmd = midpoint_vel - self.a_max * (t - self.t_end_acc)

        if not self.error_negative:
            cmd = -cmd

        # ic(self.state, t, self.t_end_acc, self.t_end_flat, self.t_end_dec, cmd)

        return cmd