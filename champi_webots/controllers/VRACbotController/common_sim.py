class LidarDataPoint:
    def __init__(self, distance: float, angle: float, quality: float):
        self.distance = distance # in m
        self.angle = angle # in degree, 0 to 360
        self.quality = quality # between 0 and 1

class SIM_Controller2Raspi:
    def __init__(self, points: list[LidarDataPoint]):
        self.points = points

class SIM_Controller2Motor:
    def __init__(self, odo_left: int, odo_right: int, mot_left: int, mot_right: int, sim_time: float):
        self.odo_left = odo_left # in ticks
        self.odo_right = odo_right # in ticks
        self.mot_left = mot_left # in ticks
        self.mot_right = mot_right # in ticks
        self.sim_time = sim_time # in seconds

# from MotorBoard to SIM controller
class SIM_Motor2Controller:
    def __init__(self, pwm_left: float, pwm_right: float):
        self.pwm_left: float = pwm_left
        self.pwm_right: float = pwm_right