import enum
import math
import random
from controller import Supervisor

# a random speed is chosen in this interval for lines
# should be less than epsilon in is_destination_achieved()
MIN_VELOCITY = 0.002
MAX_VELOCITY = 0.004

# theta incremented each simulation step when rotating
# should be less than epsilon in is_rotation_achieved()
THETA = 0.02

# collision radius of the dummy robot, in meter
RADIUS = 0.3

# positions (x, y, wait_seconds)
YELLOW_BACKSTAGE = [
    (-1.125, 0.775, 1), # charging station
    (-0.675, 0.725, 1), # stack3
]
BLUE_BACKSTAGE = [
    (1.125, 0.775, 1), # charging station
    (0.675, 0.725, 1), # stack3 symetry
]
COMMUN_STACK = [
    (-0.725, -0.75, 1), # stack1
    (-0.4, -0.05, 1), # stack2
    (-1.425, 0.325, 1), # stack4
    (-1.425, -0.6, 1), # stack5
    (0.725, -0.75, 1), # stack1 symetry
    (0.4, -0.05, 1), # stack2 symetry
    (1.425, 0.325, 1), # stack4 symetry
    (1.425, -0.6, 1), # stack5 symetry
]
FLOOR_ARUCO = [
    (0.9, 0.4, 0),
    (0.9, -0.4, 0),
    (-0.9, 0.4, 0),
    (-0.9, -0.4, 0),
]
YELLOW_CONSTRUCTION = [
    (-0.275, -0.775, 1), # big, bottom center
    (-0.725, -0.925, 1), # small, bottom center
    (1.275, -0.925, 1), # small, bottom right
    (1.275, -0.125, 1), # big, right
]
BLUE_CONSTRUCTION = [
    (0.275, -0.775, 1), # big, bottom center
    (0.725, -0.925, 1), # small, bottom center
    (-1.275, -0.925, 1), # small, bottom left
    (-1.275, -0.125, 1), # big, left
]

class States(enum.Enum):
    ROTATE = 0
    GO_TO = 1

def is_destination_achieved(current_position: list[float], destination_position: list[float], epsilon=0.005):
    return abs(current_position[0] - destination_position[0]) < epsilon and abs(current_position[1] - destination_position[1]) < epsilon

def wait_at_destination(supervisor: Supervisor, timestep: int, wait_time: float):
    current_time = supervisor.getTime()
    while supervisor.getTime() - current_time < wait_time:
        supervisor.step(timestep)

def is_rotation_achieved(current_angle: float, required_angle: float, epsilon=0.03):
    return abs(current_angle - required_angle) < epsilon

def compute_next_destination(waypoints: list[tuple[float, float, float]], current_position: list[float], current_rotation: float):
    destination = random.choice(waypoints)
    velocity_factor = random.uniform(MIN_VELOCITY, MAX_VELOCITY)

    target_angle = math.atan2(destination[1] - current_position[1], destination[0] - current_position[0])
    relative_angle = target_angle - current_rotation

    if relative_angle > math.pi:
        relative_angle -= 2.0*math.pi
    if relative_angle < -math.pi:
        relative_angle += 2.0*math.pi
    
    return (destination, target_angle, relative_angle, velocity_factor)

def are_colliding(dummy_position: list[float], opponent_position: list[float]):
    dx = opponent_position[0] - dummy_position[0]
    dy = opponent_position[1] - dummy_position[1]
    return math.sqrt(dx ** 2 + dy ** 2) < RADIUS

def main():
    supervisor = Supervisor()

    print(f"starting DummyRobot id:{supervisor.getSelf().getId()}")

    timestep = int(supervisor.getBasicTimeStep())

    robot_dummy = supervisor.getSelf()
    robot_dummy_pos = robot_dummy.getField('translation')
    robot_dummy_rot = robot_dummy.getField('rotation')

    opponents = []
    childrends = supervisor.getRoot().getField('children')
    for i in range(childrends.getCount()):
        node = childrends.getMFNode(i)
        node_type = node.getTypeName()
        
        if node.getId() == robot_dummy.getId():
            continue
    
        if node_type == "DummyRobot" or node_type == "VRACbot" or node_type == "HolonomicRobot":
            opponents.append(node)
    
    # if you only want one specific opponent
    #opponents = [supervisor.getFromDef("OPPONENT")]

    print(f"opponents nodes: {[(node.getTypeName(),node.getId()) for node in opponents]}")

    waypoints = []
    if robot_dummy.getField('color').getSFString() == "blue":
        waypoints = COMMUN_STACK + FLOOR_ARUCO + BLUE_CONSTRUCTION + BLUE_BACKSTAGE
    else:
        waypoints = COMMUN_STACK + FLOOR_ARUCO + YELLOW_CONSTRUCTION + YELLOW_BACKSTAGE

    destination, target_angle, relative_angle, velocity_factor = compute_next_destination(waypoints, robot_dummy_pos.getSFVec3f(), robot_dummy_rot.getSFRotation()[3])
    next_state = States.ROTATE

    while supervisor.step(timestep) != -1:
        pos = robot_dummy_pos.getSFVec3f()
        rot = robot_dummy_rot.getSFRotation()

        if next_state == States.ROTATE:
            if not is_rotation_achieved(rot[3], target_angle):
                if relative_angle > 0.0:
                    rot[3] += THETA
                else:
                    rot[3] -= THETA
            else:
                rot[3] = target_angle
                next_state = States.GO_TO
            
            robot_dummy_rot.setSFRotation(rot)

        elif next_state == States.GO_TO:
            if not is_destination_achieved(pos, destination):
                is_colliding = False
                for opponent in opponents:
                    if are_colliding(pos, opponent.getField('translation').getSFVec3f()):
                        is_colliding = True
                        break
                if is_colliding:
                    continue

                delta_x = math.cos(target_angle) * velocity_factor
                delta_y = math.sin(target_angle) * velocity_factor

                pos[0] += delta_x
                pos[1] += delta_y
                robot_dummy_pos.setSFVec3f(pos)
            else:
                wait_at_destination(supervisor, timestep, destination[2])

                destination, target_angle, relative_angle, velocity_factor = compute_next_destination(waypoints, robot_dummy_pos.getSFVec3f(), robot_dummy_rot.getSFRotation()[3])
                next_state = States.ROTATE

if __name__ == '__main__':
    main()
