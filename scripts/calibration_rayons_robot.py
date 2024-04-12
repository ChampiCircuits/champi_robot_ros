#! /usr/bin/env python3


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy, time
from rclpy.duration import Duration
from math import sin, cos, pi

def wait_to_go_to_pose_without_error():
    while not navigator.isTaskComplete():
        pass

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
        exit_nav()
    elif result == TaskResult.FAILED:
        print('Goal failed!')
        exit_nav()
    else:
        print('Goal has an invalid return status!')
        exit_nav()

def exit_nav():
    print('Exiting...')
    # navigator.lifecycleShutdown()
    exit(0)

def pose_from_position(position, stamp):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = stamp
    goal_pose.pose.position.x = position[0]
    goal_pose.pose.position.y = position[1]
    # theta radians to quaternion
    goal_pose.pose.orientation.z = sin(position[2] / 2)
    goal_pose.pose.orientation.w = cos(position[2] / 2)
    return goal_pose

def pose_with_cov_from_position(position, stamp):
    goal_pose = PoseWithCovarianceStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = stamp
    goal_pose.pose.pose.position.x = position[0]
    goal_pose.pose.pose.position.y = position[1]
    # theta radians to quaternion
    goal_pose.pose.pose.orientation.z = sin(position[2] / 2)
    goal_pose.pose.pose.orientation.w = cos(position[2] / 2)
    return goal_pose

    # # Set our demo's initial pose
    # initial_pose = PoseWithCovarianceStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.pose.position.x = 0.5
    # initial_pose.pose.pose.position.y = 0.5
    # # initial_pose.pose.pose.position.x = 1.825
    # # initial_pose.pose.pose.position.y = 0.175
    # initial_yaw = 1.57
    # initial_pose.pose.pose.orientation.z = sin(initial_yaw / 2)
    # initial_pose.pose.pose.orientation.w = cos(initial_yaw / 2)
    # pub_initial_pose.publish(initial_pose)

def calibrate_lin(navigator):
    print("calibration du rayon des roues // mouvement linéaire")
    # Set the initial pose
    init_pose = [0.5, 0.5, 0.0]
    pub_initial_pose.publish(pose_with_cov_from_position(init_pose,navigator.get_clock().now().to_msg()))
    rclpy.spin_once(navigator)
    print("placez le robot sur le marqueur, et appuyez sur une touche...")
    input()

    # move forward 1m
    print("Avance de 1.0 m tout droit")
    goal_pose = [init_pose[0]+1.0, init_pose[1], init_pose[2]]
    navigator.goToPose(pose_from_position(goal_pose,navigator.get_clock().now().to_msg()))
    wait_to_go_to_pose_without_error()
    print("Si distance mesurée trop petite => diminuer rayon roues")
    print("Si distance mesurée trop grande => augmenter rayon roues")



def calibrate_ang(navigator):
    print("calibration du rayon du robot // mouvement angulaire")
    # Set the initial pose
    init_pose = [1.0, 1.0, 0.0]
    pub_initial_pose.publish(pose_with_cov_from_position(init_pose,navigator.get_clock().now().to_msg()))
    rclpy.spin_once(navigator)
    print("placez le robot sur le marqueur, et appuyez sur une touche...")
    input()

    # tourne de 3 tours
    nb_tours = 3
    print("Tourne de {0} tours",nb_tours)
    for i in range(nb_tours*2):
        goal_pose = [init_pose[0], init_pose[1], init_pose[2]+pi*(i+1)]
        print(goal_pose)
        navigator.goToPose(pose_from_position(goal_pose,navigator.get_clock().now().to_msg()))
        wait_to_go_to_pose_without_error()
    print("Si angle mesuré trop petit => diminuer rayon robot")
    print("Si angle mesuré trop grand => augmenter rayon robot")


if __name__ == "__main__":
    rclpy.init()
    navigator = BasicNavigator()
    pub_initial_pose = navigator.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    time.sleep(1)

    calibrate_lin(navigator)
    # calibrate_ang(navigator) # a faire une fois calibrate_lin satisfaisant
    exit_nav()
