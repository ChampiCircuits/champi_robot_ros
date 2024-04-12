#! /usr/bin/env python3


"""
Si on veut cancel une task de nav après un timeout:
if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

On peut aussi préempter une task en en donnant just une nouvelle:
    navigator.goToPose(goal_pose)
"""


from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from math import sin, cos, pi
import signal

carre = [
    [0.5, 0.5, pi/2],
    [1.5, 0.5, pi],
    [1.5, 2.5, pi/2],
    [0.5, 1.5, pi]
]

aller_retour = [
    [1.0, 1.0, 1.57],
    [1.0, 2.0,1.57]
]

positions = carre

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

def main():
    rclpy.init()

    navigator = BasicNavigator()

    pub_initial_pose = navigator.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    # Set our demo's initial pose
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.pose.position.x = 0.5
    initial_pose.pose.pose.position.y = 0.5
    # initial_pose.pose.pose.position.x = 1.825
    # initial_pose.pose.pose.position.y = 0.175
    initial_yaw = 1.57
    initial_pose.pose.pose.orientation.z = sin(initial_yaw / 2)
    initial_pose.pose.pose.orientation.w = cos(initial_yaw / 2)
    pub_initial_pose.publish(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    print("Waiting for navigation to activate...")
    # navigator.waitUntilNav2Active(localizer="") # do not wait for amcl
    print("Navigation is active!")

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # 

    i_poses = 0
    while rclpy.ok(): # for Ctrl-C handling
        goal_pose = pose_from_position(positions[i_poses], navigator.get_clock().now().to_msg())
        # sanity check a valid path exists
        # path = navigator.getPath(current_pose, goal_pose)
        navigator.goToPose(goal_pose)
        i_poses = (i_poses + 1) % len(positions)
        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # # Some navigation timeout to demo cancellation
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                #     navigator.cancelTask()

                # # Some navigation request change to demo preemption
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                #     goal_pose.pose.position.x = -3.0
                #     navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
    print('Exiting...')
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
