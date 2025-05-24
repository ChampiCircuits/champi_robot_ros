#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import SetPose
from math import cos, sin, pi

from rclpy.executors import ExternalShutdownException
import time

init_pose = [1.855, 0.165, 2.6166]

class TestGoal(Node):

    def __init__(self):

        super().__init__('test_goal_node')

        # call service set_pose (0.145,0.165), theta = 30°
        self.set_pose_client = self.create_client(SetPose, '/set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = init_pose[0]
        request.pose.pose.pose.position.y = init_pose[1]
        request.pose.pose.pose.position.z = 0.
        request.pose.pose.pose.orientation.x = 0.
        request.pose.pose.pose.orientation.y = 0.
        request.pose.pose.pose.orientation.z = sin(init_pose[2]/2)
        request.pose.pose.pose.orientation.w = cos(init_pose[2]/2)

        future = self.set_pose_client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    break
                else:
                    self.get_logger().info('service call failed %r' % (future.exception(),))
                    break

                
        print("ok")
        time.sleep(1)

    
        # pub pose stamped
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.goals = [[1.0, 0.5, 0.],
                      [1.5, 0.4 , 0.],
                      [1.3, 2.0, 0.]]
        # self.goals = [[0., 1., -pi/2], [1., 1., 0.], [1., 0., pi/2],[0., 0., 0.]]
        self.i_goal = 0

        timer_period = 20  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.timer_callback()



    def timer_callback(self):

        goal = PoseStamped()
        # set timestamp 
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goals[self.i_goal][0]
        goal.pose.position.y = self.goals[self.i_goal][1]
        goal.pose.position.z = 0.
        goal.pose.orientation.x = 0.
        goal.pose.orientation.y = 0.
        goal.pose.orientation.z = sin(self.goals[self.i_goal][2]/2)
        goal.pose.orientation.w = cos(self.goals[self.i_goal][2]/2)

        self.pub.publish(goal)

        print("goal:",self.goals[self.i_goal])

        self.i_goal += 1
        if self.i_goal == len(self.goals):
            self.i_goal = 0

    # destructor
    def __del__(self):
        print('destroying TestGoal')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TestGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()