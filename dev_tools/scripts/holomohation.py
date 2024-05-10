import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
import time

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


        # pub cmd vel
        self.pub_cmd_vel = self.create_publisher(Twist,"/base_controller/cmd_vel", 10)


        self.i_vel = 0
        
        self.vels = [
            [0.2, 0., 4.],
            [0., 0.2, 6.],
            [-0.2, 0., 3.]
        ]
        self.start_time = time.time()
        self.temps_deplacement = 0
        self.last_time = time.time()


    def listener_callback(self, msg):
        stop = False
        # Assuming the ranges array is populated
        for distance in msg.ranges:
            if distance < 0.7:
                # print(f"Distance: {distance} is greater than 0.3")
                stop = True
        if stop:
            # publish 0 cmd_vel
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.pub_cmd_vel.publish(twist)

            # si on est stopped, on décompte pas le temps


        else:
            if self.i_vel >= len(self.vels):
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.pub_cmd_vel.publish(twist)
                return
            
            self.get_logger().info(f"{self.i_vel}")
            twist = Twist()
            twist.linear.x = self.vels[self.i_vel][0]
            twist.linear.y = self.vels[self.i_vel][1]
            self.pub_cmd_vel.publish(twist)
            
            if self.temps_deplacement > self.vels[self.i_vel][2]:
                self.i_vel += 1
                self.start_time = time.time()
                self.temps_deplacement = 0

            if self.i_vel > len(self.vels):
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.pub_cmd_vel.publish(twist)


            self.temps_deplacement += time.time() - self.last_time

        self.last_time = time.time()

def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    # Destroy the node explicitly
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()