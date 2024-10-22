#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
import math

class RvizDisplayer(Node):
    def __init__(self):
        super().__init__('rvizDisplayer')

        # Souscriptions
        self.lasers_measurements_sub = self.create_subscription(Float32MultiArray, '/lasers_distances', self.lasers_measurements_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Publisher for RViz Markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Initial values
        self.odom_pose = None
        self.lasers_measurements = None
        self.get_logger().info("\tRVIZ displayer NODE launched!")


    def odom_callback(self, msg: Odometry):
        # Récupère la position et l'orientation du robot depuis l'odométrie
        self.odom_pose = msg.pose.pose
        # self.get_logger().warn("GOT ODOM")


    def lasers_measurements_callback(self, msg: Float32MultiArray):
        # Récupère les mesures du laser
        self.lasers_measurements = msg.data
        self.get_logger().info("GOT LASER "+str(msg.data[0])+"\t"+str(msg.data[1])+"\t"+str(msg.data[2])+"\t"+str(msg.data[3]))


        # Si les deux informations (odom et lasers) sont présentes, on affiche
        if self.odom_pose is not None and self.lasers_measurements is not None:
            self.publish_markers()

    def publish_markers(self):
        robot_radius = 0.1

        # Marqueur pour le cercle (représentant la position du robot)
        circle_marker = Marker()
        circle_marker.header.frame_id = "odom"
        circle_marker.header.stamp = self.get_clock().now().to_msg()
        circle_marker.ns = "circle"
        circle_marker.id = 0
        circle_marker.type = Marker.CYLINDER
        circle_marker.action = Marker.ADD
        circle_marker.pose = self.odom_pose  # Position and orientation
        circle_marker.scale.x = robot_radius*2  # Diamètre du cercle (rayon * 2)
        circle_marker.scale.y = robot_radius*2
        circle_marker.scale.z = 0.01  # Épaisseur du cercle
        circle_marker.color.r = 0.0
        circle_marker.color.g = 1.0
        circle_marker.color.b = 0.0
        circle_marker.color.a = 0.4

        # Publier le cercle
        self.marker_pub.publish(circle_marker)
        # self.get_logger().warn("PUB CIRCLE")


        # Marqueur pour les 4 traits
        if len(self.lasers_measurements) >= 4:
            directions = [math.pi - math.pi / 50, 
                          math.pi + math.pi / 50, 
                          -math.pi/2 - math.pi / 50,
                          -math.pi/2 + math.pi / 50]  # 4 directions: avant, droite, arrière, gauche

            for i in range(4):
                line_marker = Marker()
                line_marker.header.frame_id = "odom"
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.ns = "lines"
                line_marker.id = i + 1
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD


                # Calculer l'angle du trait en tenant compte de l'orientation du robot
                orientation_q = self.odom_pose.orientation

                robot_yaw = 2*math.atan2(orientation_q.z, orientation_q.w)

                angle = robot_yaw + directions[i]

                # Starting point (position actuelle du robot)
                start_point = Point()
                start_point.x = self.odom_pose.position.x + robot_radius * math.cos(angle)
                start_point.y = self.odom_pose.position.y + robot_radius * math.sin(angle)
                start_point.z = 0.0

                # End point (en fonction de la mesure du laser et de l'orientation)
                end_point = Point()
                end_point.x = start_point.x + self.lasers_measurements[i]/1000. * math.cos(angle)
                end_point.y = start_point.y + self.lasers_measurements[i]/1000. * math.sin(angle)
                end_point.z = 0.0

                # Ajouter les points au marker
                line_marker.points.append(start_point)
                line_marker.points.append(end_point)

                # Propriétés graphiques du trait
                line_marker.scale.x = 0.02  # Épaisseur du trait
                line_marker.color.r = 1.0
                line_marker.color.g = 0.0
                line_marker.color.b = 0.0
                line_marker.color.a = 1.0

                # Publier le trait
                self.marker_pub.publish(line_marker)


def main(args=None):
    rclpy.init(args=args)
    node = RvizDisplayer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
