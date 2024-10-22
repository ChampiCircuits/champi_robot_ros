#!/usr/bin/python3
import random
import tf2_geometry_msgs
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, PointStamped
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
import math
from rclpy.executors import ExternalShutdownException
from math import sin, cos, acos, pi, inf
from sensor_msgs.msg import PointCloud, LaserScan
from geometry_msgs.msg import Point32
import tf2_ros
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN

from circle_detector import detect_circles, laserscan_to_cartesian, create_binary_image, show_image
import icp

# TODO c'est pas ca, mais on affinera
BEACON_1_POS = PointStamped()
BEACON_1_POS.point.x = 0.8+0.01
BEACON_1_POS.point.y = 1.5-0.03
BEACON_2_POS = PointStamped()
BEACON_2_POS.point.x = 2.0-0.01
BEACON_2_POS.point.y = 1.5-0.03
BEACON_3_POS = PointStamped()
BEACON_3_POS.point.x = 2.0
BEACON_3_POS.point.y = 0.8+0.01

R_BEACON = 0.055

class AbsoluteLocNode(Node):
    def __init__(self):
        super().__init__('absolute_loc')

        # self.min_intensity= self.declare_parameter('min_intensity', rclpy.Parameter.Type.INTEGER).value
        self.min_intensity = 250
        self.min_cluster_size = 3
        self.cluster_distance_max = 0.5 #m
        self.threshold_centroids_identical = 0.1 #m
        self.max_dist_centroid_to_beacon = 3.0 #m

        # Paramètres DBSCAN
        self.eps = 0.5  # Distance maximale entre deux points pour être considérés dans le même cluster
        self.min_samples = 5  # Nombre minimum de points dans un cluster

        # Paramètres Hough
        self.beacon_diameter = 0.08
        self.diameter_margin = 0.03

        self.laser_scan_sub_ = self.create_subscription(LaserScan,  "/scan", self.laser_callback, 10)
        self.last_scan: LaserScan = None

        # self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.robot_pose_callback, 10)
        # self.last_robot_pose: PoseStamped = PoseStamped()

        self.publisher = self.create_publisher(MarkerArray, '/clusters', 10)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/pose/absolute_loc', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 10Hz


        # Création d'un buffer TF pour stocker les transformations
        self.tf_buffer = tf2_ros.Buffer()
        # # Création du listener TF pour remplir le buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ref_point_cloud_pub = self.create_publisher(PointCloud, '/ref_point_cloud', 10)
        self.current_point_cloud_pub = self.create_publisher(PointCloud, '/current_point_cloud', 10)


        self.get_logger().info(f"Absolute loc node launched !")

    # def robot_pose_callback(self, msg:Odometry):
    #     self.last_robot_pose.pose = msg.pose.pose
        # self.get_logger().info(f"{self.last_robot_pose.pose.position.x} {self.last_robot_pose.pose.position.y}")

    def timer_callback(self):
        if self.last_scan is None:
            return
        self.compute()

    def laser_callback(self, msg):
        self.last_scan = msg


    def publish_robot_pose(self, robot_position):
        """
        Publie une PoseWithCovarianceStamped à partir de la position du robot (robot_position) dans ROS2.
        
        :param robot_position: Position du robot sous forme d'un geometry_msgs.msg.Point (x, y, z).
        """
        if robot_position is None:
            return

        # Créer un message PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()

        # Ajouter un header avec l'horodatage et le frame_id (par exemple "odom")
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Horodatage ROS2
        pose_msg.header.frame_id = "odom"  # Frame de référence

        # Définir la position (x, y, z)
        pose_msg.pose.pose.position = Point(
            x=robot_position.x,
            y=robot_position.y,
            z=0.0 
        )

        # Définir une orientation neutre (pas de rotation, quaternion identitaire)
        pose_msg.pose.pose.orientation = self.last_robot_pose.pose.orientation # TODO on voudra calc l'orientation aussi

        # Ajouter une covariance faible (petites valeurs pour représenter une bonne confiance)
        covariance_matrix = np.zeros((6, 6))  # Matrice 6x6 initialisée à zéro
        covariance_matrix[0, 0] = 0.02  # Variance sur x
        covariance_matrix[1, 1] = 0.02  # Variance sur y
        covariance_matrix[5, 5] = 0.02  # Variance sur yaw (rotation autour de z)

        # Aplatir la matrice en un tableau 1D pour l'insérer dans la covariance
        pose_msg.pose.covariance = covariance_matrix.flatten().tolist()

        # Publier le message via le publisher ROS2
        self.pose_publisher.publish(pose_msg)

    def compute(self):

        ref_point_cloud = self.generate_pointcloud_with_circles(R_BEACON*2, [BEACON_1_POS,BEACON_2_POS,BEACON_3_POS], num_points=50, uniform=0.1)
        point_cloud_in_odom = self.transform_scan_in_odom_point_cloud(self.last_scan)

        if len(ref_point_cloud.points) == 0 or len(point_cloud_in_odom.points) == 0:
            return


        # H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = 
        icp.apply_icp_from_ros_pointclouds(ref_point_cloud, point_cloud_in_odom)

        # self.get_logger().warn(f"X_mov_transformed: {len(X_mov_transformed)} {len(distance_residuals)}")
        # self.get_logger().warn(f"")
        # self.get_logger().warn(f"{transformation_history}")


        self.ref_point_cloud_pub.publish(ref_point_cloud)
        self.current_point_cloud_pub.publish(point_cloud_in_odom)

    def transform_scan_in_odom_point_cloud(self, laser_scan: LaserScan):
        """
        Transforme un message LaserScan en un PointCloud dans le repère 'odom'.

        :param laser_scan: message LaserScan contenant les données des points à transformer
        :return: PointCloud contenant les points transformés dans le repère 'odom'
        """
        # Créer un objet PointCloud
        cloud = PointCloud()
        cloud.header.frame_id = 'odom'  # Le nuage de points sera exprimé dans le repère 'odom'

        angle = laser_scan.angle_min  # Initialiser avec l'angle minimal

        try:
            # Parcourir les distances (ranges) du LaserScan
            for r in laser_scan.ranges:
                # Si la distance est dans la plage valide, on traite le point
                if laser_scan.range_min < r < laser_scan.range_max:
                    # Convertir chaque point (r, angle) en coordonnées cartésiennes (x, y)
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)

                    # Créer un PointStamped pour transformer le point en coordonnées 'odom'
                    p = tf2_geometry_msgs.PointStamped()
                    p.header.frame_id = laser_scan.header.frame_id  # Le frame de départ est celui du LaserScan (généralement 'base_laser')
                    p.point.x = x
                    p.point.y = y
                    p.point.z = 0.

                    # Transformer le point dans la frame 'odom'
                    transformed_point = self.tf_buffer.transform(p, 'odom')

                    # Ajouter le point transformé au nuage de points
                    point = Point32()
                    point.x = transformed_point.point.x
                    point.y = transformed_point.point.y
                    # point.z = transformed_point.point.z
                    point.z = 0. # on le remet en 0 pour matcher dans ICP
                    cloud.points.append(point)

                # Incrémenter l'angle pour le prochain point
                angle += laser_scan.angle_increment

        except Exception as e:
            self.get_logger().warn(message='Error transforming scan point: ' + str(e))

        return cloud        


    def generate_pointcloud_with_circles(self, diameter, positions: list[tf2_geometry_msgs.PointStamped], num_points=60, uniform=0.):
        """
        Génère un nuage de points PointCloud simulant trois cercles avec un diamètre unique et des positions données.

        :param diameter: diamètre unique des trois cercles
        :param positions: liste de tf2_geometry_msgs.PointStamped contenant les coordonnées (x, y) des centres des trois cercles
        :param num_points: nombre total de points à générer (par défaut 60)
        :return: message PointCloud simulé avec les cercles.
        """
        # Créer un objet PointCloud
        cloud = PointCloud()
        cloud.header.frame_id = 'odom'  # Frame de référence pour les points

        # Calcul du rayon à partir du diamètre
        radius = diameter / 2

        try:
            # Génération des points pour chaque cercle
            for position in positions:
                center_x, center_y = position.point.x, position.point.y

                # Générer des points pour chaque cercle
                num_circle_points = num_points // len(positions)  # Répartir les points sur les 3 cercles
                circle_angles = np.linspace(0, 2 * np.pi, num_circle_points)

                for angle in circle_angles:
                    # Créer un PointStamped pour chaque point sur le cercle
                    p = tf2_geometry_msgs.PointStamped()
                    p.header.frame_id = 'odom'
                    p.point.x = center_x + radius * math.cos(angle) #+ random.uniform(-uniform, uniform)
                    p.point.y = center_y + radius * math.sin(angle) +uniform #+ random.uniform(-uniform, uniform) + uniform
                    p.point.z = 0.

                    # Transformer le point dans la frame 'base_laser'
                    # transformed_point = self.tf_buffer.transform(p, 'odom')
                    transformed_point = p

                    # Ajouter le point transformé au nuage de points
                    point = Point32()
                    point.x = transformed_point.point.x
                    point.y = transformed_point.point.y
                    point.z = transformed_point.point.z

                    cloud.points.append(point)
        except Exception as e:
            self.get_logger().warn(message='Could not transform beacon point: ' + str(e))

        return cloud

def main(args=None):
    rclpy.init(args=args)

    node = AbsoluteLocNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()