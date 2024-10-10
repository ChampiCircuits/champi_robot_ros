#!/usr/bin/python3
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
from math import sin, cos, acos, pi
from sensor_msgs.msg import LaserScan
import tf2_ros

# TODO c'est pas ca, mais on affinera
BEACON_1_POS = Point()
BEACON_1_POS.x = 0.8
BEACON_1_POS.y = 1.5
BEACON_2_POS = Point()
BEACON_2_POS.x = 2.0
BEACON_2_POS.y = 1.5
BEACON_3_POS = Point()
BEACON_3_POS.x = 2.0
BEACON_3_POS.y = 0.8

class AbsoluteLocNode(Node):
    def __init__(self):
        super().__init__('absolute_loc')

        # self.min_intensity= self.declare_parameter('min_intensity', rclpy.Parameter.Type.INTEGER).value
        self.min_intensity = 255
        self.min_cluster_size = 3
        self.cluster_distance_max = 0.06 #m
        self.threshold_centroids_identical = 0.01 #m
        self.max_dist_centroid_to_beacon = 0.2 #m

        self.laser_scan_sub_ = self.create_subscription(LaserScan,  "/scan", self.laser_callback, 10)
        self.last_scan: LaserScan = None

        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.robot_pose_callback, 10)
        self.last_robot_pose: PoseStamped = PoseStamped()

        self.publisher = self.create_publisher(MarkerArray, '/clusters', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 10Hz


        # Création d'un buffer TF pour stocker les transformations
        self.tf_buffer = tf2_ros.Buffer()
        # Création du listener TF pour remplir le buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Absolute loc node launched !")

    def robot_pose_callback(self, msg:Odometry):
        self.last_robot_pose.pose = msg.pose.pose
        self.get_logger().info(f"{self.last_robot_pose.pose.position.x} {self.last_robot_pose.pose.position.y}")

    def timer_callback(self):
        if self.last_scan is None:
            return
        self.compute(self.last_scan)

    def laser_callback(self, msg):
        self.last_scan = msg

    def publish_circles(self, points: list[PointStamped]):
        marker_array = MarkerArray()
        
        for i, point in enumerate(points):
            if point is None:
                continue
            marker = Marker()
            marker.header.frame_id = 'odom'  # Cadre de référence
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position du marqueur
            marker.pose.position = point.point

            # Orientation du marqueur
            marker.pose.orientation.w = 1.0

            # Taille du cercle
            marker.scale.x = 250 / 1000
            marker.scale.y = 250 / 1000
            marker.scale.z = 250 / 1000
            
            # Couleur du cercle
            marker.color = ColorRGBA(r=0., g=1., b=0., a=0.3)  # Vert avec pleine opacité
            
            # Durée du marqueur
            marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()  # 5 secondes
            
            # Identifiant unique pour chaque marqueur
            marker.id = i
            
            # Ajouter le marqueur à l'array
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


    def extract_clusters(self, laser_scan):
        """
        Extrait les clusters dont l'intensité est >= 255 à partir d'un message LaserScan.
        Tous les points d'un cluster sont à une distance <= self.cluster_distance_max les uns des autres.
        
        :param laser_scan: Message LaserScan de ROS2 contenant les distances, angles et intensités.
        :return: Liste de clusters, chaque cluster est une liste de geometry_msgs.msg.Point.
        """

        # Liste des clusters
        clusters = []
        current_cluster = []

        # Parcourir chaque point dans le scan
        try:
            angle = laser_scan.angle_min
            prev_point = None  # Point précédent pour mesurer la distance
            for i in range(len(laser_scan.ranges)):
                if laser_scan.intensities[i] >= self.min_intensity and np.isfinite(laser_scan.ranges[i]):
                    # Créer le point actuel en coordonnées cartésiennes
                    point = self.transform(laser_scan.ranges[i], angle)
                    
                    # Si le cluster est vide ou que le point est proche du point précédent, on l'ajoute au cluster
                    if not current_cluster or self.is_within_cluster_distance(prev_point, point):
                        current_cluster.append(point)
                    else:
                        # Si le point est trop loin, on ferme le cluster courant et on commence un nouveau
                        if len(current_cluster) > self.min_cluster_size:
                            clusters.append(current_cluster)
                        current_cluster = [point]  # Nouveau cluster avec le point actuel

                    # Mettre à jour le point précédent
                    prev_point = point
                else:
                    # Si le point ne satisfait pas la condition d'intensité ou est invalide, fermer le cluster
                    if current_cluster:
                        if len(current_cluster) > self.min_cluster_size:
                            clusters.append(current_cluster)
                        current_cluster = []

                # Incrémenter l'angle pour le point suivant
                angle += laser_scan.angle_increment

            # Sauvegarder le dernier cluster s'il existe
            if current_cluster and len(current_cluster) > self.min_cluster_size:
                clusters.append(current_cluster)
        except Exception as e:
            self.get_logger().warn(f"could not transform: {e}")


        return clusters

    def compute_and_merge_centroids(self, clusters):
        """
        Calcule les centroïdes des clusters et fusionne les centroïdes proches les uns des autres.
        
        :param clusters: Liste de clusters, chaque cluster étant une liste de geometry_msgs.msg.Point.
        :return: Liste de geometry_msgs.msg.PointStamped correspondant aux centroïdes fusionnés.
        """
        centroids = []
        
        # Calcul des centroïdes de chaque cluster
        for cluster in clusters:
            if cluster is None or len(cluster) == 0:
                continue
            centroid = PointStamped()
            for point in cluster:
                centroid.point.x += point.point.x
                centroid.point.y += point.point.y
            centroid.point.x /= len(cluster)
            centroid.point.y /= len(cluster)
            centroids.append(centroid)

        # Fusionner les centroïdes proches
        merged_centroids = []
        while centroids:
            current_centroid = centroids.pop(0)
            close_centroids = [current_centroid]

            # Chercher les centroids proches
            for other_centroid in centroids[:]:
                distance = np.sqrt((current_centroid.point.x - other_centroid.point.x) ** 2 +
                                (current_centroid.point.y - other_centroid.point.y) ** 2)
                if distance < self.threshold_centroids_identical:
                    close_centroids.append(other_centroid)
                    centroids.remove(other_centroid)

            # Fusionner les centroids proches en prenant la moyenne
            if len(close_centroids) > 1:
                avg_x = np.mean([cent.point.x for cent in close_centroids])
                avg_y = np.mean([cent.point.y for cent in close_centroids])
                merged_centroid = PointStamped()
                merged_centroid.point.x = avg_x
                merged_centroid.point.y = avg_y
            else:
                merged_centroid = current_centroid

            merged_centroids.append(merged_centroid)

        return merged_centroids

    def is_within_cluster_distance(self, point1, point2):
        """
        Vérifie si la distance entre deux points est inférieure ou égale à self.cluster_distance_max.
        
        :param point1: Premier Point (geometry_msgs.msg.Point).
        :param point2: Second Point (geometry_msgs.msg.Point).
        :return: True si les points sont suffisamment proches, False sinon.
        """
        if point1 is None or point2 is None:
            return False
        distance = np.sqrt((point1.point.x - point2.point.x) ** 2 + (point1.point.y - point2.point.y) ** 2)
        return distance <= self.cluster_distance_max

    
    def transform(self, dist: float, angle: float):
        point_in_scan = PointStamped()
        point_in_scan.header.frame_id = 'base_laser'  # Le repère d'origine
        # point_in_scan.header.stamp = rclpy.time.Time().to_msg()
        point_in_scan.point.x = dist * cos(angle)
        point_in_scan.point.y = dist * sin(angle)
        point_in_scan.point.z = 0.0
        
        # Transformer le point vers le repère "odom"
        point_on_table = self.tf_buffer.transform(point_in_scan, 'odom')
        return point_on_table

    def match_centroids_to_beacons(self, centroids):
        """
        Associe chaque centroid à la balise (beacon) la plus proche, si la distance est inférieure à self.max_dist_centroid_to_beacon.
        
        :param centroids: Liste de centroids (Point).
        :return: Liste de 3 Points correspondant aux positions des beacons (None si pas d'association).
        """
        # Positions des balises connues
        beacon_positions = [BEACON_1_POS, BEACON_2_POS, BEACON_3_POS]
        
        # Liste des balises avec leur centroid associé (None par défaut)
        matched_beacons = [None, None, None]

        # Pour chaque balise, trouver le centroid le plus proche
        for i, beacon_pos in enumerate(beacon_positions):
            closest_centroid = None
            closest_distance = float('inf')
            
            # Parcourir les centroids
            for centroid in centroids:
                # Calculer la distance entre la balise et le centroid
                distance = np.sqrt((centroid.point.x - beacon_pos.x) ** 2 +
                                (centroid.point.y - beacon_pos.y) ** 2)
                
                # Si la distance est plus petite que le seuil et que c'est le centroid le plus proche
                if distance < self.max_dist_centroid_to_beacon and distance < closest_distance:
                    closest_centroid = centroid
                    closest_distance = distance
            
            # Associer le centroid le plus proche si trouvé
            matched_beacons[i] = closest_centroid if closest_centroid else None

        return matched_beacons


    def compute(self, last_scan: LaserScan):
        clusters = self.extract_clusters(last_scan)
        cluster_count = len(clusters)
        

        self.get_logger().info(f"cluster_count={cluster_count}")


        centroids = self.compute_and_merge_centroids(clusters)

        beacons_position = self.match_centroids_to_beacons(centroids)

        self.publish_circles(beacons_position)

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