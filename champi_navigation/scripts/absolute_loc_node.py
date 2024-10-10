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
from math import sin, cos
from sensor_msgs.msg import LaserScan
import tf2_ros

# TODO c'est pas ca, mais on affinera
BEACON_1_POS = (0,0)
BEACON_2_POS = (1.5,3)
BEACON_3_POS = (2.0,0)

class AbsoluteLocNode(Node):
    def __init__(self):
        super().__init__('absolute_loc')

        # self.min_intensity= self.declare_parameter('min_intensity', rclpy.Parameter.Type.INTEGER).value
        self.min_intensity= 255

        self.laser_scan_sub_ = self.create_subscription(LaserScan,  "/scan", self.laser_callback, 10)
        self.last_scan: LaserScan = None

        self.publisher = self.create_publisher(MarkerArray, '/clusters', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 10Hz


        # Création d'un buffer TF pour stocker les transformations
        self.tf_buffer = tf2_ros.Buffer()
        # Création du listener TF pour remplir le buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Absolute loc node launched !")


    def timer_callback(self):
        if self.last_scan is None:
            return
        
        self.compute(self.last_scan)

    def laser_callback(self, msg):
        self.last_scan = msg

    def publish_circles(self, points: list[PointStamped]):
        marker_array = MarkerArray()
        
        for i, point in enumerate(points):
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
        
        :param laser_scan: Message LaserScan de ROS2 contenant les distances, angles et intensités.
        :return: Liste de clusters, chaque cluster est une liste de tuples (angle, distance, intensité).
        """
        # Extraction des données nécessaires
        angles = np.linspace(laser_scan.angle_min, laser_scan.angle_max, len(laser_scan.ranges))
        ranges = np.array(laser_scan.ranges)
        intensities = np.array(laser_scan.intensities)
        
        # Seuil d'intensité
        intensity_threshold = 255

        # Liste des clusters
        clusters = []
        current_cluster = []

        # Parcourir chaque point dans le scan
        for i in range(len(ranges)):
            if intensities[i] >= intensity_threshold and np.isfinite(ranges[i]):
                # Ajouter le point actuel au cluster en cours
                current_cluster.append((angles[i], ranges[i], intensities[i]))
            else:
                # Si le point ne satisfait pas la condition ou si le cluster est terminé, on sauvegarde
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []

        # Sauvegarder le dernier cluster s'il existe
        if current_cluster:
            clusters.append(current_cluster)

        return clusters

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


    def compute(self, last_scan: LaserScan):
        mask = [0]*len(last_scan.ranges)
        # on garde que les distances dont les intensités sont supérieures au seuil
        for i in range(len(last_scan.ranges)):
            if last_scan.intensities[i] >= self.min_intensity:
                mask[i] = 1
            else:
                mask[i] = 0


        # on doit identifier chaque cluster à chaque beacon 
        # attention on a plus de clusters que nos 3 beacons (des erreurs, ou des balises adverses, ou le robot adverse)

        clusters: list[list[PointStamped]] = [None]*100 #there should not be more than 100 x)
        cluster_count = 0
        in_cluster = False

        try:
            for i in range(len(mask)):
                if mask[i] == 1:
                    if in_cluster == False:
                        #start of cluster
                        cluster_count+=1
                        in_cluster = True
                        clusters[cluster_count] = [self.transform(last_scan.ranges[i], last_scan.angle_increment*i)]
                    else:
                        clusters[cluster_count].append(self.transform(last_scan.ranges[i], last_scan.angle_increment*i))
                elif in_cluster == True:
                    #end of cluster
                    in_cluster = False
        except Exception as e:
            self.get_logger().warn('Could not transform point: ' + str(e))

        # self.get_logger().info(f"clusters={clusters}")

        self.get_logger().info(f"cluster_count={cluster_count}")


        centroids = []
        for cluster in clusters:
            if cluster is None:
                continue
            centroid = PointStamped()
            for point in cluster:
                centroid.point.x += point.point.x
                centroid.point.y += point.point.y
            centroid.point.x /= len(cluster)
            centroid.point.y /= len(cluster)
            centroids.append(centroid)

            self.get_logger().info(f"{centroid.point.x}, {centroid.point.y}")

        self.publish_circles(centroids)

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