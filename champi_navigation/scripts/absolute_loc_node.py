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
from math import sin, cos, acos, pi, inf
from sensor_msgs.msg import LaserScan
import tf2_ros
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN

# TODO c'est pas ca, mais on affinera
BEACON_1_POS = PointStamped()
BEACON_1_POS.point.x = 0.8
BEACON_1_POS.point.y = 1.5
BEACON_2_POS = PointStamped()
BEACON_2_POS.point.x = 2.0
BEACON_2_POS.point.y = 1.5
BEACON_3_POS = PointStamped()
BEACON_3_POS.point.x = 2.0
BEACON_3_POS.point.y = 0.8

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


        self.laser_scan_sub_ = self.create_subscription(LaserScan,  "/scan", self.laser_callback, 10)
        self.last_scan: LaserScan = None

        # self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.robot_pose_callback, 10)
        # self.last_robot_pose: PoseStamped = PoseStamped()

        self.publisher = self.create_publisher(MarkerArray, '/clusters', 10)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/pose/absolute_loc', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 10Hz


        # Création d'un buffer TF pour stocker les transformations
        # self.tf_buffer = tf2_ros.Buffer()
        # # Création du listener TF pour remplir le buffer
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Absolute loc node launched !")

    # def robot_pose_callback(self, msg:Odometry):
    #     self.last_robot_pose.pose = msg.pose.pose
        # self.get_logger().info(f"{self.last_robot_pose.pose.position.x} {self.last_robot_pose.pose.position.y}")

    def timer_callback(self):
        if self.last_scan is None:
            return
        self.compute(self.last_scan)

    def laser_callback(self, msg):
        self.last_scan = msg

    def publish_circles(self, points: list[PointStamped], robot_position:PoseStamped):
        marker_array = MarkerArray()
        
        for i, point in enumerate(points):
            if point is None:
                continue
            marker = Marker()
            marker.header.frame_id = 'base_laser'  # Cadre de référence
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
            marker.color = ColorRGBA(r=0., g=1., b=0., a=0.3)  
            
            # Durée du marqueur
            marker.lifetime = rclpy.time.Duration(seconds=0.5).to_msg() 
            
            # Identifiant unique pour chaque marqueur
            marker.id = i
            
            # Ajouter le marqueur à l'array
            marker_array.markers.append(marker)

        if robot_position is not None:
            marker = Marker()
            marker.header.frame_id = 'odom'  # Cadre de référence
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position du marqueur
            marker.pose.position.x = robot_position.pose.position.x
            marker.pose.position.y = robot_position.pose.position.y

            # Orientation du marqueur
            marker.pose.orientation.w = 1.0

            # Taille du cercle
            marker.scale.x = 150 / 1000
            marker.scale.y = 150 / 1000
            marker.scale.z = 150 / 1000
            
            # Couleur du cercle
            marker.color = ColorRGBA(r=0., g=0., b=1., a=0.6) 
            
            # Durée du marqueur
            marker.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()  # 5 secondes
            
            # Identifiant unique pour chaque marqueur
            marker.id = len(points)
            
            # Ajouter le marqueur à l'array
            marker_array.markers.append(marker)


        self.publisher.publish(marker_array)


    def extract_clusters(self, last_scan):
        """
        Extrait les clusters dont l'intensité est >= 255 à partir d'un message LaserScan.
        Tous les points d'un cluster sont à une distance <= self.cluster_distance_max les uns des autres.
        
        :param laser_scan: Message LaserScan de ROS2 contenant les distances, angles et intensités.
        :return: Liste de clusters, chaque cluster est une liste de geometry_msgs.msg.Point.
        """

        msg = last_scan

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        intensities = np.array(msg.intensities)

        # Filtrer les points où l'intensité est supérieure au seuil
        valid_indices = intensities > self.min_intensity
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Si aucun point valide n'est trouvé
        if len(valid_ranges) == 0:
            self.get_logger().info('Aucun point avec une intensité supérieure à min_intensity.')
            return

        # Conversion des distances et angles en coordonnées cartésiennes (x, y)
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        points = np.vstack((x_coords, y_coords)).T

        # Appliquer DBSCAN pour détecter des clusters
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)

        # Récupérer les labels des clusters (label -1 signifie "bruit", non-clusterisé)
        labels = clustering.labels_

        # Liste finale des clusters sous forme de PointStamped
        clusters = []

        # Créer des PointStamped pour chaque cluster
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue  # Ignorer le bruit

            cluster_points = points[labels == label]

            cluster = []
            for i, point in enumerate(cluster_points):
                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id  # Frame du LaserScan
                point_stamped.header.stamp = msg.header.stamp  # Timestamp du LaserScan
                point_stamped.point.x = point[0]
                point_stamped.point.y = point[1]
                point_stamped.point.z = 0.0  # Le scan est en 2D, donc z est fixé à 0

                cluster.append(point_stamped)

            # Ajouter le cluster à la liste des clusters
            clusters.append(cluster)

        # Affichage du nombre de clusters trouvés
        self.get_logger().info(f'{len(clusters)} clusters détectés')

        # Optionnel : Afficher le contenu de chaque cluster
        # for idx, cluster in enumerate(clusters):
        #     self.get_logger().info(f'Cluster {idx}: {len(cluster)} points')
        #     for point_stamped in cluster:
        #         self.get_logger().info(f'Point (x: {point_stamped.point.x:.2f}, y: {point_stamped.point.y:.2f})')

        # Retourner la liste des clusters
        return clusters

    def compute_and_merge_centroids(self, clusters) -> list[PointStamped]:
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
            # self.get_logger().info(f"{centroid.point.x} {centroid.point.y}")

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
        beacon_real_positions = [BEACON_1_POS, BEACON_2_POS, BEACON_3_POS]
        real_dist12 = self.dist_squared(BEACON_1_POS, BEACON_2_POS)
        real_dist23 = self.dist_squared(BEACON_3_POS, BEACON_2_POS)
        real_dist31 = self.dist_squared(BEACON_1_POS, BEACON_3_POS)
        real_dists = [real_dist12, real_dist23, real_dist31]
        

        dist1 = self.dist_squared(centroids[0], centroids[1])
        dist2 = self.dist_squared(centroids[1], centroids[2])
        dist3 = self.dist_squared(centroids[2], centroids[0])
        dists = [dist1, dist2, dist3]


        # on matche les deux plus petites distances entre elles
        i_min_real = np.argmin(real_dists)
        i_min =      np.argmin(dists)

        # on sait que dist1 c'est real_dist12 (PAR EXEMPLE)
        # donc 1reel et 2reel correspondent aux points 1 et 2, mais on sait pas dans quel ordre
        # on les matche en prenant celui le plus loin dans le sens trigo
        # self.get_logger().warn(f"{real_dists} {dists}")
        """BEACON_1_POS c'est l'opposé"""
    
        if i_min == 0:
            self.get_logger().warn("0")
            beacon1 = centroids[2]
            if np.cross(np.array([centroids[0].point.x, centroids[0].point.y]), 
                        np.array([centroids[1].point.x, centroids[1].point.y])) < 0:
                beacon2 = centroids[0]
                beacon3 = centroids[1]
            else:
                beacon2 = centroids[1]
                beacon3 = centroids[0]

        elif i_min == 1:
            self.get_logger().warn("1")
            beacon1 = centroids[0]
            if np.cross(np.array([centroids[1].point.x, centroids[1].point.y]), 
                        np.array([centroids[2].point.x, centroids[2].point.y])) < 0:
                beacon2 = centroids[1]
                beacon3 = centroids[2]
            else:
                beacon2 = centroids[2]
                beacon3 = centroids[1]
        else:
            self.get_logger().warn("2")
            beacon1 = centroids[1]
            if np.cross(np.array([centroids[0].point.x, centroids[0].point.y]), 
                        np.array([centroids[2].point.x, centroids[2].point.y])) < 0:
                beacon2 = centroids[0]
                beacon3 = centroids[1]
            else:
                beacon2 = centroids[1]
                beacon3 = centroids[0]


        self.get_logger().info(f"{centroids[0].point.x} {centroids[0].point.y} {BEACON_1_POS.point.x} {BEACON_1_POS.point.y}")
        self.get_logger().info(f"{centroids[1].point.x} {centroids[1].point.y} {BEACON_2_POS.point.x} {BEACON_2_POS.point.y}")
        self.get_logger().info(f"{centroids[2].point.x} {centroids[2].point.y} {BEACON_3_POS.point.x} {BEACON_3_POS.point.y}")

        
        vector_real_12 = np.array([
            BEACON_2_POS.point.x-BEACON_1_POS.point.x,
            BEACON_2_POS.point.y-BEACON_1_POS.point.y])
            
        vector_12 = np.array([
            beacon2.point.x-beacon1.point.x,
            beacon2.point.y-beacon1.point.y])

        self.get_logger().error(f"{vector_12} {vector_real_12}")
        # Trouver l'angle de rotation à partir des vecteurs (produit scalaire + atan2)
        dot_product = vector_12[0] * vector_real_12[0] + vector_12[1] * vector_real_12[1]
        cross_product = vector_12[0] * vector_real_12[1] - vector_12[1] * vector_real_12[0]
        theta = -np.arctan2(cross_product, dot_product)  # angle de rotation
        A = vector_real_12
        B = vector_12

        # c = (A[0]*B[0]+A[1]*B[1]) / (math.sqrt(A[0]**2+B[0]**2)*math.sqrt(A[1]**2+B[1]**2))
        # theta = math.acos(c )


        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])


        D = np.array([
             BEACON_1_POS.point.x - beacon1.point.x,
             BEACON_1_POS.point.y - beacon1.point.y
        ])



        robot_pose = np.array([
            D[0] - BEACON_1_POS.point.x,
            D[1] - BEACON_1_POS.point.y
        ]).dot(rotation_matrix) + np.array([
            BEACON_1_POS.point.x,
            BEACON_1_POS.point.y
        ])

        # self.get_logger().warn(f"{robot_pose} \t\t {theta*180/math.pi}")


        POSE_DU_ROBOT = PoseStamped()
        POSE_DU_ROBOT.pose.position.x = robot_pose[0]
        POSE_DU_ROBOT.pose.position.y = robot_pose[1]
        POSE_DU_ROBOT.pose.orientation.z = sin(theta / 2)
        POSE_DU_ROBOT.pose.orientation.w = cos(theta / 2)

        return [beacon1,beacon2,beacon3], POSE_DU_ROBOT

    def norm(self, a:np.array):
        return math.sqrt(math.pow(a[0],2)+math.pow(a[1],2))
    
    def calculate_robot_position(self, beacons: list[PointStamped]):
        """
        Calcule la position du robot par triangulation à partir des positions des 3 balises.
        
        :param beacons: Liste de 3 Points correspondant aux positions des balises. Si une balise est absente, la valeur est None.
        :return: Point correspondant à la position triangulée du robot, ou None si triangulation impossible.
        """
        # Vérifier que les trois balises sont présentes
        if any(beacon is None for beacon in beacons):
            return None
        
        # Extraire les positions des balises
        beacon_1 = beacons[0]
        beacon_2 = beacons[1]
        beacon_3 = beacons[2]

        # Les distances entre le robot et chaque balise (on suppose que ces distances sont disponibles)
        d1 = np.sqrt((self.last_robot_pose.pose.position.x - beacon_1.point.x) ** 2 + (self.last_robot_pose.pose.position.y - beacon_1.point.y) ** 2)
        d2 = np.sqrt((self.last_robot_pose.pose.position.x - beacon_2.point.x) ** 2 + (self.last_robot_pose.pose.position.y - beacon_2.point.y) ** 2)
        d3 = np.sqrt((self.last_robot_pose.pose.position.x - beacon_3.point.x) ** 2 + (self.last_robot_pose.pose.position.y - beacon_3.point.y) ** 2)
        

        # Triangulation pour estimer la position du robot
        # Utiliser une méthode de triangulation classique basée sur les distances
        A = 2 * (beacon_2.point.x - beacon_1.point.x)
        B = 2 * (beacon_2.point.y - beacon_1.point.y)
        C = d1**2 - d2**2 - beacon_1.point.x**2 - beacon_1.point.y**2 + beacon_2.point.x**2 + beacon_2.point.y**2

        D = 2 * (beacon_3.point.x - beacon_2.point.x)
        E = 2 * (beacon_3.point.y - beacon_2.point.y)
        F = d2**2 - d3**2 - beacon_2.point.x**2 - beacon_2.point.y**2 + beacon_3.point.x**2 + beacon_3.point.y**2

        # Résoudre le système d'équations linéaires pour trouver x et y
        denom = (A * E - B * D)
        if denom == 0:
            return None  # Cas de dégénérescence où les balises sont alignées

        robot_x = (C * E - B * F) / denom
        robot_y = (A * F - C * D) / denom

        # Créer un Point avec la position triangulée du robot
        robot_position = Point(x=robot_x, y=robot_y, z=0.0)

        return robot_position

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


    def dist_squared(self, A:PointStamped, B:PointStamped) -> float:
        return pow(A.point.x - B.point.x, 2) + pow(A.point.y - B.point.y, 2)
    
    def find_three_beacons(self, last_scan: LaserScan):
        pass
        # # on transforme les positions des 3 beacons dans le repère du laser
        # BEACON_1_POS_stamped = PointStamped()
        # BEACON_1_POS_stamped.point = BEACON_1_POS
        # BEACON_1_POS_stamped.header.frame_id = "odom"
        # BEACON_2_POS_stamped = PointStamped()
        # BEACON_2_POS_stamped.point = BEACON_2_POS
        # BEACON_2_POS_stamped.header.frame_id = "odom"
        # BEACON_3_POS_stamped = PointStamped()
        # BEACON_3_POS_stamped.point = BEACON_3_POS
        # BEACON_3_POS_stamped.header.frame_id = "odom"

        # try:
        #     BEACON_1_POS_in_laser = self.tf_buffer.transform(BEACON_1_POS_stamped, 'base_laser')
        #     BEACON_2_POS_in_laser = self.tf_buffer.transform(BEACON_2_POS_stamped, 'base_laser')
        #     BEACON_3_POS_in_laser = self.tf_buffer.transform(BEACON_3_POS_stamped, 'base_laser')
        # except Exception as e:
        #     self.get_logger().info(f"{e}")
        #     return
        
        # # parcours du scan
        # closests_points_of_beacons = [None, None, None] # beacons 1,2,3
        # closests_dists_of_beacons = [inf, inf, inf] # beacons 1,2,3

        # angle = 0
        # for i in range(len(last_scan.ranges)):
        #     if last_scan.intensities[i] >= self.min_intensity:
        #         point_in_scan = PointStamped()
        #         point_in_scan.header.frame_id = 'base_laser'
        #         point_in_scan.point.x = last_scan.ranges[i] * cos(angle)
        #         point_in_scan.point.y = last_scan.ranges[i] * sin(angle)

        #         if self.dist_squared(point_in_scan, BEACON_1_POS_in_laser) < closests_dists_of_beacons[0]:
        #             closests_dists_of_beacons[0] = self.dist_squared(point_in_scan, BEACON_1_POS_in_laser)
        #             closests_points_of_beacons[0] = point_in_scan

        #         if self.dist_squared(point_in_scan, BEACON_2_POS_in_laser) < closests_dists_of_beacons[1]:
        #             closests_dists_of_beacons[1] = self.dist_squared(point_in_scan, BEACON_2_POS_in_laser)
        #             closests_points_of_beacons[1] = point_in_scan

        #         if self.dist_squared(point_in_scan, BEACON_3_POS_in_laser) < closests_dists_of_beacons[2]:
        #             closests_dists_of_beacons[2] = self.dist_squared(point_in_scan, BEACON_2_POS_in_laser)
        #             closests_points_of_beacons[2] = point_in_scan

        #     angle += self.last_scan.angle_increment

        # try:
        #     closests_points_of_beacons[0] = self.tf_buffer.transform(closests_points_of_beacons[0], 'odom')
        #     closests_points_of_beacons[1] = self.tf_buffer.transform(closests_points_of_beacons[1], 'odom')
        #     closests_points_of_beacons[2] = self.tf_buffer.transform(closests_points_of_beacons[2], 'odom')        
        # except Exception as e:
        #     self.get_logger.info(f"{e}")
        #     return
    
        # return closests_points_of_beacons

    def compute(self, last_scan: LaserScan):
        clusters = self.extract_clusters(last_scan)
       
        # self.get_logger().info(f"{len(clusters)}")
        centroids = self.compute_and_merge_centroids(clusters)
        if len(centroids) != 3:
            self.get_logger().warn(f"{len(centroids)}")
            return

        # self.get_logger().info(f"centroids_count={len(centroids)}")

        beacons_position, robot_position = self.match_centroids_to_beacons(centroids)
        # if beacons_position is None:
        #     self.get_logger().warn(f"beacons_position is None")
        #     return
        # beacons_position = self.find_three_beacons(last_scan)

        # robot_position: Point = self.calculate_robot_position(beacons_position)
        self.publish_circles(beacons_position, robot_position)
        # self.get_logger().warn(f"robot pos {robot_position.pose.position.x} {robot_position.pose.position.y}")

        # self.publish_robot_pose(robot_position)

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