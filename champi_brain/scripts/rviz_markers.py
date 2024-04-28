import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class CircleArrayPublisher(Node):
    def __init__(self, points):
        super().__init__('circle_array_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Liste des points à utiliser
        self.points = points
        
        # Publier les cercles dans un MarkerArray
        self.timer = self.create_timer(1.0, self.publish_circles)

    def publish_circles(self):
        marker_array = MarkerArray()
        
        for i, point in enumerate(self.points):
            marker = Marker()
            marker.header.frame_id = 'odom'  # Cadre de référence
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position du marqueur
            marker.pose.position = point

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


def main(args=None):
    rclpy.init(args=args)
    
    # Liste de points où créer des cercles autour des plantes

# actions = [
    # {"name":"plantes1","type":"prendre_plantes","pose": (1000-300, 1500-500),    "time": 10, "points": 0},
    # {"name":"plantes2","type":"prendre_plantes","pose": (1000-500, 1500),        "time": 10, "points": 0},
    # {"name":"plantes3","type":"prendre_plantes","pose": (1000-300, 1500+500),    "time": 10, "points": 0},
    # {"name":"plantes4","type":"prendre_plantes","pose": (1000+300, 1500-500),    "time": 10, "points": 0},
    # {"name":"plantes6","type":"prendre_plantes","pose": (1000+500, 1500),        "time": 10, "points": 0},
    # {"name":"plantes5","type":"prendre_plantes","pose": (1000+300, 1500+500),    "time": 10, "points": 0},

    points = [
        Point(x=1000.0-300.0, y=1500.0-500.0,z=0.0),
        Point(x=1000.0-500.0, y=1500.0,z=.0),
        Point(x=1000.0-300.0, y=1500.0+500.0,z=0.0),
        Point(x=1000.0+300.0, y=1500.0-500.0,z=0.0),
        Point(x=1000.0+500.0, y=1500.0,z=.0),
        Point(x=1000.0+300.0, y=1500.0+500.0,z=0.00)
    ]

    # convert to mm
    for point in points:
        point.x /= 1000.0
        point.y /= 1000.0
        point.z /= 1000.0
    
    node = CircleArrayPublisher(points)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
