#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyueye_api as pyueye_api
#import champi_camera_yolo.pyueye_api as pyueye_api

# Exemple de fonction get_image
def get_image():
    # Vous pouvez implémenter cette fonction selon vos besoins
    # Voici un exemple basique qui utilise OpenCV pour lire une caméra
    import cv2
    cap = cv2.VideoCapture(0)  # Utilise la caméra par défaut
    ret, frame = cap.read()  # Lit une image de la caméra
    cap.release()  # Libère la caméra immédiatement après la lecture
    return ret, frame

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_ = self.create_publisher(Image, "camera_images", 10)  # Le topic où les images seront publiées
        self.timer = self.create_timer(0.1, self.capture_and_publish)  # Capture toutes les 100 ms
        self.bridge = CvBridge()

    def capture_and_publish(self):
        # Appel de la fonction get_image
        frame = pyueye_api.get_image() # 8UC4 images
        
        # Convertit l'image OpenCV en message ROS
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgra8")
        # # Publie l'image sur le topic
        self.publisher_.publish(ros_image)
    

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)

    pyueye_api.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
