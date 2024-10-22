import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
import math

def laserscan_to_cartesian(scan_msg: LaserScan):
    """
    Convertit les données de LaserScan en coordonnées cartésiennes (x, y).
    """
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
    points = []

    for r, theta in zip(scan_msg.ranges, angles):
        if scan_msg.range_min < r < scan_msg.range_max:
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            points.append((x, y))

    return np.array(points)

def create_binary_image(points, resolution=1000, scale=200):
    """
    Crée une image binaire à partir des points 2D.
    """
    img = np.zeros((resolution, resolution), dtype=np.uint8)

    # Transformer les points pour les faire rentrer dans l'image
    for point in points:
        x, y = int(point[0] * scale + resolution // 2), int(point[1] * scale + resolution // 2)
        if 0 <= x < resolution and 0 <= y < resolution:
            img[y, x] = 255

    return img

def detect_circles(img, beacon_diameter: float, diameter_margin:float):
    """
    Détecte les cercles à l'aide de la transformée de Hough.
    """
    dp = 1  # Inverse ratio of resolution
    min_dist = 20  # Minimum distance between detected centers
    
    beacon_diameter = 0.04
    diameter_margin = 0.01
    scale = 200
    # Calcul des marges de rayon basé sur le diamètre de la balise
    min_radius = int((beacon_diameter / 2 - diameter_margin) * scale)
    max_radius = int((beacon_diameter / 2 + diameter_margin) * scale)
    min_dist = 0.4

    img = cv2.GaussianBlur(img, (9, 9), 2)
    # Utilisation de la transformée de Hough pour détecter les cercles
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, int(min_dist*100),
                                param1=1, param2=5,
                                minRadius=min_radius, maxRadius=max_radius)

    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    return circles

def show_image(img, circles=None):
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    # Si des cercles sont détectés, les dessiner sur l'image
    if circles is not None:
        for circle in circles[0, :]:
            center = (circle[0], circle[1])  # centre du cercle
            radius = circle[2]  # rayon du cercle
            
            # Dessiner le contour du cercle (en vert)
            cv2.circle(img, (int(center[0]), int(center[1])), int(radius), (0, 255, 0), 2)
            # Dessiner le centre du cercle (en rouge)
            cv2.circle(img, (int(center[0]), int(center[1])), 2, (0, 0, 255), 3)

    # Afficher l'image avec les cercles détectés
    cv2.imshow('Detected Circles', img)
    cv2.waitKey(0)  # Attendre une touche pour fermer la fenêtre
    cv2.destroyAllWindows()  # Fermer la fenêtre une fois l'image fermée
