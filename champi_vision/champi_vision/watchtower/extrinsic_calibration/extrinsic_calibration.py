"""
Utilities for extrinsic calibration and green square detection.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt


def show(title, img, cmap=None, size=6):
    """Display an image with matplotlib"""
    plt.figure(figsize=(size, size))
    if len(img.shape) == 2:
        plt.imshow(img, cmap=cmap)
    else:
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title(title)
    plt.axis("off")
    plt.show()


def order_points(pts):
    """
    Order 4 points of a quadrilateral into [TL, TR, BR, BL]
    
    Args:
        pts: Nx2 array of points
        
    Returns:
        4x2 ordered array [TL, TR, BR, BL]
    """
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # TL
    rect[2] = pts[np.argmax(s)]  # BR

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # TR
    rect[3] = pts[np.argmax(diff)]  # BL

    return rect


def find_full_quad(contour1, contour2,
                   epsilon_ratio=0.02,
                   min_area_ratio=0.2,
                   min_side_ratio=0.2):
    """
    Reconstruct a plausible quadrilateral (square projection) from 2 contours.
    
    Args:
        contour1: Nx2 array of points from first contour
        contour2: Nx2 array of points from second contour
        epsilon_ratio: Ratio for cv2.approxPolyDP (default 0.02)
        min_area_ratio: Minimum area ratio (default 0.2)
        min_side_ratio: Minimum side ratio (default 0.2)
        
    Returns:
        4x2 ordered array (TL, TR, BR, BL) or None if invalid
    """

    # 1. Merge
    pts = np.vstack((contour1, contour2))

    # 2. Convex hull
    hull = cv2.convexHull(pts)

    # 3. Approx poly
    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, epsilon_ratio * peri, True)

    # 4. Adjust epsilon
    for eps in np.linspace(epsilon_ratio, 0.1, 10):
        approx = cv2.approxPolyDP(hull, eps * peri, True)
        if len(approx) == 4:
            break

    if len(approx) != 4:
        return None

    # Reshape
    quad = approx.reshape(4, 2).astype(np.float32)

    # 5. Strict convexity
    if not cv2.isContourConvex(quad.reshape(-1, 1, 2)):
        return None

    # 6. Minimum area
    quad_area = cv2.contourArea(quad)
    hull_area = cv2.contourArea(hull)

    if hull_area == 0 or quad_area / hull_area < min_area_ratio:
        return None

    # 7. Order points
    quad = order_points(quad)

    # 8. Side lengths
    sides = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)

    if np.min(sides) / np.max(sides) < min_side_ratio:
        return None

    # 9. Internal angles (avoid degenerate triangle)
    for i in range(4):
        p0 = quad[i - 1]
        p1 = quad[i]
        p2 = quad[(i + 1) % 4]

        v1 = p0 - p1
        v2 = p2 - p1

        cosang = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.degrees(np.arccos(np.clip(cosang, -1, 1)))

        if angle < 20 or angle > 160:
            return None

    return quad


def detect_green_squares(img, lower_green=None, upper_green=None):
    """
    Detect green squares in an image.
    
    Args:
        img: BGR image
        lower_green: Lower HSV bound for green (default [40, 80, 80])
        upper_green: Upper HSV bound for green (default [85, 255, 255])
        
    Returns:
        Tuple (squares, squares_cnts) where:
        - squares: List of 4x2 arrays (ordered points TL, TR, BR, BL)
        - squares_cnts: List of contours in OpenCV format
    """
    if lower_green is None:
        lower_green = np.array([40, 80, 80])
    if upper_green is None:
        upper_green = np.array([85, 255, 255])
    
    # HSV conversion
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Green threshold
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Morphology
    kernel = np.ones((1, 1), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)
    
    # Contour detection
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Contour approximation
    approx_contours = []
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.004 * peri, True)
        area = cv2.contourArea(approx)
        if area >= 100:
            approx_contours.append(approx)
    
    # Square detection by contour pairs
    from itertools import combinations
    
    squares = []
    squares_cnts = []
    
    if len(approx_contours) >= 2:
        for i, j in combinations(range(len(approx_contours)), 2):
            cnt1 = approx_contours[i].reshape(-1, 2)
            cnt2 = approx_contours[j].reshape(-1, 2)
            
            square = find_full_quad(cnt1, cnt2)
            if square is not None:
                try:
                    square_cnt = square.astype(np.int32).reshape((-1, 1, 2))
                except Exception:
                    square_cnt = np.array(square, dtype=np.int32).reshape((-1, 1, 2))
                
                area = cv2.contourArea(square_cnt)
                if 500 <= area <= 10000:
                    squares.append(square)
                    squares_cnts.append(square_cnt)
    
    return squares, squares_cnts


def estimate_square_poses(
    squares,
    square_size,
    camera_matrix,
    dist_coeffs,
):
    """
    Estimate pose (rvec, tvec) for each detected square.
    
    Args:
        squares: List of 4x2 ordered square arrays (TL, TR, BR, BL)
        square_size: Real square size in meters (default 0.1)
        camera_matrix: 3x3 camera intrinsic matrix.
        dist_coeffs: Distortion coefficients
        
    Returns:
        List of dicts with 'id', 'rvec', 'tvec', 'imagePoints'
    """
    cameraMatrix = np.array(camera_matrix, dtype=np.float32).reshape(3, 3)
    
    # 3D square points on z=0 plane
    markerPoints = np.array([
        [-square_size/2, square_size/2, 0],   # TL
        [square_size/2, square_size/2, 0],    # TR
        [square_size/2, -square_size/2, 0],   # BR
        [-square_size/2, -square_size/2, 0]   # BL
    ], dtype=np.float32)
    
    poses = []
    for idx, sq in enumerate(squares):
        imagePoints = sq.astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            markerPoints,
            imagePoints,
            cameraMatrix,
            dist_coeffs,
            useExtrinsicGuess=False,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        # 180-degree rotation correction on z-axis
        # R, _ = cv2.Rodrigues(rvec)
        # R_z180 = np.array([
        #     [-1, 0, 0],
        #     [0, -1, 0],
        #     [0, 0, 1]
        # ])
        # R_corrected = R_z180 @ R
        # rvec, _ = cv2.Rodrigues(R_corrected)
        
        if success:
            poses.append({
                'id': idx,
                'rvec': rvec,
                'tvec': tvec,
                'imagePoints': imagePoints
            })
    
    return poses


def draw_poses(img, poses, cameraMatrix, dist_coeffs):
    """
    Draw reference axes for each detected pose.
    
    Args:
        img: BGR image to modify
        poses: List of dicts with 'id', 'rvec', 'tvec', 'imagePoints'
        cameraMatrix: Camera matrix
        dist_coeffs: Distortion coefficients
        
    Returns:
        Modified image with drawn axes
    """
    result = img.copy()
    
    for pose in poses:
        rvec = pose['rvec']
        tvec = pose['tvec']
        idx = pose['id']
        
        # Draw axes (red=X, green=Y, blue=Z)
        cv2.drawFrameAxes(result, cameraMatrix, dist_coeffs, rvec, tvec, 0.1)
        
        # Label
        c = np.mean(pose['imagePoints'], axis=0).astype(int)
        cv2.putText(result, f"#{idx}", tuple(c), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 2)
    
    return result
