import cv2
import numpy as np
import cv2.aruco as aruco
import tf_transformations as tf_trans
from typing import Tuple, List, Optional, Dict


class WatchtowerRobotLocalizer:
    """Localize robots using ArUco markers detected by a watchtower camera."""
    
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, 
                 marker_length: float = 0.07, aruco_dict_type=aruco.DICT_4X4_50,
                 is_simu_with_webots: bool = False):
        """
        Initialize the localizer.
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            marker_length: Size of ArUco markers in meters (default: 0.07 = 7cm)
            aruco_dict_type: ArUco dictionary type
            is_simu_with_webots: If True, apply coordinate transformation from OpenCV to Webots frame
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length
        
        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
        self.detector_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        
        # Transformation from OpenCV camera frame to Webots camera frame (or identity for real camera)
        if is_simu_with_webots:
            # OpenCV: Z+ forward, X+ right, Y+ down
            # Webots: X+ forward, Y+ left, Z+ up
            self.T_webots_opencv = np.array([
                [0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])
        else:
            # No transformation needed - axes are already aligned
            self.T_webots_opencv = np.eye(4)
        
    def set_camera_pose(self, camera_pos: np.ndarray, camera_quat: np.ndarray):
        """
        Set the camera pose in world frame.
        
        Args:
            camera_pos: Camera position in world frame (x, y, z)
            camera_quat: Camera orientation as quaternion (x, y, z, w)
        """
        self.T_world_camera = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(camera_pos),
            tf_trans.quaternion_matrix(camera_quat)
        )
        
    def set_camera_pose_from_transforms(self, support_pos: np.ndarray, support_quat: np.ndarray,
                                       camera_in_support_pos: np.ndarray, 
                                       camera_in_support_quat: np.ndarray):
        """
        Set camera pose from support and camera-in-support transforms.
        
        Args:
            support_pos: Support position in world frame
            support_quat: Support orientation quaternion (x, y, z, w)
            camera_in_support_pos: Camera position relative to support
            camera_in_support_quat: Camera orientation relative to support (x, y, z, w)
        """
        T_world_support = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(support_pos),
            tf_trans.quaternion_matrix(support_quat)
        )
        T_support_camera = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(camera_in_support_pos),
            tf_trans.quaternion_matrix(camera_in_support_quat)
        )
        self.T_world_camera = T_world_support @ T_support_camera
        
    def detect_markers(self, image: np.ndarray, 
                      id_filter: Optional[Tuple[int, int]] = None) -> Tuple[List, np.ndarray, List]:
        """
        Detect ArUco markers in an image.
        
        Args:
            image: Grayscale or BGR image
            id_filter: Optional tuple (min_id, max_id) to filter detected markers
            
        Returns:
            Tuple of (corners, ids, rejected_corners)
        """
        corners, ids, rejected = self.detector.detectMarkers(image)
        
        if id_filter is not None and ids is not None:
            min_id, max_id = id_filter
            valid_indices = [i for i, marker_id in enumerate(ids) 
                           if min_id <= marker_id[0] <= max_id]
            corners = [corners[i] for i in valid_indices]
            ids = ids[valid_indices]
            
        return corners, ids, rejected
        
    def estimate_marker_pose(self, corners: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate the pose of a single marker.
        
        Args:
            corners: Marker corners
            
        Returns:
            Tuple of (rvec, tvec)
        """
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
        )
        return rvec[0], tvec[0]
        
    def marker_pose_to_world(self, rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Transform marker pose from camera frame to world frame.
        
        Args:
            rvec: Rotation vector in camera frame
            tvec: Translation vector in camera frame
            
        Returns:
            Tuple of (position, quaternion) in world frame
        """
        # Convert rotation vector to rotation matrix
        R_marker_camera_opencv, _ = cv2.Rodrigues(rvec)
        t_marker_camera_opencv = tvec.flatten()
        
        # Build transformation matrix in OpenCV frame
        T_camera_opencv_marker = np.eye(4)
        T_camera_opencv_marker[:3, :3] = R_marker_camera_opencv
        T_camera_opencv_marker[:3, 3] = t_marker_camera_opencv
        
        # Transform to Webots frame then to world frame
        T_camera_webots_marker = self.T_webots_opencv @ T_camera_opencv_marker
        T_world_marker = self.T_world_camera @ T_camera_webots_marker
        
        # Extract position and orientation
        position = tf_trans.translation_from_matrix(T_world_marker)
        quaternion = tf_trans.quaternion_from_matrix(T_world_marker)
        
        return position, quaternion
        
    def localize_robots(self, image: np.ndarray, 
                       id_filter: Optional[Tuple[int, int]] = (0, 10)) -> Dict[int, Dict]:
        """
        Detect and localize all robots in the image.
        
        Args:
            image: Input image (grayscale or BGR)
            id_filter: Optional tuple (min_id, max_id) to filter marker IDs
            
        Returns:
            Dictionary mapping marker_id to {position, quaternion, rvec, tvec}
        """
        corners, ids, _ = self.detect_markers(image, id_filter)
        
        results = {}
        if ids is not None:
            for i, marker_id in enumerate(ids):
                rvec, tvec = self.estimate_marker_pose(corners[i])
                position, quaternion = self.marker_pose_to_world(rvec, tvec)
                
                results[int(marker_id[0])] = {
                    'position': position,
                    'quaternion': quaternion,
                    'rvec': rvec,
                    'tvec': tvec,
                    'corners': corners[i]
                }
                
        return results
        
    def visualize_detections(self, image: np.ndarray, 
                           id_filter: Optional[Tuple[int, int]] = (0, 10),
                           draw_axes: bool = True,
                           draw_corner_numbers: bool = False) -> np.ndarray:
        """
        Visualize detected markers on the image.
        
        Args:
            image: Input image (grayscale or BGR)
            id_filter: Optional tuple (min_id, max_id) to filter marker IDs
            draw_axes: Whether to draw coordinate axes
            draw_corner_numbers: Whether to number the corners
            
        Returns:
            Image with visualizations
        """
        # Convert to BGR if grayscale
        if len(image.shape) == 2:
            img_vis = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            img_vis = image.copy()
            
        corners, ids, _ = self.detect_markers(image, id_filter)
        
        if ids is not None and len(corners) > 0:
            aruco.drawDetectedMarkers(img_vis, corners, ids)
            
            for i, marker_id in enumerate(ids):
                if draw_axes:
                    rvec, tvec = self.estimate_marker_pose(corners[i])
                    cv2.drawFrameAxes(img_vis, self.camera_matrix, self.dist_coeffs,
                                    rvec, tvec, self.marker_length * 0.5)
                
                if draw_corner_numbers:
                    corner_points = corners[i][0].astype(int)
                    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
                    for j, (pt, color) in enumerate(zip(corner_points, colors)):
                        cv2.circle(img_vis, tuple(pt), 8, color, -1)
                        cv2.putText(img_vis, str(j), tuple(pt + 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
        return img_vis


def compute_localization_error(estimated_pos: np.ndarray, true_pos: np.ndarray,
                              estimated_quat: np.ndarray, true_quat: np.ndarray) -> Tuple[float, float]:
    """
    Compute position and orientation errors.
    
    Args:
        estimated_pos: Estimated position (x, y, z)
        true_pos: Ground truth position (x, y, z)
        estimated_quat: Estimated quaternion (x, y, z, w)
        true_quat: Ground truth quaternion (x, y, z, w)
        
    Returns:
        Tuple of (position_error_m, angle_error_deg)
    """
    # Position error (Euclidean distance)
    position_error = np.linalg.norm(estimated_pos - true_pos)
    
    # Orientation error (angle between quaternions)
    q_true_inv = tf_trans.quaternion_inverse(true_quat)
    q_error = tf_trans.quaternion_multiply(q_true_inv, estimated_quat)
    angle_error = 2 * np.arccos(np.clip(abs(q_error[3]), 0, 1))
    angle_error_deg = np.degrees(angle_error)
    
    return position_error, angle_error_deg
