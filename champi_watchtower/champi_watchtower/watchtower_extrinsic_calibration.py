import cv2
import numpy as np
import transforms3d.quaternions as quat
from typing import Tuple

import tf_transformations as tf_trans


class WatchtowerExtrinsicCalibrator:
    """Calibrate watchtower camera extrinsic parameters using SIFT feature matching and PnP."""
    
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 table_width: float = 3.0, table_height: float = 2.0,
                 sift_ratio_threshold: float = 0.75,
                 ransac_threshold: float = 5.0, using_webots: bool = False):
        """
        Initialize the extrinsic calibrator.
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            table_width: Real width of the table in meters (default: 3.0)
            table_height: Real height of the table in meters (default: 2.0)
            sift_ratio_threshold: Ratio threshold for Lowe's ratio test (default: 0.75)
            ransac_threshold: RANSAC reprojection threshold in pixels (default: 5.0)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.table_width = table_width
        self.table_height = table_height
        self.sift_ratio_threshold = sift_ratio_threshold
        self.ransac_threshold = ransac_threshold
        self.using_webots = using_webots
        
        # Initialize SIFT detector
        self.sift = cv2.SIFT_create()
        
        # Initialize BFMatcher
        self.matcher = cv2.BFMatcher()
        
        # Transformation from OpenCV camera frame to Webots camera frame
        # OpenCV: Z+ forward (optical axis), X+ right, Y+ down
        # Webots: X+ forward (optical axis), Y+ left, Z+ up
        self.R_opencv_to_webots = np.array([
            [0, -1, 0],   # X_opencv = -Y_webots
            [0, 0, -1],   # Y_opencv = -Z_webots
            [1, 0, 0]     # Z_opencv = X_webots
        ])
        
    def detect_and_match_features(self, img_table: np.ndarray, img_camera: np.ndarray) -> Tuple[np.ndarray, np.ndarray, int]:
        """
        Detect SIFT features and match them between table and camera images.
        
        Args:
            img_table: Grayscale table image (top-down view)
            img_camera: Grayscale camera image
            
        Returns:
            Tuple of (inlier_src_points, inlier_dst_points, num_inliers)
        """
        # Detect keypoints and compute descriptors
        kp_table, des_table = self.sift.detectAndCompute(img_table, None)
        kp_camera, des_camera = self.sift.detectAndCompute(img_camera, None)
        
        # Match with k-nearest neighbors (k=2 for ratio test)
        matches = self.matcher.knnMatch(des_table, des_camera, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < self.sift_ratio_threshold * n.distance:
                good_matches.append(m)
        
        # Extract matched points
        src_pts = np.float32([kp_table[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_camera[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        # Filter with RANSAC homography
        _, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, self.ransac_threshold)
        
        # Keep only inliers
        inlier_src = src_pts[mask.ravel() == 1].squeeze()
        inlier_dst = dst_pts[mask.ravel() == 1].squeeze()
        
        return inlier_src, inlier_dst, len(inlier_src)
    
    def convert_pixels_to_world_coords(self, pixel_points: np.ndarray, img_width: int, img_height: int) -> np.ndarray:
        """
        Convert pixel coordinates from table image to real-world 3D coordinates.
        
        Args:
            pixel_points: Nx2 array of pixel coordinates (x, y)
            img_width: Width of the table image in pixels
            img_height: Height of the table image in pixels
            
        Returns:
            Nx3 array of world coordinates (x, y, z=0)
        """
        # Convert pixels to meters        
        # Put origin at bottom-left corner. X points right, Y points up
        x_m = (img_width - pixel_points[:, 0]) * (self.table_width / img_width)
        y_m = (pixel_points[:, 1]) * (self.table_height / img_height)

        # Z coordinate is 0 (table plane)
        object_points = np.column_stack((x_m, y_m, np.zeros_like(x_m))).astype(np.float32)
        
        return object_points
    
    def solve_pnp(self, object_points: np.ndarray, image_points: np.ndarray) -> Tuple[bool, np.ndarray, np.ndarray]:
        """
        Solve the Perspective-n-Point problem to estimate camera pose.
        
        Args:
            object_points: Nx3 array of 3D world coordinates
            image_points: Nx2 array of 2D image coordinates
            
        Returns:
            Tuple of (success, rvec, tvec)
        """
        success, rvec, tvec = cv2.solvePnP(
            object_points, 
            image_points, 
            self.camera_matrix, 
            self.dist_coeffs
        )
        
        return success, rvec, tvec
    
    def compute_camera_pose_in_world(self, rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute camera position and orientation in world frame from PnP results.
        
        Args:
            rvec: Rotation vector from solvePnP (world to camera)
            tvec: Translation vector from solvePnP (world to camera)
            
        Returns:
            Tuple of (position, quaternion_xyzw) in world frame, with Webots convention
        """
        # Convert rotation vector to matrix
        R_opencv, _ = cv2.Rodrigues(rvec)
        
        # Camera position in world frame (invert the transformation)
        pos_camera = -R_opencv.T @ tvec
        pos_camera = pos_camera.flatten()
        
        # Transform rotation to Webots convention
        # R_opencv is the rotation from world to camera in OpenCV frame
        # We want camera orientation in world frame with Webots axes
        # Correct composition: transform axes first, then invert rotation
        R_webots = self.R_opencv_to_webots @ R_opencv.T
        
        # Convert to quaternion (w, x, y, z) then reorder to (x, y, z, w)
        quat_wxyz = quat.mat2quat(R_webots)
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        
        return pos_camera, quat_xyzw
    
    def calibrate(self, img_table: np.ndarray, img_camera: np.ndarray) -> dict:
        """
        Perform full extrinsic calibration pipeline.
        
        Args:
            img_table: Grayscale table reference image (top-down view)
            img_camera: Grayscale camera image
            
        Returns:
            Dictionary with calibration results:
                - success: bool
                - position: camera position in world (x, y, z)
                - quaternion: camera orientation (x, y, z, w) in Webots convention
                - rvec: rotation vector from solvePnP
                - tvec: translation vector from solvePnP
                - num_inliers: number of matched features
                - reprojection_error: mean reprojection error in pixels
                - R_camera_webots: 3x3 rotation matrix in Webots convention
        """
        # Step 1: Feature matching
        inlier_src, inlier_dst, num_inliers = self.detect_and_match_features(img_table, img_camera)
        
        if num_inliers < 4:
            return {
                'success': False,
                'error': f'Not enough inliers: {num_inliers} (minimum 4 required)'
            }
        
        # Step 2: Convert to 3D world coordinates
        h_table, w_table = img_table.shape[:2]
        object_points = self.convert_pixels_to_world_coords(inlier_src, w_table, h_table)
        image_points = inlier_dst.astype(np.float32)
        
        # Step 3: Solve PnP
        success, rvec, tvec = self.solve_pnp(object_points, image_points)
        
        if not success:
            return {
                'success': False,
                'error': 'SolvePnP failed to converge'
            }
        
        # Step 4: Compute camera pose in world frame
        position, quaternion = self.compute_camera_pose_in_world(rvec, tvec)
        
        # Step 5: Compute reprojection error
        projected_pts, _ = cv2.projectPoints(object_points, rvec, tvec, 
                                            self.camera_matrix, self.dist_coeffs)
        repr_error = np.mean(np.linalg.norm(image_points - projected_pts.squeeze(), axis=1))
        
        # Get rotation matrix in Webots frame for further analysis
        R_opencv, _ = cv2.Rodrigues(rvec)
        R_webots = R_opencv.T @ self.R_opencv_to_webots
        
        return {
            'success': True,
            'position': position,
            'quaternion': quaternion,
            'rvec': rvec,
            'tvec': tvec,
            'num_inliers': num_inliers,
            'reprojection_error': repr_error,
            'R_camera_webots': R_webots,
            'object_points': object_points,
            'image_points': image_points
        }


def compute_calibration_error(estimated_pos: np.ndarray, true_pos: np.ndarray,
                              estimated_quat: np.ndarray, true_quat: np.ndarray) -> Tuple[float, float]:
    """
    Compute position and orientation errors between estimated and true camera pose.
    
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


def compute_axis_alignment_error(R_estimated: np.ndarray, R_true: np.ndarray, 
                                 axis: np.ndarray) -> float:
    """
    Compute the angular error between a specific axis in two reference frames.
    
    Args:
        R_estimated: Estimated 3x3 rotation matrix
        R_true: Ground truth 3x3 rotation matrix
        axis: 3D unit vector representing the axis to compare (e.g., [1, 0, 0] for X)
        
    Returns:
        Angular error in degrees
    """
    axis_est = R_estimated @ axis
    axis_true = R_true @ axis
    
    cos_theta = np.dot(axis_est, axis_true) / (np.linalg.norm(axis_est) * np.linalg.norm(axis_true))
    angle_error_rad = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    
    return np.degrees(angle_error_rad)
