import cv2
import numpy as np
from typing import Tuple

import tf_transformations as tf_trans
import transforms3d.affines as affines
import transforms3d.quaternions as quat
from scipy.spatial.transform import Rotation as R

class WatchtowerExtrinsicCalibrator:
    """Calibrate watchtower camera extrinsic parameters using SIFT feature matching and PnP."""
    
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, is_simu_with_webots: bool,
                 table_width: float = 3.0, table_height: float = 2.0,
                 sift_ratio_threshold: float = 0.75,
                 ransac_threshold: float = 5.0):
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
        self.is_simu_with_webots = is_simu_with_webots
        
        # Initialize SIFT detector
        self.sift = cv2.SIFT_create()
        
        # Initialize BFMatcher
        self.matcher = cv2.BFMatcher()
        
        # Transformation from Webots camera frame to OpenCV camera frame
        # OpenCV: Z+ forward (optical axis), X+ right, Y+ down
        # Webots: X+ forward (optical axis), Y+ left, Z+ up
        self.R_webots_to_opencv = np.array([
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

        # # X points right (0 at left edge) # TODO this should be better no?
        # x_m = (pixel_points[:, 0]) * (self.table_width / img_width)
        # # Y points up (0 at bottom edge)
        # y_m = (img_height - pixel_points[:, 1]) * (self.table_height / img_height)

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
    
    def compute_camera_in_world_transform(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """
        Compute camera position and orientation in world frame from PnP results.
        
        Args:
            rvec: Rotation vector from solvePnP (world to camera)
            tvec: Translation vector from solvePnP (world to camera)
            
        Returns:
            M_T_camera_to_world: Transformation matrix (world to camera)
        """
        # Convert rotation vector to matrix
        R_world_to_opencv, _ = cv2.Rodrigues(rvec)
        
        # Camera position in world frame (invert the transformation)
        M_t_camera_to_world = -R_world_to_opencv.T @ tvec # Using R_world_to_opencv or R_webots is the same for the position
        M_t_camera_to_world = M_t_camera_to_world.flatten()

        if self.is_simu_with_webots:
            # Transform rotation to Webots convention
            # R_world_to_opencv is the rotation from world to camera in OpenCV frame
            # We want camera orientation in world frame with Webots axes
            # Correct composition: transform axes first, then invert rotation
            R_webots = R_world_to_opencv.T @ self.R_webots_to_opencv

            M_R_camera_to_world = R_webots
        else:
            M_R_camera_to_world = R_world_to_opencv.T

        # Correction matrix: 180 degree rotation around World Z-axis
        R_z_180 = np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]
        ])

        # Apply correction to the computed camera orientation
        M_R_camera_to_world = R_z_180 @ M_R_camera_to_world

        M_Z_neutral = np.ones_like(M_t_camera_to_world) # just to pass this arg
        M_T_camera_to_world = affines.compose(M_t_camera_to_world, M_R_camera_to_world, M_Z_neutral)
        return M_T_camera_to_world
    
    def calibrate(self, img_table: np.ndarray, img_camera: np.ndarray) -> dict:
        """
        Perform full extrinsic calibration pipeline.
        
        Args:
            img_table: Grayscale table reference image (top-down view)
            img_camera: Grayscale camera image
            
        Returns:
            Dictionary with calibration results: # TODO MAJ DOC
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
        M_T_camera_to_world = self.compute_camera_in_world_transform(rvec, tvec)

        # Step 5: Compute reprojection error
        projected_pts, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        repr_error = np.mean(np.linalg.norm(image_points - projected_pts.squeeze(), axis=1))

        return {
            'success': True,
            'M_T_camera_to_world': M_T_camera_to_world,
            'rvec': rvec,
            'tvec': tvec,
            'num_inliers': num_inliers,
            'reprojection_error': repr_error,
            'object_points': object_points,
            'image_points': image_points
        }

def compute_calibration_error(M_T_estimated: np.ndarray, M_T_true: np.ndarray) -> Tuple[float, float]:
    """
    Compute position and orientation errors between estimated and true camera pose.
    
    Args:
        estimated_pos: Estimated position (x, y, z)
        true_pos: Ground truth position (x, y, z) # TODO update DOC
        estimated_quat: Estimated quaternion (x, y, z, w)
        true_quat: Ground truth quaternion (x, y, z, w)
        
    Returns:
        Tuple of (position_error_m, angle_error_deg)
    """
    M_t_estimated, M_R_estimated, _, _ = affines.decompose(M_T_estimated)
    M_t_true, M_R_true, _, _ = affines.decompose(M_T_true)

    quat_estimated_xyzw = R.from_matrix(M_R_estimated).as_quat()
    quat_true_xyzw = R.from_matrix(M_R_true).as_quat()

    # Position error (Euclidean distance)
    position_error = np.linalg.norm(M_t_estimated - M_t_true)
    
    # Orientation error (angle between quaternions)
    quat_true_inv = tf_trans.quaternion_inverse(quat_true_xyzw)
    quat_error = tf_trans.quaternion_multiply(quat_true_inv, quat_estimated_xyzw)
    angle_error = 2 * np.arccos(np.clip(abs(quat_error[3]), 0, 1))
    
    angle_error_deg = np.degrees(angle_error)
    
    return position_error, angle_error_deg


def compute_axis_alignment_error(R_estimated: np.ndarray, R_true: np.ndarray, axis: np.ndarray) -> float:
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
