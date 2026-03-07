#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2, math
from champi_libraries_py.utils import angles
import numpy as np
import yaml
import os
from enum import Enum
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Optional
from PIL import Image as PILImage, ImageTk
from datetime import datetime
import tf_transformations as tf_trans

from champi_vision.watchtower.robot_localization_from_watchtower import WatchtowerRobotLocalizer
from champi_vision.watchtower.watchtower_extrinsic_calibration import WatchtowerExtrinsicCalibrator, compute_calibration_error


class WatchtowerState(Enum):
    """State machine states for watchtower node."""
    INIT = "Initialization"
    CALIBRATION = "Calibration"
    RUNNING = "Running"


def get_team_color(marker_id: int) -> tuple:
    """
    Get team color based on marker ID.
    
    Args:
        marker_id: ArUco marker ID
        
    Returns:
        Tuple of (color_name, color_code, emoji)
        - IDs 1-5: Blue team
        - IDs 6-10: Yellow team
    """
    if 1 <= marker_id <= 5:
        return ("Blue", "#2196F3", "🔵")
    elif 6 <= marker_id <= 10:
        return ("Yellow", "#FFC107", "🟡")
    else:
        return ("Unknown", "#9E9E9E", "⚪")


class WatchtowerGUI:
    """GUI to control watchtower calibration and display robot detections."""
    
    def __init__(self, on_start_calib_callback, on_stop_callback):
        """
        Initialize the GUI.
        
        Args:
            on_start_calib_callback: Function to call when "Start Calibration" is clicked
            on_stop_callback: Function to call when window is closed
        """
        self.on_start_calib = on_start_calib_callback
        self.on_stop = on_stop_callback
        
        self.root = tk.Tk()
        self.root.title("Watchtower Control - Robot Localization")
        self.root.geometry("1400x900")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        # Top frame: State and control
        top_frame = tk.Frame(self.root)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        # State label
        self.state_var = tk.StringVar(value="State: INIT")
        state_label = tk.Label(top_frame, textvariable=self.state_var, 
                              font=("Arial", 14, "bold"))
        state_label.pack(side=tk.LEFT, padx=20)
        
        # Status text
        self.status_var = tk.StringVar(value="Waiting for initialization...")
        status_label = tk.Label(top_frame, textvariable=self.status_var, 
                               font=("Arial", 10), fg="#FF9800")
        status_label.pack(side=tk.LEFT, padx=20)
        
        # Calibration button
        self.calib_button = tk.Button(top_frame, text="Start Calibration", 
                                      command=self._on_calib_clicked,
                                      state=tk.DISABLED, font=("Arial", 11, "bold"),
                                      bg="#4CAF50", fg="white", padx=20, pady=8)
        self.calib_button.pack(side=tk.RIGHT, padx=20)
        
        # Main content area: Image on left, logs on right
        content_frame = tk.Frame(self.root)
        content_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left side: Camera feed and table view
        left_frame = tk.Frame(content_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Top: Camera feed
        camera_frame = tk.LabelFrame(left_frame, text="Camera Feed with Robot Detections", padx=5, pady=5)
        camera_frame.pack(fill=tk.BOTH, expand=True, padx=(0, 0), pady=(0, 5))
        camera_frame.pack_propagate(False)
        
        # Use a canvas for camera image
        self.image_canvas = tk.Canvas(camera_frame, bg="gray30", highlightthickness=0)
        self.image_canvas.pack(fill=tk.BOTH, expand=True)
        
        self.image_label = tk.Label(self.image_canvas, bg="gray30")
        self.image_on_canvas = self.image_canvas.create_window(0, 0, window=self.image_label, anchor=tk.NW)
        
        # Bottom: Table top view
        table_frame = tk.LabelFrame(left_frame, text="Table Top View with Robot Positions", padx=5, pady=5)
        table_frame.pack(fill=tk.BOTH, expand=True, padx=(0, 0), pady=(5, 0))
        table_frame.pack_propagate(False)
        
        # Use a canvas for table image
        self.table_canvas = tk.Canvas(table_frame, bg="gray30", highlightthickness=0)
        self.table_canvas.pack(fill=tk.BOTH, expand=True)
        
        self.table_label = tk.Label(self.table_canvas, bg="gray30")
        self.table_on_canvas = self.table_canvas.create_window(0, 0, window=self.table_label, anchor=tk.NW)
        
        self.current_image = None
        self.current_table_image = None
        self.photo_image = None
        self.table_photo_image = None
        
        # Right side: Info and logs
        right_frame = tk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        
        # Info section
        info_frame = tk.LabelFrame(right_frame, text="Calibration Info", padx=5, pady=5)
        info_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.info_text = tk.Text(info_frame, height=6, width=50, state=tk.DISABLED, 
                                font=("Courier", 9))
        info_text_scroll = tk.Scrollbar(info_frame, command=self.info_text.yview)
        self.info_text.config(yscrollcommand=info_text_scroll.set)
        self.info_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        info_text_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Detections section
        det_frame = tk.LabelFrame(right_frame, text="Robot Detections", padx=5, pady=5)
        det_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.detections_text = tk.Text(det_frame, height=10, width=50, state=tk.DISABLED,
                                      font=("Courier", 9))
        det_scroll = tk.Scrollbar(det_frame, command=self.detections_text.yview)
        self.detections_text.config(yscrollcommand=det_scroll.set)
        self.detections_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        det_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Debug logs section
        logs_frame = tk.LabelFrame(right_frame, text="Debug Logs", padx=5, pady=5)
        logs_frame.pack(fill=tk.BOTH, expand=True)
        
        self.logs_text = tk.Text(logs_frame, height=12, width=50, state=tk.DISABLED,
                               font=("Courier", 8))
        logs_scroll = tk.Scrollbar(logs_frame, command=self.logs_text.yview)
        self.logs_text.config(yscrollcommand=logs_scroll.set)
        self.logs_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        logs_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
    def _on_calib_clicked(self):
        """Handle calibration button click."""
        if self.on_start_calib:
            self.on_start_calib()
            
    def _on_closing(self):
        """Handle window close."""
        if self.on_stop:
            self.on_stop()
        self.root.destroy()
        
    def update_state(self, state: WatchtowerState):
        """Update the displayed state."""
        self.state_var.set(f"State: {state.value}")
        
        # Update button state based on state
        if state == WatchtowerState.INIT:
            self.calib_button.config(state=tk.NORMAL)
        elif state == WatchtowerState.CALIBRATION:
            self.calib_button.config(state=tk.DISABLED)
        elif state == WatchtowerState.RUNNING:
            self.calib_button.config(state=tk.DISABLED, text="Calibrated ✓", bg="#2196F3")
            
    def update_status(self, status: str):
        """Update the status message."""
        self.status_var.set(status)
        
    def update_info(self, info: str):
        """Update the calibration info text area."""
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(1.0, info)
        self.info_text.config(state=tk.DISABLED)
    
    def update_image(self, cv_image: np.ndarray):
        """Update the camera image display (BGR format)."""
        try:
            if cv_image is None:
                return
            
            # Get the canvas dimensions
            self.image_canvas.update()
            canvas_width = self.image_canvas.winfo_width()
            canvas_height = self.image_canvas.winfo_height()
            
            # If canvas hasn't been rendered yet, use defaults
            if canvas_width <= 1:
                canvas_width = 600
            if canvas_height <= 1:
                canvas_height = 400
            
            # Get image dimensions
            h, w = cv_image.shape[:2]
            
            # Calculate scaling to fit in canvas while maintaining aspect ratio
            scale = min(canvas_width / w, canvas_height / h)
            
            # Resize image
            new_w = int(w * scale)
            new_h = int(h * scale)
            display_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
            # Convert BGR to RGB for PIL
            rgb_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            self.photo_image = ImageTk.PhotoImage(pil_image)
            self.image_label.config(image=self.photo_image)
            self.current_image = cv_image
            
            # Update canvas scroll region
            self.image_canvas.itemconfig(self.image_on_canvas, window=self.image_label)
        except Exception as e:
            self.add_log(f"Error updating image: {e}")
    
    def update_table_image(self, table_image: np.ndarray):
        """Update the table top view with robot positions (BGR format)."""
        try:
            if table_image is None:
                return
            
            # Get the canvas dimensions
            self.table_canvas.update()
            canvas_width = self.table_canvas.winfo_width()
            canvas_height = self.table_canvas.winfo_height()
            
            # If canvas hasn't been rendered yet, use defaults
            if canvas_width <= 1:
                canvas_width = 600
            if canvas_height <= 1:
                canvas_height = 400
            
            # Get image dimensions BEFORE resizing
            h_orig, w_orig = table_image.shape[:2]
            
            # Calculate scaling to fit in canvas while maintaining aspect ratio
            scale = min(canvas_width / w_orig, canvas_height / h_orig)
            
            # Resize image
            new_w = int(w_orig * scale)
            new_h = int(h_orig * scale)
            
            # Resize the image for display
            display_image = cv2.resize(table_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
            # Convert BGR to RGB for PIL
            rgb_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            self.table_photo_image = ImageTk.PhotoImage(pil_image)
            self.table_label.config(image=self.table_photo_image)
            self.current_table_image = table_image
            
            # Update canvas
            self.table_canvas.itemconfig(self.table_on_canvas, window=self.table_label)
        except Exception as e:
            self.add_log(f"Error updating table image: {e}")
    
    def update_detections(self, detections_dict: dict):
        """Update the robot detections display."""
        self.detections_text.config(state=tk.NORMAL)
        self.detections_text.delete(1.0, tk.END)
        
        if not detections_dict:
            self.detections_text.insert(1.0, "No robots detected")
        else:
            text = ""
            for marker_id, data in sorted(detections_dict.items()):
                team_name, team_color, emoji = get_team_color(marker_id)
                pos = data['position']
                quat = data['quaternion']
                text += f"\n{emoji} ID: {marker_id}\n"
                text += f"   Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]\n"
                text += f"   Quaternion: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]\n"
            
            self.detections_text.insert(1.0, text)
        
        self.detections_text.config(state=tk.DISABLED)
    
    def add_log(self, message: str):
        """Add a message to the debug logs."""
        self.logs_text.config(state=tk.NORMAL)
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.logs_text.insert(tk.END, log_entry)
        # Keep only last 100 lines
        lines = self.logs_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.logs_text.delete("1.0", "101.0")
        self.logs_text.see(tk.END)
        self.logs_text.config(state=tk.DISABLED)
        
    def update(self):
        """Update the GUI (non-blocking)."""
        try:
            self.root.update()
        except:
            pass
    
    def destroy(self):
        """Destroy the GUI window."""
        try:
            self.root.quit()
            self.root.destroy()
        except:
            pass


class WatchtowerNode(Node):
    """
    ROS2 node for watchtower-based robot localization with extrinsic calibration.
    
    State Machine:
    - INIT: Initialize node, load camera parameters, create GUI
    - CALIBRATION: Perform extrinsic calibration using reference image
    - RUNNING: Continuously localize robots from camera images
    """
    
    def __init__(self):
        super().__init__('watchtower_node')
        
        # Declare parameters
        self.declare_parameter('camera_info_file', '')
        self.declare_parameter('table_reference_image', '')
        self.declare_parameter('marker_length', 0.07)  # 7cm
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('image_topic', '/watchtower/camera/image_color')
        self.declare_parameter('camera_info_topic', '/watchtower/camera/camera_info')
        self.declare_parameter('is_simu_with_webots', False)
        self.declare_parameter('marker_id_min', 0)
        self.declare_parameter('marker_id_max', 10)
        
        # Get parameters
        self.camera_info_file = self.get_parameter('camera_info_file').value
        self.marker_length = self.get_parameter('marker_length').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.is_simu = self.get_parameter('is_simu_with_webots').value
        self.marker_id_min = self.get_parameter('marker_id_min').value
        self.marker_id_max = self.get_parameter('marker_id_max').value
        self.table_ref_image_path = self.get_parameter('table_reference_image').value
        self.table_width = 3.0
        self.table_height = 2.0
        
        # State machine
        self.state = WatchtowerState.INIT
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        
        # Calibration objects
        self.calibrator: Optional[WatchtowerExtrinsicCalibrator] = None
        self.localizer: Optional[WatchtowerRobotLocalizer] = None
        self.table_ref_image = None
        self.table_ref_image_color = None
        
        # Latest image from camera
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # ROS2 publishers and subscribers
        self.pose_array_pub = self.create_publisher(PoseArray, '/watchtower/robot_poses', 10)
        self.state_pub = self.create_publisher(String, '/watchtower/state', 10)
        self.viz_image_pub = self.create_publisher(Image, '/watchtower/visualization', 10)
        
        self.image_sub = None  # Created after calibration
        
        # GUI
        self.gui = None
        self.shutdown_flag = False
        
        # Initialize the node
        self._initialize()
        
    def _initialize(self):
        """Initialize state: Load camera parameters and create GUI."""
        log_msg = "Initializing watchtower node..."
        self.get_logger().info(log_msg)
        
        # Load camera calibration
        if not self._load_camera_calibration():
            self.get_logger().error("Failed to load camera calibration!")
            return
            
        # Load table reference image
        if not self._load_table_reference():
            self.get_logger().error("Failed to load table reference image!")
            return
            
        # Create calibrator
        self.calibrator = WatchtowerExtrinsicCalibrator(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            table_width=self.table_width,
            table_height=self.table_height
        )
        
        # Create localizer (pose will be set after calibration)
        self.localizer = WatchtowerRobotLocalizer(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            marker_length=self.marker_length,
            is_simu_with_webots=self.is_simu
        )
        
        # Create GUI
        self._create_gui()
        
        if self.gui:
            self.gui.add_log("Node initialized successfully")
            self.gui.add_log(f"Camera calibration loaded: {Path(self.camera_info_file).name}")
            self.gui.add_log(f"Table reference image loaded: {self.table_ref_image.shape}")
        
        self.get_logger().info("Initialization complete. Ready for calibration.")
        
    def _load_camera_calibration(self) -> bool:
        """Load camera intrinsic parameters from file."""
        if not self.camera_info_file or not os.path.exists(self.camera_info_file):
            self.get_logger().error(f"Camera info file not found: {self.camera_info_file}")
            return False
            
        try:
            with open(self.camera_info_file, 'r') as f:
                calib_data = yaml.safe_load(f)
                
            # Extract camera matrix
            K = calib_data['camera_matrix']['data']
            self.camera_matrix = np.array(K).reshape(3, 3)
            
            # Extract distortion coefficients
            D = calib_data['distortion_coefficients']['data']
            self.dist_coeffs = np.array(D)
            
            self.get_logger().info(f"Loaded camera calibration from {self.camera_info_file}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading camera calibration: {e}")
            return False
            
    def _load_table_reference(self) -> bool:
        """Load the table reference image for calibration."""
        if not self.table_ref_image_path or not os.path.exists(self.table_ref_image_path):
            self.get_logger().error(f"Table reference image not found: {self.table_ref_image_path}")
            return False
            
        try:
            # Load grayscale for calibration
            self.table_ref_image = cv2.imread(self.table_ref_image_path, cv2.IMREAD_GRAYSCALE)
            if self.table_ref_image is None:
                self.get_logger().error(f"Failed to read image: {self.table_ref_image_path}")
                return False
            
            # Also load in color for visualization
            self.table_ref_image_color = cv2.imread(self.table_ref_image_path, cv2.IMREAD_COLOR)
            if self.table_ref_image_color is None:
                self.get_logger().error(f"Failed to read color image: {self.table_ref_image_path}")
                return False
                
            self.get_logger().info(f"Loaded table reference image: {self.table_ref_image.shape}")
            self.get_logger().info(f"Loaded table reference color image: {self.table_ref_image_color.shape}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading table reference: {e}")
            return False
            
    def _create_gui(self):
        """Create the GUI."""
        self.gui = WatchtowerGUI(
            on_start_calib_callback=self._on_gui_start_calibration,
            on_stop_callback=self._on_gui_stop
        )
        
        self.gui.update_state(self.state)
        self.gui.update_status("Node initialized. Click 'Start Calibration' when ready.")
        self.gui.update_info(f"Camera: {Path(self.camera_info_file).name}\n"
                           f"Table: {self.table_width}x{self.table_height}m\n"
                           f"Marker size: {self.marker_length}m")
        
        # GUI will be updated in the main spin loop
        
    def _on_gui_start_calibration(self):
        """Callback when user clicks 'Start Calibration' button."""
        self.get_logger().info("Calibration requested by user")
        # Transition to calibration state
        self._transition_to_calibration()
        
    def _on_gui_stop(self):
        """Callback when user closes GUI window."""
        self.get_logger().info("Shutdown requested by user")
        self.shutdown_flag = True
        
    def _transition_to_calibration(self):
        """Transition to CALIBRATION state and perform extrinsic calibration."""
        self.state = WatchtowerState.CALIBRATION
        if self.gui:
            self.gui.update_state(self.state)
            self.gui.update_status("Calibrating camera extrinsics...")
            self.gui.add_log("Starting calibration process...")
            
        self.state_pub.publish(String(data=self.state.value))
        self.get_logger().info("=== Starting Calibration ===")
        
        # Subscribe to camera to get current image
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback_for_calib,
            10
        )
        
        if self.gui:
            self.gui.update_status("Waiting for camera image...")
        
    def _image_callback_for_calib(self, msg: Image):
        """Callback to receive image for calibration (one-shot)."""
        try:
            self.get_logger().info("Received image for calibration")
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Unsubscribe - we only need one image
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            
            # Perform calibration
            self._perform_calibration(gray_image)
            
        except Exception as e:
            self.get_logger().error(f"Error in calibration image callback: {e}")
            if self.gui:
                self.gui.update_status(f"Calibration failed: {e}")
                self.gui.update_state(WatchtowerState.INIT)
            self.state = WatchtowerState.INIT
            
    def _perform_calibration(self, camera_image: np.ndarray):
        """Perform the extrinsic calibration."""
        log_msg = "Performing extrinsic calibration..."
        self.get_logger().info(log_msg)
        if self.gui:
            self.gui.add_log(log_msg)
        
        try:
            # Run calibration
            result = self.calibrator.calibrate(self.table_ref_image, camera_image)
            
            if not result['success']:
                error_msg = result.get('error', 'Unknown error')
                self.get_logger().error(f"Calibration failed: {error_msg}")
                if self.gui:
                    self.gui.update_status(f"Calibration failed: {error_msg}")
                    self.gui.add_log(f"ERROR: Calibration failed - {error_msg}")
                    self.gui.update_state(WatchtowerState.INIT)
                self.state = WatchtowerState.INIT
                return

                
            # Extract calibration results
            position = result['position']
            quaternion = result['quaternion']
            num_inliers = result['num_inliers']
            repr_error = result['reprojection_error']

            # TRUE POSITION
            # # Position/orientation vraie de la caméra (simu) # TODO BETTER
            camera_support_in_world_pos = np.array([1.5+0.225, 1-1.12, 0.08]) # TODO le +/-0.225 dépend de couleur jaune ou bleu
            camera_support_in_world_quat = np.array([0, 0, -0.707105, 0.707105]) # quaternion (x, y, z, w)
            camera_in_support_pos = np.array([0.0, 0.0, 0.91])
            camera_in_support_quat = np.array([-0.031363, 0.34146, -0.0859194, 0.935435]) # quaternion (x, y, z, w) 

            T_world_support = tf_trans.concatenate_matrices(
                tf_trans.translation_matrix(camera_support_in_world_pos),
                tf_trans.quaternion_matrix(camera_support_in_world_quat)
            )
            T_support_camera = tf_trans.concatenate_matrices(
                tf_trans.translation_matrix(camera_in_support_pos),
                tf_trans.quaternion_matrix(camera_in_support_quat)
            )
            T_world_camera_true = T_world_support @ T_support_camera
            R_true = T_world_camera_true[:3, :3]
            t_vector_true = T_world_camera_true[:3, 3]

            camera_pos_in_world = tf_trans.translation_from_matrix(T_world_camera_true)
            camera_quat_in_world = tf_trans.quaternion_from_matrix(T_world_camera_true)

            position_error, angle_error_deg = compute_calibration_error(
                position, t_vector_true, quaternion, camera_quat_in_world
            )
                        
            self.get_logger().info(f"Calibration successful!")
            self.get_logger().info(f"  Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
            self.get_logger().info(f"  Quaternion: [{quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}]")
            self.get_logger().info(f"  Inliers: {num_inliers}")
            self.get_logger().info(f"  Reprojection error: {repr_error:.2f} px")
            self.get_logger().info(f"  Position error: {position_error:.3f} m")
            self.get_logger().info(f"  Orientation error: {angle_error_deg:.2f} degrees")
            
            # Set camera pose in localizer
            self.localizer.set_camera_pose(position, quaternion)
            
            # Update GUI
            if self.gui:
                info_str = (f"Calibration successful!\n"
                          f"Is Simu? {'Yes' if self.is_simu else 'No'}\n"
                          f"Cam Position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]\n"
                          f"Cam true Position: [{camera_pos_in_world[0]:.2f}, {camera_pos_in_world[1]:.2f}, {camera_pos_in_world[2]:.2f}]\n"
                          f"Inliers: {num_inliers} --- Repr. error: {repr_error:.2f} px\n"
                          f"Pos. error: {position_error:.3f} m\n"
                          f"Orient. error: {angle_error_deg:.2f}°")
                self.gui.update_info(info_str)
                self.gui.update_status("Calibration complete. Starting robot localization...")
                self.gui.add_log("✓ Calibration successful!")
                self.gui.add_log(f"Camera position: {position}")
                self.gui.add_log(f"Reprojection error: {repr_error:.2f} px")
                self.gui.add_log(f"Position error: {position_error:.3f} m")
                self.gui.add_log(f"Orientation error: {angle_error_deg:.2f} degrees")
                
            # Transition to running state
            self._transition_to_running()
            
        except Exception as e:
            error_msg = f"Exception during calibration: {e}"
            self.get_logger().error(error_msg)
            import traceback
            traceback.print_exc()
            if self.gui:
                self.gui.update_status(f"Calibration error: {e}")
                self.gui.add_log(f"ERROR: {error_msg}")
                self.gui.update_state(WatchtowerState.INIT)
            self.state = WatchtowerState.INIT
            
    def _transition_to_running(self):
        """Transition to RUNNING state and start robot localization."""
        self.state = WatchtowerState.RUNNING
        if self.gui:
            self.gui.update_state(self.state)
            self.gui.update_status("Localizing robots...")
            self.gui.add_log("="*35)
            self.gui.add_log("Entering RUNNING state")
            self.gui.add_log("Waiting for robot detections...")
            
        self.state_pub.publish(String(data=self.state.value))
        self.get_logger().info("=== Entering Running Mode ===")
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback_running,
            10
        )
        
        # Create timer for processing at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self._process_and_publish)
        
    def _image_callback_running(self, msg: Image):
        """Callback to receive images during running state."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.image_lock:
                self.latest_image = cv_image
                
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def _visualize_table_with_robots(self, detections_dict: dict, target_width: int = None, target_height: int = None) -> np.ndarray:
        """
        Create a visualization of the table with detected robot positions.
        
        Args:
            detections_dict: Dictionary of detected robots with positions/orientations
            target_width: Target width for the visualization (optional, for GUI display)
            target_height: Target height for the visualization (optional, for GUI display)
            
        Returns:
            Visualization image (BGR)
        """
        try:
            if self.table_ref_image_color is None:
                self.get_logger().warning("Table reference image is None")
                return None
            
            # Copy the reference image
            viz_image = self.table_ref_image_color.copy()
            h_orig, w_orig = viz_image.shape[:2]
            
            # If target dimensions provided, resize now (before drawing)
            if target_width is not None and target_height is not None:
                scale = min(target_width / w_orig, target_height / h_orig)
                new_w = int(w_orig * scale)
                new_h = int(h_orig * scale)
                viz_image = cv2.resize(viz_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                h_orig, w_orig = new_h, new_w
                self.get_logger().debug(f"Resized viz image to {w_orig}x{h_orig} for display, scale={scale:.3f}")
            
            h, w = viz_image.shape[:2]
            
            if not detections_dict:
                self.get_logger().debug("No detections to visualize on table")
                return viz_image
            
            self.get_logger().info(f"Visualizing {len(detections_dict)} robots on table (image size: {w}x{h})")
            
            # ABOUT DRAWING ON THE IMAGE
            # image coordinates: (0,0) top-left, (w,h) bottom-right
            # SO WE SHOULD ALWAYS INVERSE THE Y AXIS WHEN MAPPING FROM TABLE TO IMAGE

            # Draw camera position first
            if hasattr(self.localizer, 'camera_pose') and self.localizer.camera_pose is not None:
                cam_pos = np.array(self.localizer.camera_pose['position'])
                table_w, table_h = self.table_width, self.table_height
                
                cv2.circle(viz_image, (20, 20), 50, (0, 0, 0), 10)  # Black outline

                # Normalize to 0-1 range
                norm_cam_x = ((table_w-cam_pos[0]) / table_w)
                norm_cam_y = (cam_pos[1] / table_h)
                
                # Convert to pixel coordinates
                cam_img_x = int(norm_cam_x * w)
                cam_img_y = int(norm_cam_y * h) + 30 # TODO MARGIN TO SEE IT
                
                self.get_logger().info(f"Camera position in table frame: {cam_pos}, normalized: ({norm_cam_x:.3f}, {norm_cam_y:.3f}), pixel: ({cam_img_x}, {cam_img_y})")
                
                # Draw camera as a square (cyan color)
                size = max(10, int(min(w, h) / 30)) # Proportional to image size
                cv2.rectangle(viz_image, (cam_img_x - size, cam_img_y - size), (cam_img_x + size, cam_img_y + size), (255, 255, 0), 3)
                text_x = max(0, cam_img_x - 50)
                text_y = max(30, cam_img_y - size + 30)
                cv2.putText(viz_image, "CAM", (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)

                cam_quat = self.localizer.camera_pose['quaternion']
                cam_yaw = angles.quat_to_rad(w=cam_quat[3], z=cam_quat[2])
                self.get_logger().debug(f"Camera yaw angle: {math.degrees(cam_yaw):.1f} degrees")
            else:
                self.get_logger().warning(f"Cannot draw camera: hasattr={hasattr(self.localizer, 'camera_pose')}, value={getattr(self.localizer, 'camera_pose', None)}")
            
            # Draw robots on the table
            for marker_id, data in detections_dict.items():
                team_name, team_color_hex, emoji = get_team_color(marker_id)
                
                # Convert hex to BGR for OpenCV
                team_color_bgr = tuple(int(team_color_hex[i:i+2], 16) for i in (5, 3, 1))
                
                # data['position'] is in world/table frame
                pos_table = np.array(data['position'])
                self.get_logger().info(f"Robot {marker_id} position in table frame: {pos_table}")
                
                # Project to image coordinates
                table_w, table_h = self.table_width, self.table_height
                
                # Normalize to 0-1 range (center of table at 0.5, 0.5)
                norm_x = ((table_w - pos_table[1]) / table_w)
                norm_y = (pos_table[0] / table_h)
                
                # Convert to pixel coordinates
                img_x = int(norm_x * w)
                img_y = int((1.0 - norm_y) * h)
                
                self.get_logger().info(f"Robot {marker_id} normalized: ({norm_x:.3f}, {norm_y:.3f}) -> pixel ({img_x}, {img_y})")
                
                # Clamp to image bounds with small margin
                # img_x = max(30, min(img_x, w - 30))
                # img_y = max(30, min(img_y, h - 30))
                
                # Draw circle for robot position (radius based on image size)
                radius = max(10, int(min(w, h) / 30))
                cv2.circle(viz_image, (img_x, img_y), radius, team_color_bgr, -1)
                cv2.circle(viz_image, (img_x, img_y), radius, (0, 0, 0), 2)  # Black outline
                
                # Draw ID text
                text = f"{emoji}ID{marker_id}"
                font_scale = 0.5
                thickness = 1
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                text_x = img_x - text_size[0] // 2
                text_y = img_y + text_size[1] // 2 + radius + 10
                
                cv2.putText(viz_image, text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
                
                self.get_logger().debug(f"Drew robot {marker_id} at ({img_x}, {img_y})")
            
            return viz_image
            
        except Exception as e:
            self.get_logger().error(f"Error visualizing table: {e}")
            import traceback
            traceback.print_exc()
            return self.table_ref_image_color if self.table_ref_image_color is not None else None
            
    def _process_and_publish(self):
        """Process latest image and publish robot poses."""
        with self.image_lock:
            if self.latest_image is None:
                return
            image = self.latest_image.copy()
            
        try:
            # Convert to grayscale for processing
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Localize robots
            id_filter = (self.marker_id_min, self.marker_id_max)
            results = self.localizer.localize_robots(gray, id_filter=id_filter)
            
            if self.gui:
                # Log detection with team colors
                if results:
                    detection_log = f"Detected {len(results)} robot(s):"
                    for marker_id in sorted(results.keys()):
                        team_name, _, emoji = get_team_color(marker_id)
                        detection_log += f" {emoji}ID{marker_id}"
                    self.gui.add_log(detection_log)
                else:
                    self.gui.add_log("No robots detected in this frame")
            
            # Publish pose array
            if results:
                pose_array = PoseArray()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'world'
                
                for marker_id, data in results.items():
                    pose = Pose()
                    pose.position.x = float(data['position'][0])
                    pose.position.y = float(data['position'][1])
                    pose.position.z = float(data['position'][2])
                    pose.orientation.x = float(data['quaternion'][0])
                    pose.orientation.y = float(data['quaternion'][1])
                    pose.orientation.z = float(data['quaternion'][2])
                    pose.orientation.w = float(data['quaternion'][3])
                    pose_array.poses.append(pose)
                    
                self.pose_array_pub.publish(pose_array)
            
            # Create visualization image
            viz_image = self.localizer.visualize_detections(
                image, 
                id_filter=id_filter,
                draw_axes=True
            )
            
            # Create table visualization with robot positions
            # Get target size for visualization
            canvas_width = 600
            canvas_height = 400
            if self.gui:
                self.gui.table_canvas.update()
                canvas_width = self.gui.table_canvas.winfo_width()
                canvas_height = self.gui.table_canvas.winfo_height()
                if canvas_width <= 1:
                    canvas_width = 600
                if canvas_height <= 1:
                    canvas_height = 400
            
            table_viz = self._visualize_table_with_robots(results if results else {}, 
                                                          target_width=canvas_width,
                                                          target_height=canvas_height)
            
            # Update GUI with visualizations
            if self.gui:
                self.gui.update_image(viz_image)
                if table_viz is not None:
                    self.gui.update_table_image(table_viz)
                self.gui.update_detections(results if results else {})
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
            self.viz_image_pub.publish(viz_msg)
            
        except Exception as e:
            error_msg = f"Error processing image: {e}"
            self.get_logger().error(error_msg)
            if self.gui:
                self.gui.add_log(f"ERROR: {error_msg}")
            
    def spin(self):
        """Custom spin to handle GUI updates and ROS callbacks."""
        while rclpy.ok() and not self.shutdown_flag:
            # Process ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Update GUI in main thread
            if self.gui:
                self.gui.update()
    
    def cleanup(self):
        """Clean up resources."""
        if self.gui:
            self.gui.destroy()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WatchtowerNode()
        node.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
