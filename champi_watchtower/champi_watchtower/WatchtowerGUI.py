#!/usr/bin/env python3

from champi_watchtower.WatchTowerState import WatchtowerState

import tkinter as tk
from PIL import Image as PILImage, ImageTk
from datetime import datetime
import numpy as np
import cv2

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
        return "Blue", "#2196F3", "🔵"
    elif 6 <= marker_id <= 10:
        return "Yellow", "#FFC107", "🟡"
    else:
        return "Unknown", "#9E9E9E", "⚪"


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

        self.info_text = tk.Text(info_frame, height=8, width=50, state=tk.DISABLED,
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
