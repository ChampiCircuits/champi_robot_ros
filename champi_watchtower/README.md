# Champi Watchtower

## 🎯 **Package Objective**

The `champi_watchtower` package provides image processing capabilities for the watchtower camera in the Webots simulation. It subscribes to camera topics and performs real-time image analysis using OpenCV.


## 🚀 **Usage**

### **1. Compilation**

```bash
cd ~/champi_ws
build --packages-select champi_watchtower --symlink-install
source install/setup.bash # or .zsh
```

### **2. Basic Launch**
```bash
# First start the Webots simulation
ros2 launch champi_webots webots.launch.py

# Then launch image processing (in another terminal)
ros2 run champi_watchtower watchtower_image_processor.py
```

### **3. Launch with Custom Parameters**
```bash
ros2 run champi_watchtower watchtower_image_processor.py \
  --ros-args \
  -p show_processed_image:=true \
  -p processing_rate_hz:=10.0
```

## 🎮 **Used ROS2 Topics**

### **Subscriptions**
- `/watchtower/camera/image_color` (sensor_msgs/Image) - Camera color images
- `/watchtower/camera/camera_info` (sensor_msgs/CameraInfo) - Calibration information
