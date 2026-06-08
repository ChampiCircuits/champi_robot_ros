# Champi Webots

## 🎯 **Package Objective**

The `champi_webots` package provides the interface between ROS2 and the Webots simulator for the French Robotics Cup. It configures and launches the simulation with a camera of the playing field.

## 🚀 **Usage**

### **Basic Launch**
```bash
cd ~/champi_ws
source install/setup.bash # or .zsh
ros2 launch champi_webots webots.launch.py
```

### **Published Topics**
- `/watchtower/camera/image_color` (sensor_msgs/Image) - Color images
- `/watchtower/camera/camera_info` (sensor_msgs/CameraInfo) - Calibration information

### **Functionality Verification**
```bash
# List available topics
ros2 topic list | grep watchtower

# Check publication frequency
ros2 topic hz /watchtower/camera/image_color

# Visualize images
ros2 run rqt_image_view rqt_image_view /watchtower/camera/image_color
```

## ⚙️ **Camera Configuration**

### **Modifiable Parameters in URDF**

The `urdf/watchtower.urdf` file contains the complete camera configuration. Here are the main parameters:


#### **1. Camera Activation**
```xml
<enabled>True</enabled>
```
- `True` : Camera active and publishes to ROS2
- `False` : Camera disabled

#### **2. ROS2 Topic Names**
```xml
<topicName>/watchtower/camera</topicName>
```
- Defines the main topic: `/watchtower/camera`
- Sub-topics are automatic:
  - `/watchtower/camera/image_color` (images)
  - `/watchtower/camera/camera_info` (calibration)

#### **3. Publication Frequency**
```xml
<updateRate>10</updateRate>
```
- Value in Hz (images per second)
- **Recommendations**:
  - `5-10 Hz` : Normal usage, saves resources
  - `15-30 Hz` : Fast object tracking
  - `60+ Hz` : Critical real-time applications

#### **4. Publication Mode**
```xml
<alwaysOn>True</alwaysOn>
```
- `True` : Publishes continuously (more CPU load)
- `False` : Only publishes when nodes are subscribed (recommended)

#### **5. Frame ID for TF2**
```xml
<frameName>watchtower_camera_frame</frameName>
```
- Frame name for TF2 coordinate system
- Used for 3D localization

### **Physical Camera Parameters (in Webots world)**

In the `worlds/champi_webots.wbt` file, you can modify:

#### **1. Resolution**
```vrml
Camera {
  name "watchtower_camera"
  width 640      # Width in pixels
  height 480     # Height in pixels
}
```

#### **2. Position and Orientation**
```vrml
Camera {
  translation 0 0 0.91           # Position X Y Z (meters)
  rotation 0.357 0.357 -0.863 1.717  # Rotation (axis + angle)
}
```

#### **3. Optical Parameters** (optional)
```vrml
Camera {
  fieldOfView 0.785398    # Field of view (radians)
  near 0.01              # Minimum distance (meters)  
  far 100.0              # Maximum distance (meters)
}
```

**After modifying parameters, don't forget to rebuild:**
```bash
colcon build --packages-select champi_webots
source install/setup.bash # or .zsh
```
