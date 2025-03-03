# Marker Helper


## Supported types

- tuple(x, y): the z coordinate considered 0. If it is a pose the orientation is considered quat(0, 0, 0, 1)
- tuple (x, y, z).
- Point(ROS type)
- Pose (ROS type)
- PoseWithCovariance
- PoseWithCovarianceStamped
- geometry.Pose2D


## Example usage

```python
# Import
from champi_libraries_py.marker_helper.canva import *

# Initialize
Canva(self, enable=True)

# Use
Canva().clear()

Canva().add(items.Polyline(self.some_poses, size=presets.LINE_MEDIUM, color=presets.GREEN))
Canva().add(items.OrientedCube(self.some_poses[0], size=(0.1, 0.2, 0.3)))
Canva().add(items.Sphere((0.5, 1), size=0.4, color=presets.BLUE))
Canva().add(items.Arrows(self.some_poses, color=presets.RED))
Canva().add(items.Cubes(self.some_poses, color=presets.MAGENTA))
Canva().add(items.OrientedCubes(self.some_poses, color=presets.MAGENTA))
Canva().add(items.Points(self.some_poses, color=presets.MAGENTA))
        Canva().add(items.Spheres(self.some_poses, color=presets.MAGENTA), frame_id='odom')


Canva().draw()
```

Check out the full example [here](../../../quick_tests/scripts/test_markers.py).

## Performance

Above code takes 5 ms to execute on André's laptop, and 0.02s when enable=False.
