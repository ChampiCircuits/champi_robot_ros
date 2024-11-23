from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import champi_libraries_py.marker_helper.presets as presets  # For not having to import it in user code
import champi_libraries_py.marker_helper.items as items
import champi_libraries_py.marker_helper.shared_variables as shared_variables


class SingletonMeta(type):
    """A metaclass for Singleton behavior."""
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            # Create a new instance if it doesn't exist
            cls._instances[cls] = super(SingletonMeta, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Canva(metaclass=SingletonMeta):

    is_initialized = False

    def __init__(self, node: Node = None, enable: bool = None):
        """Canva is a singleton. First time, you must initialize it with a node and enable value. If you call the constructor
        again, you can leave the arguments empty. They will not be used anyway.

        Args:
            node (Node, optional): _description_. Defaults to None.
            enable (bool, optional): _description_. Defaults to None.

        Raises:
            ValueError: _description_
        """

        if self.is_initialized:
            return
        elif node is None or enable is None:
            raise ValueError('Canva must be initialized with a node and enable value.')
        
        self.is_initialized = True

        shared_variables.visualization_enabled = enable
        if not enable: return
        
        self.node = node
        self.items = []
        self.nb_markers = 0

        topic_name = "/" + self.node.get_name() + "/visualization_markers"
        self.pub_marker = node.create_publisher(MarkerArray, topic_name, 10)


    def draw(self):
        if not shared_variables.visualization_enabled: return

        if len(self.items) == 0:
            return
        
        marker_array = MarkerArray()

        for item in self.items:
            for marker in item.get_marker_array():
                marker_array.markers.append(marker)
        
        for i, marker in enumerate(marker_array.markers):
            marker.id = i
        
        self.pub_marker.publish(marker_array)

        self.nb_markers = len(marker_array.markers)


    def clear(self):
        if not shared_variables.visualization_enabled: return

        self.items = []
        shared_variables.i_color = 0

        marker_array = MarkerArray()
        for i in range(self.nb_markers):
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = i
            marker_array.markers.append(marker)


    def add(self, item: items.Item, frame_id: str = 'map'):
        if not shared_variables.visualization_enabled: return
        item.set_frame_id(frame_id)
        self.items.append(item)