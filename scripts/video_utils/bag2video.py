import rosbag2_py
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_images(bag_file, image_topic):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()

    bridge = CvBridge()
    writer = None

    for topic in topics:
        if topic.name == image_topic and topic.type == 'sensor_msgs/msg/Image':
            break
    else:
        print('Image topic not found in bag file')
        return
    
    topic_types = reader.get_all_topics_and_types()
    
    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")


    while reader.has_next():
        (topic, msg, t) = reader.read_next()
        if topic == image_topic:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(msg, msg_type)
            image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            if writer is None:
                writer = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 3, (image.shape[1], image.shape[0]))
            writer.write(image)

    if writer is not None:
        writer.release()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract images from a ROS bag file and output as a video.')
    parser.add_argument('bag_file', help='Path to the bag file.')
    parser.add_argument('image_topic', help='Name of the image topic.')
    args = parser.parse_args()

    extract_images(args.bag_file, args.image_topic)