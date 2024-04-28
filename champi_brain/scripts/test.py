#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from threading import Thread
from time import sleep


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'example_topic', 10)

    def publish_messages(self):
        i = 0
        while rclpy.ok():
            message = String()
            message.data = f'Hello ROS 2! Message {i}'
            self.publisher.publish(message)
            self.get_logger().info(f'Published: {message.data}')
            i += 1
            sleep(1)


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscriber = self.create_subscription(
            String,
            'example_topic',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    # Créez deux nœuds distincts
    pub_node = PublisherNode()
    sub_node = SubscriberNode()

    # Créez un thread séparé pour publier des messages
    publisher_thread = Thread(target=pub_node.publish_messages)
    publisher_thread.start()

    # Utilisez un MultiThreadedExecutor pour exécuter plusieurs nœuds simultanément
    executor = MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    # L'executor assure la réactivité de tous les nœuds ajoutés
    executor.spin()

    # Nettoyage des nœuds après arrêt
    pub_node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
