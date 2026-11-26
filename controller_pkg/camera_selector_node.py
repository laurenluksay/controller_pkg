#!/usr/bin/env python3
"""
Camera selector node.

Listens to all 4 robot cameras and to /camera_select, where the state
machine says which camera to use ("front", "back", "left" or "right").
Republishes only the chosen camera's images on /selected_camera_feed,
so the clue reader always reads from one topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Topic names for each camera (from the robot's camera setup)
FRONT_CAMERA_TOPIC = '/camera/image_raw'
BACK_CAMERA_TOPIC = '/back_camera/image_raw'
LEFT_CAMERA_TOPIC = '/left_camera/image_raw'
RIGHT_CAMERA_TOPIC = '/right_camera/image_raw'

# Topic Marnu's state machine publishes on, and the topic this node outputs on
CAMERA_SELECT_TOPIC = '/camera_select'
OUTPUT_TOPIC = '/selected_camera_feed'

# Camera names the state machine is allowed to send
VALID_CAMERA_NAMES = ['front', 'back', 'left', 'right']

# Camera used until the state machine says otherwise
DEFAULT_CAMERA = 'front'

class CameraSelectorNode(Node):

    def __init__(self):
        super().__init__('camera_selector_node')

        # Remember which camera is currently selected
        self.selected_camera = DEFAULT_CAMERA

        # Publisher: sends the selected camera's images out on /selected_camera_feed
        self.feed_publisher = self.create_publisher(Image, OUTPUT_TOPIC, 10)

        # Subscriber: listens for the state machine's camera choice
        self.camera_select_subscriber = self.create_subscription(
            String, CAMERA_SELECT_TOPIC, self.camera_select_callback, 10)

        # Subscribers: one per camera, each with its own callback function
        self.front_subscriber = self.create_subscription(
            Image, FRONT_CAMERA_TOPIC, self.front_image_callback, qos_profile_sensor_data)
        self.back_subscriber = self.create_subscription(
            Image, BACK_CAMERA_TOPIC, self.back_image_callback, qos_profile_sensor_data)
        self.left_subscriber = self.create_subscription(
            Image, LEFT_CAMERA_TOPIC, self.left_image_callback, qos_profile_sensor_data)
        self.right_subscriber = self.create_subscription(
            Image, RIGHT_CAMERA_TOPIC, self.right_image_callback, qos_profile_sensor_data)

        # Print a startup message to the terminal
        self.get_logger().info(
            f'Camera selector started. Using "{self.selected_camera}" camera, '
            f'publishing on {OUTPUT_TOPIC}')

    def camera_select_callback(self, msg):
        # msg.data is the text the state machine sent, e.g. "left"
        requested_camera = msg.data.strip().lower()

        # Ignore anything that isn't one of the 4 camera names
        if requested_camera not in VALID_CAMERA_NAMES:
            self.get_logger().warn(
                f'Unknown camera "{msg.data}" on {CAMERA_SELECT_TOPIC}. '
                f'Keeping "{self.selected_camera}".')
            return

        # Only print a message if the camera actually changed
        if requested_camera != self.selected_camera:
            self.get_logger().info(
                f'Switching camera: {self.selected_camera} -> {requested_camera}')
            self.selected_camera = requested_camera

    def front_image_callback(self, msg):
        self.republish_if_selected(msg, 'front')

    def back_image_callback(self, msg):
        self.republish_if_selected(msg, 'back')

    def left_image_callback(self, msg):
        self.republish_if_selected(msg, 'left')

    def right_image_callback(self, msg):
        self.republish_if_selected(msg, 'right')

    def republish_if_selected(self, image_msg, camera_name):
        # Only pass the image on if it came from the selected camera
        if camera_name == self.selected_camera:
            self.feed_publisher.publish(image_msg)

def main(args=None):
    # Start ROS2, create the node, and keep it running until Ctrl+C
    rclpy.init(args=args)
    camera_selector_node = CameraSelectorNode()
    try:
        rclpy.spin(camera_selector_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_selector_node.destroy_node()
        rclpy.try_shutdown()
if __name__ == '__main__':
    main()