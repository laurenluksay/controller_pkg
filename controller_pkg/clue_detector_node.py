"""
clue_detector_node.py - finds the clue board in the live camera feed.

Subscribes:
    /selected_camera_feed  (sensor_msgs/Image)  - the camera chosen by the state machine
    /camera_select         (std_msgs/String)    - which camera that is (front/back/left/right)
Publishes:
    /clue_detector/debug_image (sensor_msgs/Image) - the camera image with the detected
                                                     board outline and corners drawn on it
    /clue_detector/board_image (sensor_msgs/Image) - the detected board's grey middle,
                                                     straightened to a flat 600 x 400 picture
                                                     (only published when a board is found)

The detection and straightening live in clue_detection.py
(detect_board, straighten_board).
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from controller_pkg.clue_detection import detect_board, straighten_board


# Smallest board outline area (pixels) worth reading, for each camera.
# Boards smaller than this are too far away to read. Only a minimum, so a board
# that ends up closer after route tuning still passes.
# Smallest real board in the saved photos: ~6400 px (sign 4, front). Tune live.
MIN_BOARD_AREA_BY_CAMERA = {
    'front': 3000,
    'back': 3000,
    'left': 3000,
    'right': 3000,
}

# The camera selector starts on the front camera, so we do too
STARTING_CAMERA = 'front'


class ClueDetectorNode(Node):

    def __init__(self):
        super().__init__('clue_detector_node')

        # Converts between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Which camera the feed is currently coming from
        self.current_camera = STARTING_CAMERA

        # Camera images: sensor-data QoS drops old frames instead of queueing them
        self.image_subscriber = self.create_subscription(
            Image, '/selected_camera_feed', self.image_callback, qos_profile_sensor_data)

        # Camera choice from the state machine
        self.camera_select_subscriber = self.create_subscription(
            String, '/camera_select', self.camera_select_callback, 10)

        # Debug picture, to watch the detection in rqt_image_view
        self.debug_image_publisher = self.create_publisher(
            Image, '/clue_detector/debug_image', 10)

        # Straightened board picture, to check the perspective transform live
        self.board_image_publisher = self.create_publisher(
            Image, '/clue_detector/board_image', 10)

        self.get_logger().info('Clue detector started, assuming "' +
                               self.current_camera + '" camera')

    def camera_select_callback(self, message):
        """Remember which camera the feed is now coming from."""
        camera_name = message.data
        if camera_name in MIN_BOARD_AREA_BY_CAMERA:
            self.current_camera = camera_name
        else:
            self.get_logger().warn('Unknown camera name: ' + camera_name)

    def image_callback(self, image_message):
        """Runs once per camera frame: find the board, publish the debug picture and,
        if a board was found, the straightened board."""
        # 1. ROS Image -> OpenCV image (BGR, like cv2.imread gives)
        photo_bgr = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')

        # 2. Find the board, using this camera's minimum size
        min_board_area = MIN_BOARD_AREA_BY_CAMERA[self.current_camera]
        ordered_corners, board_contour = detect_board(photo_bgr, min_board_area)

        # 3. Draw what was found on a copy of the image
        debug_image = photo_bgr.copy()
        if board_contour is not None:
            cv2.drawContours(debug_image, [board_contour], -1, (0, 255, 0), 2)
        if ordered_corners is not None:
            corner_labels = ['TL', 'TR', 'BR', 'BL']
            for corner_index in range(4):
                corner_x = int(ordered_corners[corner_index][0])
                corner_y = int(ordered_corners[corner_index][1])
                cv2.circle(debug_image, (corner_x, corner_y), 6, (0, 255, 255), -1)
                cv2.putText(debug_image, corner_labels[corner_index],
                            (corner_x + 8, corner_y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            # At most one log line per second, so the terminal doesn't flood
            self.get_logger().info('Board found on ' + self.current_camera + ' camera',
                                   throttle_duration_sec=1.0)

            # Straighten the board and publish it, keeping the original timestamp
            straight_board = straighten_board(photo_bgr, ordered_corners)
            board_message = self.bridge.cv2_to_imgmsg(straight_board, encoding='bgr8')
            board_message.header = image_message.header
            self.board_image_publisher.publish(board_message)

        cv2.putText(debug_image, 'camera: ' + self.current_camera, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 4. OpenCV image -> ROS Image, keeping the original timestamp, and publish
        debug_message = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_message.header = image_message.header
        self.debug_image_publisher.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = ClueDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()