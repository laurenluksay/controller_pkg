"""
state_machine_node.py - hardcoded driving state machine for the Fizz Detective competition.

Drives the robot along the planned route (straight lines + teleports),
tells the camera selector which camera to use in each state, and
starts/stops the competition timer.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from controller_pkg.driving import teleport, DEFAULT_SPAWN_HEIGHT


# ---------------- Team info ----------------
TEAM_NAME = 'Team11'
TEAM_PASSWORD = 'MarLau'

# ---------------- Teleport poses ----------------
# (x [m], y [m], yaw [rad]) - recorded 2026-11-26 with `gz model -m my_robot -p`
HOME_POSE = (-5.5, -2.5, math.pi / 2)            # start box, free respawn
FIRST_SPAWN_POSE = (-0.5, 0.0, -math.pi / 2)     # first pink line past the truck -> clues 4, 6, 5
MOUNTAIN_SPAWN_POSE = (4.0, 2.3, math.pi)        # tunnel at the mountain base -> clue 7

# ---------------- Loop rate ----------------
CONTROL_PERIOD = 0.05   # seconds between control loop runs (20 Hz)


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.camera_select_publisher = self.create_publisher(String, '/camera_select', 10)
        self.score_publisher = self.create_publisher(String, '/score_tracker', 10)

        # State machine bookkeeping
        self.state = 'STARTUP'
        self.state_start_time = self.get_clock().now()   # sim time, because use_sim_time is on

        # Control loop: runs every CONTROL_PERIOD seconds of sim time
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.get_logger().info('State machine started in state ' + self.state)

    def drive(self, linear_speed, angular_speed):
        """
        Constructs and publishes Twist message to create robot movement.

        Inputs: 
            linear speed: forward speed in m/s
            angular speed: turning speed in rad/s. Follows right hand convention
        """

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_msg)

    def stop(self):
        """
        Stops the robot from driving by publishing an all 0 twist message.
        """
        #New Twist() messages already have all 0s
        self.cmd_vel_publisher.publish(Twist())

    def select_camera(self, camera_name):
        """
        Selects camera to be reading from at a given moment. Makes a new String
        message and sets its data field to 'camera_name'. Then publishes on 
        '/camera_select' with 'self.camera_select_publisher'

        Inputs:
            camera_name: 'front', 'back', 'left', 'right'
        """
        selection_msg = String()
        selection_msg.data = camera_name
        self.camera_select_publisher.publish(selection_msg)

    def send_score_message(self, location, value):
        """
        Sending messages to score tracker in proper formatting

        Inputs:
            location: The clue location number. 0 = start timer, -1 = stop timer
            value: Clue text or NA for timer messages
        """
        score_msg = String()
        score_text = TEAM_NAME + ',' + TEAM_PASSWORD + ',' + str(location) + ',' + value
        score_msg.data = score_text
        self.score_publisher.publish(score_msg)
        self.get_logger().info('Sent score message:' + score_text)


    def start_timer(self):
        """
        Registers team number and starts competition clock. First message the robot
        sends on /score_tracker
        """
        self.send_score_message(0,'NA')

    def stop_timer(self):
            """
            Stops Competition Clock. Last message sent by robot
            """
            self.send_score_message(-1,'NA')
    
    def change_state(self, new_state):
        """
        Moving from one state to the next and recording when that change occurs to allow
        states to time themselves.

        Inputs:
            new_state: the name of the next state, ex. 'CLUE_1'

        Process:
            1. Logs transition for bookkeeping/visualization
            2. sets self.state = new_state
            3. sets self.state_start_time = self.get_clock().now()
        """
        self.get_logger().info('State: ' + self.state + ' -> ' + new_state)
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def time_in_state(self):
        """
        Finds the lapsed time the robot spends in a given state.

        Process:
            1. Gets current sim time with self.get_clock().now()
            2. Subtracts self.state_start_time to find Duration
            3. Converts duration to seconds
        """
        current_time = self.get_clock().now()
        elapsed = current_time - self.state_start_time
        return elapsed.nanoseconds / 1e9
        
    def control_loop(self):
        """
        Repeats state action until told otherwise
        """
        # Runs every CONTROL_PERIOD. The states go here.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()