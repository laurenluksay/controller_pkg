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

    def control_loop(self):
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