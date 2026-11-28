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

# ---------------- Timings (seconds of sim time) ----------------
STARTUP_WAIT = 2.5   # wait after launch before starting the timer
READ_TIME = 0.25 # seconds to pause and read signs

# ---------------- Team info ----------------
TEAM_NAME = 'Team11'
TEAM_PASSWORD = 'MarLau'

# ---------------- Teleport poses ----------------
# (x [m], y [m], yaw [rad]) - recorded 2026-11-26 with `gz model -m my_robot -p`
HOME_POSE = (-5.5, -2.5, math.pi / 2)            # start box, free respawn
FIRST_SPAWN_POSE = (-0.575, 0.0, -math.pi / 2)     # first pink line past the truck -> clues 4, 6, 5
SECOND_SPAWN_POSE = (4.1, 2.3, math.pi)        # tunnel at the mountain base -> clue 7

# ---------------- Loop rate ----------------
CONTROL_PERIOD = 0.05   # seconds between control loop runs (20 Hz)

# ---------------- Drive Segments ----------------
#Format: (Linear {m/s}, Angular {rad/s}, time {s})
SIGN_1_DRIVE = (0.0, 0.0, 0.0) #Pause to read
SIGN_2_DRIVE = (0.5, 0.0, 5.5) #Home y=-2.5 -> y=0.5
SIGN_3_DRIVE = (0.5, 0.0, 2.6) # y=0.5 -> y=1.5
SIGN_4_DRIVE = (-0.5, 0.0, 3.2) # y=0 -> y=1.5
SIGN_6_DRIVE = (0.6, 0.0, 5.4) # y=1.5 -> y=-1.7
SIGN_5_DRIVE_1 = (0.5, 0.0, 1.2) # y=-1.7 -> y=-2.2
SIGN_5_DRIVE_2 = (0.0, math.pi/4, 0.8) # Turn to Sign 5
SIGN_7_DRIVE = (0.0, math.pi/4, 1.3) # Turn to Sign 7

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
        self.get_logger().info('Sent score message: ' + score_text)


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
            new_state: the name of the next state, ex. 'SIGN1'

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
        Runs every CONTROL_PERIOD. Does the current state's action and
        decides when to move to the next state.
        """
        if self.state == 'STARTUP':
            # Wait so publishers finish connecting, then start the run
            if self.time_in_state() >= STARTUP_WAIT:
                self.start_timer()
                self.select_camera('front')
                self.change_state('SIGN1')

        # ------------------------------------------- #

        elif self.state == 'SIGN1':
            linv, angv, dur = SIGN_1_DRIVE
                                
            if self.time_in_state() < READ_TIME + 2.0:
                self.stop()

            else:
                self.select_camera('front')
                self.change_state('SIGN2')

        # ------------------------------------------- #

        elif self.state == 'SIGN2':
            linv, angv, dur = SIGN_2_DRIVE
            if self.time_in_state() < dur:
                self.drive(linv, angv)
            
            elif self.time_in_state() < dur + READ_TIME:
                self.stop()

            else:
                self.select_camera('right')
                self.change_state('SIGN3')

        # ------------------------------------------- #

        elif self.state == 'SIGN3':
            linv, angv, dur = SIGN_3_DRIVE
            if self.time_in_state() < dur:
                self.drive(linv, angv)
            
            elif self.time_in_state() < dur + READ_TIME:
                self.stop()

            else:
                self.change_state('TELEPORT1')

        # ------------------------------------------- #

        elif self.state == 'TELEPORT1':
            self.stop()
            x,y,yaw = FIRST_SPAWN_POSE

            tele_worked = teleport(x, y, DEFAULT_SPAWN_HEIGHT, yaw)
            if not tele_worked:
                self.get_logger().warn('Teleport 1 failed')

            self.select_camera('front')    
            self.change_state('SIGN4')

        # ------------------------------------------- #  

        elif self.state == 'SIGN4':
            linv, angv, dur = SIGN_4_DRIVE
            if self.time_in_state() < dur:
                self.drive(linv, angv)
            
            elif self.time_in_state() < dur + READ_TIME:
                self.stop()

            else:
                self.select_camera('left')
                self.change_state('SIGN6')

        # ------------------------------------------- #

        elif self.state == 'SIGN6':
            linv, angv, dur = SIGN_6_DRIVE
            if self.time_in_state() < dur:
                self.drive(linv, angv)
            
            elif self.time_in_state() < dur + READ_TIME:
                self.stop()

            else:
                self.select_camera('back')
                self.change_state('SIGN5')

        # ------------------------------------------- #

        elif self.state == 'SIGN5':
            linv1, angv1, dur1 = SIGN_5_DRIVE_1
            linv2, angv2, dur2 = SIGN_5_DRIVE_2
            
            if self.time_in_state() < dur1:
                self.drive(linv1, angv1)

            elif self.time_in_state() < dur1 + dur2:
                self.drive(linv2, angv2)
            
            elif self.time_in_state() < dur1 + dur2 + READ_TIME:
                self.stop()

            else:
                self.change_state('TELEPORT2')

        # ------------------------------------------- #

        elif self.state == 'TELEPORT2':
            self.stop()
            x,y,yaw = SECOND_SPAWN_POSE

            tele_worked = teleport(x, y, DEFAULT_SPAWN_HEIGHT, yaw)
            if not tele_worked:
                self.get_logger().warn('Teleport 2 failed')

            self.select_camera('front')    
            self.change_state('SIGN7')

        # ------------------------------------------- #

        elif self.state == 'SIGN7':
            linv, angv, dur = SIGN_7_DRIVE
            if self.time_in_state() < dur:
                self.drive(linv, angv)

            elif self.time_in_state() < dur + READ_TIME:
                self.stop()

            else:
                self.stop()
                self.stop_timer()
                self.change_state('DONE')

        # ------------------------------------------- #

        elif self.state == 'DONE':
            # Run is over. Keep the robot still and do nothing else.
            self.stop()

        else:
            self.get_logger().warn('Unknown state: ' + self.state)


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