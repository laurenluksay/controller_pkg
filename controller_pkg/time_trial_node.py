#!/usr/bin/env python3
# Above line indicates for Linux to run this file with Python3
"""
Time Trial Code Setup

Node Function:
1. Stalls for ROS to finalize connecting to publishers,
2. Sends activation method to timer
3. Tells the robot to drive forward for 3 seconds, then stop
4. Stops "stop timer" to score tracker
5. Shuts down the environment
"""

import rclpy # main python ROS2 library
from rclpy.node import Node # Basic Node building blocks
from rclpy.parameter import Parameter # Node settings
from geometry_msgs.msg import Twist # Command type for robot commands
from std_msgs.msg import String # Message type for score tracker

# --- Time Trial Settings --- #
TEAM_NAME = "Team11"
TEAM_PASSWORD = "MarLau"
STALL_TIME = 1.0 #s - The time to stall the program so ROS can properly connect
DRIVE_SPEED = 1.0 #m/s - Speed to drive forward at
DRIVE_DUR = 5.0 #s - How long the robot should be driving for
END_STALL = 0.5 #s - The time to stall the program at the end so ROS can terminate connections
CONTROL_PERIOD = 0.1 #s, so runs 10 times /s

"""
class: TimeTrialNode(Node)
@brief Creates a new node type called TimeTrialNode
@param Node - Node name
"""
class TimeTrialNode(Node):

    def __init__(self):
        # Creates node and gets it to use the simulation time
        # over laptop clock
        super().__init__("time_trial_node",
                         parameter_overrides=[Parameter("use_sim_time,",
                                              Parameter.Type.BOOL, True)],
        )

        # driver command publisher
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Publisher for score tracker messages
        self.score_publisher = self.create_publisher(String, "/score_tracker", 10)

        # Which step the robot is currently on
        self.current_state = "WAITING_TO_START"

        # Sim time when the current step began (filled in on the first loop)
        self.state_start_time = None

        # Becomes True when everything is done, so main() knows to exit
        self.finished = False

        # Run control_loop() every TIMER_PERIOD_SECONDS
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.get_logger().info("Time trial node started. Waiting before sending start message...")

    def get_sim_time_seconds(self):
        # Current simulation time, converted from nanoseconds to seconds
        return self.get_clock().now().nanoseconds / 1e9

    def change_state(self, new_state):
        # Move to a new step and remember when it started
        self.current_state = new_state
        self.state_start_time = self.get_sim_time_seconds() # Get simulation time for new step
        self.get_logger().info("Now in state: " + new_state) # Records message of current state

    def send_score_message(self, clue_location, clue_prediction):
        # Build the "TeamName,password,location,prediction" text and publish it to the score tracker
        message = String()
        message.data = (
            TEAM_NAME + "," + TEAM_PASSWORD + "," + str(clue_location) + "," + clue_prediction
        )
        self.score_publisher.publish(message) # Publishes message to score tracker
        self.get_logger().info("Sent to score tracker: " + message.data) # Logs previously sent message

    def send_velocity(self, forward_speed, turn_speed):
        # Build a velocity command and publish it to /cmd_vel
        velocity_message = Twist() # Builds velocity command
        velocity_message.linear.x = forward_speed   # metres per second, forward is positive
        velocity_message.angular.z = turn_speed     # radians per second, left is positive
        self.velocity_publisher.publish(velocity_message) # publishes message to Gazebo

    def control_loop(self):
        current_time = self.get_sim_time_seconds()

        # On the very first loop, remember when we started waiting
        # Initialize start time
        if self.state_start_time is None:
            self.state_start_time = current_time

        # Calculates how long we have been in the current step
        time_in_state = current_time - self.state_start_time

        # Operation for first stage prior to driving
        if self.current_state == "WAITING_TO_START":
            # Wait, then start the competition timer
            if time_in_state >= STALL_TIME:
                self.send_score_message(0, "NA")
                self.change_state("DRIVING")

        # Operation for driving state of the robot
        elif self.current_state == "DRIVING":
            # Drive forward until the drive time has lapsed
            if time_in_state < DRIVE_DUR:
                self.send_velocity(DRIVE_SPEED, 0.0)
            else:
                self.send_velocity(0.0, 0.0)
                self.change_state("STOPPING")

        elif self.current_state == "STOPPING":
            # Keep sending "stop" for a moment so the robot definitely stops,
            # then stop the competition timer
            self.send_velocity(0.0, 0.0)
            if time_in_state >= END_STALL:
                self.send_score_message(-1, "NA")
                self.change_state("DONE")

        elif self.current_state == "DONE":
            self.finished = True

def main(args=None):
    rclpy.init(args=args)
    time_trial_node = TimeTrialNode()

    # Keep running the timer until the node says it is finished
    while rclpy.ok() and not time_trial_node.finished:
        rclpy.spin_once(time_trial_node, timeout_sec=0.1)

    time_trial_node.get_logger().info("Time trial finished. Shutting down.")
    time_trial_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()