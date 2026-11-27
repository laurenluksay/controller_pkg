"""
Team launch file for the Fizz Detective competition.

Starts everything on our side (not the simulation itself):
  - the bridge for the back, left and right cameras
  - the camera selector node
Later: State machine and the clue reader get added here too.

Run with:  ros2 launch controller_pkg competition.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Bridge for the 3 extra cameras (Gazebo -> ROS2).
    # The front camera is already bridged by the course's launch file.
    extra_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='extra_camera_bridge',
        arguments=[
            '/back_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/left_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/right_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )
    # Camera selector: republishes the camera chosen on /camera_select
    # to /selected_camera_feed
    camera_selector = Node(
        package='controller_pkg',
        executable='camera_selector_node',
        name='camera_selector_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    state_machine = Node(
        package='controller_pkg',
        executable='state_machine_node',
        name='state_machine_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Everything listed here starts when the launch file runs
    return LaunchDescription([
        extra_camera_bridge,
        camera_selector,
        state_machine,
    ])