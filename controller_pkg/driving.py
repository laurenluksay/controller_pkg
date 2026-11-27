"""
driving.py - helper functions for moving the robot.

Describes the movement functions for the robot so the state machine node
can focus on the route itself
"""

import math
import subprocess

# Names were found with 'gz service -l' and 'gz model --list' (2026-11-26)
WORLD_NAME = 'fast_world'
ROBOT_MODEL_NAME = 'my_robot'
SET_POSE_SERVICE = '/world/' + WORLD_NAME + '/set_pose'

# Height to spawn robot at to avoid phasing into void
DEFAULT_SPAWN_HEIGHT = 0.1

def yaw_to_quaterion(yaw):
    """
    Turn a yaw angle into a quaterion for flat robot.

    Why: Gazebo uses a 4D quaterion to define orientation of the robot, where
    x and y are roll and pitch quaterion outputs, and both z and w depend on yaw and are restricted 
    by each other's values (z=sin(yaw/2), w=cos(yaw/2), w^2+z^2=1). Function makes
    handling orientation simpler.

    Input: Yaw in radians (-pi to pi same as gz)
    Output: (quat_x, quat_y, quat_z, quat_w)
    """
    half_yaw = yaw / 2.0

    # x=y=0 since its flat, only turns about z
    quat_x = 0.0
    quat_y = 0.0
    quat_z = math.sin(half_yaw)
    quat_w = math.cos(half_yaw)

    return quat_x, quat_y, quat_z, quat_w

def teleport(x, y, z, yaw):
    """
    Move to robot instantly to (x,y) and facing yaw in radians
    Outputs: True if Gzebo says the move worked, false otherwise

    **Note:** Stop the robot before calling this (publish zero /cmd_vel) otherwise
    robot will continue driving when it lands.
    """
    quat_x, quat_y, quat_z, quat_w = yaw_to_quaterion(yaw)

    #Build request text

    request_text = ('name: "' + ROBOT_MODEL_NAME + '",'
                    'position: {'
                    'x: ' + str(x) + ', '
                    'y: ' + str(y) + ', '
                    'z: ' + str(z) + '}, '
                    'orientation: {'
                    'x: ' + str(quat_x) + ', '
                    'y: ' + str(quat_y) + ', '
                    'z: ' + str(quat_z) + ', '
                    'w: ' + str(quat_w) + '}'
                    )

    # Build fill gz service command
    command = [
        'gz', 'service',
        '-s', SET_POSE_SERVICE,
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000',
        '--req', request_text,
    ]

    # Run it and capture printed output
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    except subprocess.TimeoutExpired:
        print('teleport: gz service timed out')
        return False
 
    # Gazebo prints "data: true" when the move worked
    succeeded = 'data: true' in result.stdout
    if not succeeded:
        print('teleport failed. stdout: ' + result.stdout + ' stderr: ' + result.stderr)
 
    return succeeded