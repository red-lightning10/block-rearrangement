from launch import LaunchDescription
from launch_ros.actions import Node

# Not needed to be launched separately, it is launched as part of start_robot.launch.py in ur_elpis_control
def generate_launch_description():
    """Launch gripper action server and joint state merger"""
    
    # Gripper action server - provides MoveIt-compatible action interface
    gripper_action_server = Node(
        package='gripper',
        executable='gripper_action_server',
        name='gripper_action_server',
        output='screen',
        respawn=True
    )
    
    # Joint state merger - merges UR arm and gripper joint states
    joint_state_merger = Node(
        package='gripper',
        executable='joint_state_merger',
        name='joint_state_merger',
        output='screen',
        respawn=True
    )
    
    return LaunchDescription([
        gripper_action_server,
        joint_state_merger,
    ])


