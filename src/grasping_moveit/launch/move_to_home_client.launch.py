from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    """Load a yaml file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def get_robot_description_semantic():
    """Generate robot semantic description from SRDF file."""
    package_path = get_package_share_directory("ur_elpis_moveit_config")
    srdf_path = os.path.join(package_path, "config", "my_robot_cell.srdf")
    try:
        with open(srdf_path, 'r') as f:
            robot_description_semantic_content = f.read()
    except EnvironmentError:
        return None
    
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    return robot_description_semantic


def generate_launch_description():
    grasping_moveit_config = os.path.join(
        get_package_share_directory('grasping_moveit'),
        'config',
        'config.yaml'
    )
    
    robot_description_semantic = get_robot_description_semantic()
    
    kinematics_yaml = load_yaml("ur_elpis_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    move_to_pose_server = Node(
        package='grasping_moveit',
        executable='move_to_pose_server',
        name='move_to_pose_server',
        output='screen',
        parameters=[
            grasping_moveit_config,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        respawn=False
    )
    
    move_to_home_client = Node(
        package='grasping_moveit',
        executable='move_to_home_client',
        name='move_to_home_client',
        output='screen',
        respawn=False
    )

    return LaunchDescription([
        move_to_pose_server,
        move_to_home_client,
    ])

