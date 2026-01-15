from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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


def get_robot_description(ur_type, robot_ip):
    """Generate robot description from custom URDF xacro."""
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_elpis_control"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_elpis_control"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_elpis_control"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_elpis_control"), "config", ur_type, "visual_parameters.yaml"]
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_elpis_control"), "urdf", "my_robot_cell_controlled.urdf.xacro"]),
            " ", "ur_type:=", ur_type, " ", "robot_ip:=", robot_ip,
            " ", "joint_limit_params:=", joint_limit_params,
            " ", "kinematics_params:=", kinematics_params,
            " ", "physical_params:=", physical_params,
            " ", "visual_params:=", visual_params,
            " ", "use_fake_hardware:=", "false",
            " ", "fake_sensor_commands:=", "false",
            " ", "headless_mode:=", "false", " "
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    return robot_description


def get_robot_description_semantic():
    """Generate robot semantic description from SRDF file."""
    # Load SRDF directly (not xacro) from ur_elpis_moveit_config
    # SRDF is XML, not YAML, so load it as text
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
    """Generate launch description for pick system."""
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch'
    )
    
    # Declare launch arguments
    camera_serial = DeclareLaunchArgument('camera_serial', default_value='042222070546', description='RealSense camera serial number')
    # These parameters are required by move_to_pose_server for MoveIt initialization
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur10', description='UR robot type')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.0.100', description='Robot IP address')
    
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    
    # These generate parameters required by move_to_pose_server (MoveIt needs robot_description)
    robot_description = get_robot_description(ur_type, robot_ip)
    robot_description_semantic = get_robot_description_semantic()
    
    # Load kinematics.yaml - needed for move_to_pose_server
    kinematics_yaml = load_yaml("ur_elpis_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # Load planning pipelines - needed for mtc_action_server to use OMPL
    ompl_planning_yaml = load_yaml("ur_elpis_moveit_config", "config/ompl_planning.yaml") or {}
    chomp_planning_yaml = load_yaml("ur_elpis_moveit_config", "config/chomp_planning.yaml") or {}
    
    # Structure parameters for MTC's PipelinePlanner
    # When PipelinePlanner is created with pipeline name "ompl", it looks for "ompl.planning_plugin"
    planning_pipelines = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            **ompl_planning_yaml
        },
        "chomp": {
            "planning_plugin": "chomp_interface/CHOMPPlanner",
            **chomp_planning_yaml
        }
    }
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'serial_no': [TextSubstitution(text='_'), LaunchConfiguration('camera_serial')],
            'hole_filling_filter.enable': 'true'
        }.items()
    )
    
    # NOTE: UR robot driver and MoveIt infrastructure should be launched separately
    # Launch with: ros2 launch ur_elpis_moveit_config demo.launch.py
    # This ensures MoveIt (move_group node) and robot driver are running before pick_system
    # The robot description parameters below are passed to move_to_pose_server for MoveIt to initialize MoveGroupInterface

    segmentation_server = Node(
        package='segmentation',
        executable='segmentation_server',
        name='segmentation_server',
        output='screen',
        parameters=[],
        respawn=False
    )
    
    world_coords_server = Node(
        package='world_coords',
        executable='world_coords_server',
        name='world_coords_server',
        output='screen',
        parameters=[],
        respawn=False
    )
    
    # Load grasping config
    grasping_config = os.path.join(
        get_package_share_directory('grasping_moveit'),
        'config',
        'config.yaml'
    )
    
    move_to_pose_server = Node(
        package='grasping_moveit',
        executable='move_to_pose_server',
        name='move_to_pose_server',
        output='screen',
        parameters=[
            grasping_config,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        respawn=False
    )
    
    placement_server = Node(
        package='placing',
        executable='placement_server',
        name='placement_server',
        output='screen',
        parameters=[],
        respawn=False
    )
    
    # MTC-based action server for pick and place operations
    mtc_action_server = Node(
        package='grasping_moveit',
        executable='mtc_action_server',
        name='mtc_action_server',
        output='screen',
        parameters=[
            grasping_config,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines,
        ],
        respawn=False
    )
    
    # Load grounding config
    grounding_config = os.path.join(
        get_package_share_directory('grounding'),
        'config',
        'config.yaml'
    )
    
    # Grounding node - converts MoveIt scene to PDDL predicates
    grounding_node = Node(
        package='grounding',
        executable='grounding_node',
        name='grounding_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            grounding_config,
        ],
        respawn=False
    )
    
    # TAMP planning server - generates action sequences from goals
    task_planning_server = Node(
        package='task_planner',
        executable='task_planning_server',
        name='task_planning_server',
        output='screen',
        parameters=[],
        respawn=False
    )

    return LaunchDescription([
        camera_serial,
        ur_type_arg,
        robot_ip_arg,
        realsense_launch,
        segmentation_server,
        world_coords_server,
        move_to_pose_server,
        placement_server,
        mtc_action_server,
        grounding_node,
        task_planning_server,
    ])

