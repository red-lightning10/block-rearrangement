from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for spawn_objects_server."""
    
    spawn_objects_config = os.path.join(
        get_package_share_directory('spawn_objects'),
        'config',
        'config.yaml'
    )
    
    spawn_objects_server = Node(
        package='spawn_objects',
        executable='spawn_objects_server',
        name='spawn_objects_server',
        output='screen',
        parameters=[
            spawn_objects_config,
        ],
        respawn=False
    )

    return LaunchDescription([
        spawn_objects_server,
    ])

