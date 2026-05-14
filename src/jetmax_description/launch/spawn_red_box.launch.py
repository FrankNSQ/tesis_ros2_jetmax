from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('jetmax_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'red_box.urdf')

    spawn_red_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'red_box',
            '-file', urdf_file,
            '-x', '0.00',
            '-y', '-0.185',
            '-z', '0.03'
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_red_box
    ])
