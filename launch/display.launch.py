import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mosa')
    default_params = os.path.join(pkg_share, 'config', 'mid360', 'mid360.yaml')

    params_file = LaunchConfiguration('params_file')
    pc_topic = LaunchConfiguration('pc_topic')
    frame_id = LaunchConfiguration('frame_id')
    pc_file = LaunchConfiguration('pc_file')
    pred_file = LaunchConfiguration('pred_file')

    display_node = Node(
        package='mosa',
        executable='display_prediction',
        name='display_prediction',
        output='screen',
        parameters=[
            params_file,
            {"dyn_obj/pc_topic": pc_topic},
            {"dyn_obj/frame_id": frame_id},
            {"dyn_obj/pc_file": pc_file},
            {"dyn_obj/pred_file": pred_file},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('pc_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('frame_id', default_value='camera_init'),
        DeclareLaunchArgument('pc_file', default_value=''),
        DeclareLaunchArgument('pred_file', default_value=''),
        display_node,
    ])
