import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('mosa')
    default_params = os.path.join(pkg_share, 'config', 'nuscenes', 'nuscenes0.yaml')

    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    time_file = LaunchConfiguration('time_file')
    out_path = LaunchConfiguration('out_path')
    out_origin_path = LaunchConfiguration('out_origin_path')
    cluster_out_file = LaunchConfiguration('cluster_out_file')
    time_breakdown_file = LaunchConfiguration('time_breakdown_file')

    dynfilter = Node(
        package='mosa',
        executable='dynfilter',
        name='dynfilter_odom',
        output='screen',
        parameters=[
            params_file,
            {"dyn_obj/out_file": out_path},
            {"dyn_obj/out_file_origin": out_origin_path},
            {"dyn_obj/time_file": time_file},
            {"dyn_obj/cluster_out_file": cluster_out_file},
            {"dyn_obj/time_breakdown_file": time_breakdown_file},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mosa.rviz')],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('time_file', default_value=''),
        DeclareLaunchArgument('out_path', default_value=''),
        DeclareLaunchArgument('out_origin_path', default_value=''),
        DeclareLaunchArgument('cluster_out_file', default_value=''),
        DeclareLaunchArgument('time_breakdown_file', default_value=''),
        dynfilter,
        rviz_node,
    ])
