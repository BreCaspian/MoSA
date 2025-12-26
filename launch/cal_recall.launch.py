from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dataset = LaunchConfiguration('dataset')
    dataset_folder = LaunchConfiguration('dataset_folder')
    start_se = LaunchConfiguration('start_se')
    end_se = LaunchConfiguration('end_se')
    start_param = LaunchConfiguration('start_param')
    end_param = LaunchConfiguration('end_param')
    is_origin = LaunchConfiguration('is_origin')

    recall_node = Node(
        package='mosa',
        executable='cal_recall',
        name='check_dynamic',
        output='screen',
        parameters=[
            {"dyn_obj/dataset": ParameterValue(dataset, value_type=int)},
            {"dyn_obj/dataset_folder": dataset_folder},
            {"dyn_obj/start_se": ParameterValue(start_se, value_type=int)},
            {"dyn_obj/end_se": ParameterValue(end_se, value_type=int)},
            {"dyn_obj/start_param": ParameterValue(start_param, value_type=int)},
            {"dyn_obj/end_param": ParameterValue(end_param, value_type=int)},
            {"dyn_obj/is_origin": ParameterValue(is_origin, value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='0'),
        DeclareLaunchArgument('dataset_folder', default_value=''),
        DeclareLaunchArgument('start_se', default_value='0'),
        DeclareLaunchArgument('end_se', default_value='0'),
        DeclareLaunchArgument('start_param', default_value='0'),
        DeclareLaunchArgument('end_param', default_value='0'),
        DeclareLaunchArgument('is_origin', default_value='false'),
        recall_node,
    ])
