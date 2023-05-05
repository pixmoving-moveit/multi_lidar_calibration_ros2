from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pass_through_node = Node(
        package='multi_lidar_calibration',
        executable='point_cloud_filter_node',
        name='point_cloud_filter_node',
        output='screen',
        parameters=[{
            'input_topic_name':"/input/point_cloud",
            'output_topic_name': "/output/point_cloud",
            "min_x": 0.0,
            "max_x": 2.0,

            "min_y": 0.0,
            "max_y": 2.0,

            "min_z": 0.0,
            "max_z": 2.0,
        }]
    )
    return LaunchDescription([
        pass_through_node
    ])