from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments (similar to XML <arg>)
        DeclareLaunchArgument(
            'resolution', default_value='0.05', description='Map resolution'
        ),
        DeclareLaunchArgument(
            'depth_levels', default_value='16', description='Depth levels for the map'
        ),
        DeclareLaunchArgument(
            'num_workers', default_value='1', description='Number of workers'
        ),
        DeclareLaunchArgument(
            'color_map', default_value='true', description='Use color mapping'
        ),

        # Node (similar to XML <node>)
        Node(
            package='ufomap_mapping',
            executable='ufomap_mapping_server_node',
            name='ufomap_mapping_server_node',
            output='log',
            remappings=[('cloud_in', '/camera/depth/points')],
            parameters=[{
                'resolution': LaunchConfiguration('resolution'),
                'depth_levels': LaunchConfiguration('depth_levels'),
                'num_workers': LaunchConfiguration('num_workers'),
                'color_map': LaunchConfiguration('color_map')
            }]
        )
    ])
