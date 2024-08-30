import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('robeff_udp_cam_ros2_driver'),  
        'config',       
        'robeff_udp_cam_ros2_driver.param.yaml'
    )
    print(config)
    return LaunchDescription([
        Node(
            package='robeff_udp_cam_ros2_driver',
            executable='robeff_udp_cam_ros2_driver',
            name='robeff_udp_cam_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]          
        )
    ])