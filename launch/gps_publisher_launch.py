
import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '2'),
        Node(
            package='xrower_gps',  # Tên gói
            executable='gps_publisher',  # Tên của console_script (được định nghĩa trong setup.py)
            name='gps_publisher_node',  # Tên node
            output='screen',  # Hiển thị đầu ra trên màn hình
            parameters=[]
        ),
    ])
