
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    img = "/home/rhys/ros2_ws/src/RS2/visual/Testmodels/BottleEsky/Test1.png"
    return LaunchDescription([
        Node(package='visual', executable='camera_node',   name='camera_node',
             parameters=[{'image_file': img}]),
        Node(package='visual', executable='esky_detector', name='esky_detector'),
        Node(package='visual', executable='bottle_detector',
             name='bottle_detector'),
        Node(package='visual', executable='cartesian',      name='cartesian'),
        Node(package='visual', executable='brand_recognition',
             name='brand_recognition'),
        Node(package='visual', executable='fusion_node',    name='fusion_node'),
    ])