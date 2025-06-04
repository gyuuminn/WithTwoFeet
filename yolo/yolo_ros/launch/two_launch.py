from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joystick', output='screen'),
        Node(package='yolo_ros', executable='yolo1'),
        Node(package='yolo_ros', executable='joy_twist'),
        Node(package='yolo_ros', executable='motor'),
        Node(package='yolo_ros', executable='mode_mux'),
        Node(package='yolo_ros', executable='motor_bridge',
            parameters=[{'serial_port':'/tmp/ttyV0'}, {'baud_rate':115200}])
    ])
