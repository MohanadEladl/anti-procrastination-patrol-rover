from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[{
                'in1_pin': 23,
                'in2_pin': 24,
                'in3_pin': 27,
                'in4_pin': 22,
            }],
        )
    ])
