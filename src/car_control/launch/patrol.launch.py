from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            executable='patrol',
            name='patrol',
            output='screen',
            parameters=[{
                'servo_pin':     12,
                'led1_pin':      17,
                'led2_pin':      25,
                'buzzer_pin':     5,
                'ena_pin':       18,
                'in1_pin':       23,
                'in2_pin':       24,
                'enb_pin':       13,
                'in3_pin':       27,
                'in4_pin':       22,
                'servo_step':     4.0,
                'patrol_time':    5.0,
                'base_speed':    45,
                'lock_frames':    3,
                'unlock_frames': 10,
                'unlock_secs':   8.0,
                'confidence':     0.35,
                'led_flash_hz':  15,
                'model_path':    '/home/mohanad/yolo11s_ncnn_model',
                'trig_pin':       4,
                'echo_pin':       6,
                'proximity_cm':  30.0,
            }],
        ),
    ])
