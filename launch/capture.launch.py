from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dv_processing_driver',
            executable='dv_processor_node',
            name='capture_node',
            namespace='dvxm',
            output='screen',
            parameters=[
                {'calibration_path': ''},
                {'accumulate_frames': True},
                {'imu_frame_name': 'imu_link'},
                {'camera_frame_name': 'camera_link'},
                {'transformImuToCameraFrame': False},
                {'unbiasedImuData': False},
                {'accumulator_decay_param': 1.0e+6},
                {'accumulator_event_contribution': 0.15},
                {'accumulator_decay_function': 'EXPONENTIAL'}
            ]
        )
    ])