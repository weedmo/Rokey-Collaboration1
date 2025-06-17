from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='draw_path',               
            executable='draw_path',        # 1번: 그림판 (Float32MultiArray publish)
            name='drawing_pad_node',
            output='screen'
        ),
        Node(
            package='draw_path',
            executable='pose_downsample',               # 2번: 등간격 샘플링
            name='drawing_path_resampler_node',
            output='screen',
            parameters=[{'interval': 3.0}]
        ),
        Node(
            package='draw_path',
            executable='cal_yaw',           # 3번: yaw, PoseArray 생성
            name='path_to_posearray_node',
            output='screen'
        )
    ])
