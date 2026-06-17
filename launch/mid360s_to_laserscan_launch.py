# Livox Mid-360S -> 2D LaserScan 변환 런치 (2D SLAM/Nav2, 수평 장착, 이동 로봇 기준)
#
# 파라미터는 config/mid360s_to_laserscan.yaml 에서 로드한다.
# 다른 설정을 쓰려면 launch 인자 params_file 로 교체:
#   ros2 launch pointcloud_to_laserscan mid360s_to_laserscan_launch.py \
#       params_file:=/path/to/custom.yaml
#
# 전제:
#   - /livox/lidar 가 sensor_msgs/PointCloud2 로 발행 (livox_ros_driver2 의 xfer_format=0)
#   - target_frame(base_link)으로의 TF 존재 (수평 장착이면 회전 0, 위치 오프셋만)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('pointcloud_to_laserscan'),
        'config', 'mid360s_to_laserscan.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=default_params,
            description='변환 노드 파라미터 YAML 경로',
        ),
        DeclareLaunchArgument(
            name='cloud_in', default_value='/livox/lidar',
            description='입력 PointCloud2 토픽',
        ),
        DeclareLaunchArgument(
            name='scan', default_value='/scan',
            description='출력 LaserScan 토픽',
        ),
        DeclareLaunchArgument(
            name='use_sim_time', default_value='false',
            description='시뮬레이션/bag 재생 시 /clock 사용 (params_file 값을 덮어씀)',
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', LaunchConfiguration('cloud_in')),
                ('scan', LaunchConfiguration('scan')),
            ],
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
