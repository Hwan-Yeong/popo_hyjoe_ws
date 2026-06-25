#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('inusensor_ros2_driver')
    
    # --- URDF / TF 관련 추가 설정 ---
    # xacro 파일을 읽어서 robot_description 파라미터로 변환합니다.
    xacro_path = os.path.join(get_package_share_directory('inusensor_ros2_driver'), 'urdf', 'm4_51s.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_path).toxml()
    
    # TF를 뿌려줄 robot_state_publisher 노드 정의
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{'robot_description': robot_description_config}]
    )
    # ------------------------------

    # Declare launch arguments (원본 그대로 유지)
    config_file_arg = DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([pkg_share, 'config', 'sensor_config.yaml']),
        description='Path to sensor configuration file'
    )
    
    namespace_arg = DeclareLaunchArgument('namespace',default_value='camera',
        description='Namespace for the sensor node'
    )
    
    publish_depth_arg = DeclareLaunchArgument('publish_depth',default_value='true',
        description='Enable depth stream publishing'
    )
    
    publish_rgb_arg = DeclareLaunchArgument('publish_rgb',default_value='true',
        description='Enable RGB stream publishing'
    )
    
    publish_imu_arg = DeclareLaunchArgument('publish_imu',default_value='false',
        description='Enable IMU data publishing'
    )
    
    rgh_depth_register = DeclareLaunchArgument('rgh_depth_register',default_value='false',
            description='Enable rgh_depth_register publishing'
    )

    
    # Publishing rate arguments
    use_timer_mode_arg = DeclareLaunchArgument('use_timer_mode',default_value='true',
            description='Timer based publisher mode set'
    )
    timer_frequency_arg = DeclareLaunchArgument('timer_frequency',default_value='15.0',
            description='Timer callback worgin Hz'
    )

    # Image resolution settings
    rgb_resolution_mode_arg = DeclareLaunchArgument('rgb_resolution_mode',default_value='1',
            description='RGB image\' resolution set... 0 : 1280 x 720, 1: 640 x 480 , 2: 640 x 360'
    )

    depth_resolution_mode_arg = DeclareLaunchArgument('depth_resolution_mode',default_value='1',
            description='Depth image\' resolution set...  0 : 1080 x 720, 1: 848 x 480, 2: 544 x 360'
    )


    
    # InuSensor node
    inusensor_node = Node(
        package='inusensor_ros2_driver',
        executable='inusensor_ros2_driver_node',
        name='inusensor_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # Pub enable/disable
                'publish_depth': LaunchConfiguration('publish_depth'),
                'publish_rgb': LaunchConfiguration('publish_rgb'),
                'publish_imu': LaunchConfiguration('publish_imu'),
                'rgh_depth_register': LaunchConfiguration('rgh_depth_register'),
                
                # Timer mode setup
                'use_timer_mode': LaunchConfiguration('use_timer_mode'),
                'timer_frequency': LaunchConfiguration('timer_frequency'),

                # Image resolution setup
                'rgb_resolution_mode': LaunchConfiguration('rgb_resolution_mode'),
                'depth_resolution_mode': LaunchConfiguration('depth_resolution_mode'),
                
                # ========== image_transport QoS 설정 ==========
                # image_transport가 이 파라미터들을 읽어서 QoS 적용
                'image_transport.default_qos_reliability': 'best_effort',  # 'reliable' or 'best_effort'
                'image_transport.default_qos_history': 'keep_last',
                'image_transport.default_qos_depth': 1,
                
                # 압축 설정
                'image_transport.compressed.jpeg_quality': 75,
                'image_transport.compressed.png_level': 3,

            }
        ],
        remappings=[
                # Topic remappings (필요시)
                # ('/depth/image_raw', '/camera/depth/image_raw'),
                # ('/rgb/image_raw', '/camera/rgb/image_raw'),
                # ('/camera/imu', '/camera/imu'),
        ],
        output='screen',
        emulate_tty=True,
        respawn=False,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        publish_depth_arg,
        publish_rgb_arg,
        publish_imu_arg,
        rgh_depth_register,
        use_timer_mode_arg,    
        timer_frequency_arg,
        rgb_resolution_mode_arg,
        depth_resolution_mode_arg,
        # 추가된 노드들
        robot_state_publisher_node,
        inusensor_node
    ])