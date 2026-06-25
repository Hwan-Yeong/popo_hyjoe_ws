#!/usr/bin/env python3
#
# depth_to_pointcloud 실험용 launch.
#
# 이 launch 는 inusensor_ros2_driver 가 "이미 실행 중"이라고 가정한다.
# (드라이버가 /camera/depth/image_raw, /camera/depth/camera_info 를 발행하고
#  robot_state_publisher 가 camera_link -> ... -> inusensor_depth TF 를 발행함)
#
# 여기서 추가로 띄우는 것:
#   1) base_link -> camera_link static TF  (로봇 본체에 카메라를 붙임)
#   2) depth_to_pointcloud_node            (depth -> PointCloud2 변환)
#
# RViz는 이 launch에서 띄우지 않는다(별도 PC/세션에서 직접 실행).
#
# 실행 예:
#   ros2 launch depth_pointcloud_converter depth_to_pointcloud.launch.py
#   ros2 launch depth_pointcloud_converter depth_to_pointcloud.launch.py launch_static_tf:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- launch arguments ----------------
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth/image_raw',
        description='Input depth image topic (sensor_msgs/Image, 16UC1, mm)')

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/depth/camera_info',
        description='Input depth CameraInfo topic')

    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/camera/depth/points',
        description='Output PointCloud2 topic')

    range_min_arg = DeclareLaunchArgument(
        'range_min', default_value='0.1',
        description='Minimum valid depth [m]')

    range_max_arg = DeclareLaunchArgument(
        'range_max', default_value='5.0',
        description='Maximum valid depth [m]')

    row_step_arg = DeclareLaunchArgument(
        'row_step', default_value='1',
        description='Row downsample step (1 = full resolution)')

    col_step_arg = DeclareLaunchArgument(
        'col_step', default_value='1',
        description='Column downsample step (1 = full resolution)')

    launch_static_tf_arg = DeclareLaunchArgument(
        'launch_static_tf', default_value='true',
        description='Publish base_link -> camera_link static TF')

    # base_link -> camera_link 부착 위치 (사용자 제원)
    cam_x_arg = DeclareLaunchArgument('cam_x', default_value='0.13')
    cam_y_arg = DeclareLaunchArgument('cam_y', default_value='0.0')
    cam_z_arg = DeclareLaunchArgument('cam_z', default_value='0.1')
    cam_roll_arg = DeclareLaunchArgument('cam_roll', default_value='0.0')
    cam_pitch_arg = DeclareLaunchArgument('cam_pitch', default_value='0.0')
    cam_yaw_arg = DeclareLaunchArgument('cam_yaw', default_value='0.0')

    # ---------------- nodes ----------------
    # base_link -> camera_link static transform
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_static_tf',
        condition=IfCondition(LaunchConfiguration('launch_static_tf')),
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    converter_node = Node(
        package='depth_pointcloud_converter',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'output_frame': '',          # 비우면 depth 이미지 frame_id(inusensor_depth) 사용
            'depth_scale': 0.001,        # 16UC1 mm -> m
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'row_step': LaunchConfiguration('row_step'),
            'col_step': LaunchConfiguration('col_step'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        output_topic_arg,
        range_min_arg,
        range_max_arg,
        row_step_arg,
        col_step_arg,
        launch_static_tf_arg,
        cam_x_arg,
        cam_y_arg,
        cam_z_arg,
        cam_roll_arg,
        cam_pitch_arg,
        cam_yaw_arg,
        static_tf_node,
        converter_node,
    ])
