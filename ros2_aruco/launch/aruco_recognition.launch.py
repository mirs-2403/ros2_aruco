import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters.yaml'
    )

    aruco_server = Node(
        package='ros2_aruco',
        executable='aruco_server',
        #parameters=[aruco_params]
    )

    aruco_server_2 = Node(
        package='ros2_aruco',
        executable='aruco_server_2',
        #parameters=[aruco_params]
    )

    aruco_tf_node = Node(
        package='ros2_aruco',
        executable='aruco_tf',
        #parameters=[aruco_params]
    )

    v4l2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node_1',
        namespace='camera_1',
        output='screen',
        parameters=[
            {'video_device': '/dev/video4'},
            {'image_size': [640,480]},
            {'pixel_format': 'YUYV'},
            {'camera_frame_id': 'camera_1'}
        ]
    )

    v4l2_node_2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node_2',
        namespace='camera_2',
        output='screen',
        parameters=[
            {'video_device': '/dev/video2'},
            {'image_size': [640,480]},
            {'pixel_format': 'YUYV'},
            {'camera_frame_id': 'camera_2'}
        ]
    )

    return LaunchDescription([
        aruco_server,
        aruco_server_2,
        v4l2_node,
        v4l2_node_2
        #aruco_tf_node
    ])
