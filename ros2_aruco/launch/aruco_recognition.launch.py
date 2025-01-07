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

    aruco_tf_node = Node(
        package='ros2_aruco',
        executable='aruco_tf',
        #parameters=[aruco_params]
    )

    v4l2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[
            {'video_device': '/dev/video2'},
            {'image_width': 680},
            {'image_height': 480},
            {'pixel_format': 'YUYV'},
            {'camera_frame_id': 'camera_frame'}
        ]
    )

    return LaunchDescription([
        aruco_server,
        v4l2_node,
        #aruco_tf_node
    ])
