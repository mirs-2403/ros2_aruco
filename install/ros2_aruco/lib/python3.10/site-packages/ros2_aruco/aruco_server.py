import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from ros2_aruco_interfaces.srv import GetMarkerPose
from tf2_ros import TransformListener, Buffer, TransformException
import tf_transformations
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

class ArucoService(Node):
    def __init__(self):
        super().__init__('aruco_service')
        self.srv = self.create_service(GetMarkerPose, 'get_marker_pose', self.get_marker_pose_callback)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('camera1_frame', 'camera_1')
        self.declare_parameter('camera2_frame', 'camera_2')
        self.declare_parameter('map_frame', 'map')

        self.camera1_frame = self.get_parameter('camera1_frame').get_parameter_value().string_value
        self.camera2_frame = self.get_parameter('camera2_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.declare_parameter('marker_size', 0.0625)
        self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250')

        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dictionary_id_name = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.__getattribute__(dictionary_id_name))
        self.aruco_parameters = cv2.aruco.DetectorParameters()

        self.create_subscription(CameraInfo, '/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_callback)  # Assume that camera parameters will remain the same

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            self.detected_markers = [(marker_id[0], tvec, rvec) for marker_id, tvec, rvec in zip(marker_ids, tvecs, rvecs)]
            self.detect_time = self.get_clock().now()

    def get_marker_pose_callback(self, request, response):
        camera_frame = self.camera1_frame if request.camera_id == 1 else self.camera2_frame

        try:
            # カメラからmapへの変換を取得
            camera_to_map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_frame,
                rclpy.time.Time()
            )

            if not hasattr(self, 'detected_markers') or len(self.detected_markers) == 0:
                response.success = False
                return response

            # 現在時間と最後にマーカーを検出した時間の差がmax_time_diffより大きい場合は失敗
            max_time_diff = 0.5
            if (self.get_clock().now() - self.detect_time).nanoseconds / 1e9 > max_time_diff:
                response.success = False
                return response

            marker_id, tvec, rvec = self.detected_markers[0]

            pose = PoseStamped()
            pose.pose.position.x = tvec[0][0]
            pose.pose.position.y = tvec[0][1]
            pose.pose.position.z = tvec[0][2]

            rot_matrix = np.eye(4)
            rot_matrix[0:3, 0:3] = cv2.Rodrigues(rvec[0])[0]

            quat = tf_transformations.quaternion_from_matrix(rot_matrix)

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            if request.camera_id == 3:
                pose.header.frame_id = 'base_link'
                response.pose = pose
            else:
                # Poseをmapフレームに変換
                transformed_pose = self.transform_pose(pose, camera_to_map_transform)
                response.pose = transformed_pose

            response.success = True
            response.marker_id = int(marker_id)
        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')
            response.success = False

        return response

    def transform_pose(self, pose, transform):
        # transformStampedを使用してPoseを変換
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Poseの位置と方向を同次行列に変換
        pose_matrix = tf_transformations.quaternion_matrix([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ])
        pose_matrix[0, 3] = pose.pose.position.x
        pose_matrix[1, 3] = pose.pose.position.y
        pose_matrix[2, 3] = pose.pose.position.z

        # 変換を同次行列に変換
        transform_matrix = tf_transformations.quaternion_matrix([
            rotation.x, rotation.y, rotation.z, rotation.w
        ])
        transform_matrix[0, 3] = translation.x
        transform_matrix[1, 3] = translation.y
        transform_matrix[2, 3] = translation.z

        # 変換をPoseに適用
        transformed_matrix = tf_transformations.concatenate_matrices(transform_matrix, pose_matrix)

        # 変換された位置と方向を抽出
        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = transform.header.frame_id
        transformed_pose.header.stamp = self.get_clock().now().to_msg()
        transformed_pose.pose.position.x = transformed_matrix[0, 3]
        transformed_pose.pose.position.y = transformed_matrix[1, 3]
        transformed_pose.pose.position.z = transformed_matrix[2, 3]
        transformed_quaternion = tf_transformations.quaternion_from_matrix(transformed_matrix)
        transformed_pose.pose.orientation.x = transformed_quaternion[0]
        transformed_pose.pose.orientation.y = transformed_quaternion[1]
        transformed_pose.pose.orientation.z = transformed_quaternion[2]
        transformed_pose.pose.orientation.w = transformed_quaternion[3]

        return transformed_pose


def main(args=None):
    rclpy.init(args=args)
    node = ArucoService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()