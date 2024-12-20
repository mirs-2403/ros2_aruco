import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_ros import TransformListener, Buffer, TransformException
import tf_transformations

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # Create TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber to ArUco marker PoseArray
        self.aruco_subscription = self.create_subscription(
            PoseArray,
            'aruco_poses',
            self.aruco_callback,
            10
        )

        # Publisher for PoseStamped in map frame
        self.pose_publisher = self.create_publisher(PoseStamped, 'aruco_pose_in_map', 10)

    def aruco_callback(self, msg):
        try:
            for pose in msg.poses:
                # Get transform from camera to base_link
                self.get_logger().info(f'DID!')
                camera_to_base_link = self.tf_buffer.lookup_transform(
                    'base_link',
                    'my_camera',
                    rclpy.time.Time()
                )

                # Transform Pose to base_link frame
                pose_in_base_link = self.transform_pose(pose, camera_to_base_link)

                # Get transform from base_link to map
                base_link_to_map = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                )

                # Transform Pose to map frame
                pose_in_map = self.transform_pose(pose_in_base_link, base_link_to_map)
                
                # Create PoseStamped to publish
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.pose = pose_in_map.pose

                # Publish the transformed PoseStamped
                self.pose_publisher.publish(pose_stamped)

        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')

    def transform_pose(self, pose, transform):
        """Transforms a Pose using a given TransformStamped."""
        # Extract translation and rotation from transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Convert pose position and orientation to homogeneous matrix
        pose_matrix = tf_transformations.quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        pose_matrix[0, 3] = pose.position.x
        pose_matrix[1, 3] = pose.position.y
        pose_matrix[2, 3] = pose.position.z

        # Convert transform to homogeneous matrix
        transform_matrix = tf_transformations.quaternion_matrix([
            rotation.x, rotation.y, rotation.z, rotation.w
        ])
        transform_matrix[0, 3] = translation.x
        transform_matrix[1, 3] = translation.y
        transform_matrix[2, 3] = translation.z

        # Apply transform to pose
        transformed_matrix = tf_transformations.concatenate_matrices(transform_matrix, pose_matrix)

        # Extract transformed position and orientation
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
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
