import sys
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.srv import GetMarkerPose


class ArucoClient(Node):
    def __init__(self):
        super().__init__('aruco_client')
        self.client = self.create_client(GetMarkerPose, 'get_marker_pose_2')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMarkerPose.Request()

    def send_request(self, camera_id):
        self.req.camera_id = camera_id
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    client = ArucoClient()
    camera_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    response = client.send_request(camera_id)
    if response.success:
        print(f'Marker ID: {response.marker_id}')
        print(f'Pose: {response.pose}')
    else:
        print('Failed to get marker pose')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()