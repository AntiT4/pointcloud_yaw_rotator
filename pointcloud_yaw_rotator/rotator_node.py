import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, JointState
import numpy as np
import transformations
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudYawRotator(Node):
    def __init__(self):
        super().__init__('pointcloud_yaw_rotator')
        self.declare_parameter('rotation_deg', 0.0)
        self.rotation_deg = self.get_parameter('rotation_deg').get_parameter_value().double_value
        self.rotation_rad = np.deg2rad(self.rotation_deg)

        # Subscribers
        self.create_subscription(
            PointCloud2,
            '/scan',
            self.pointcloud_callback,
            10
        )
        
        self.create_subscription(
            JointState,
            '/lidar_yaw',
            self.jointstate_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            'rotated_pointcloud',
            10
        )
        self.get_logger().info(f'Initialized with yaw rotation = {self.rotation_deg:.2f} degrees')

    def jointstate_callback(self, msg: JointState):
        """Update yaw angle (in radians) from JointState message"""
        try:
            if "yaw" in msg.name:
                idx = msg.name.index("yaw")
                self.rotation_rad = msg.position[idx]
                self.get_logger().debug(f'Updated yaw from JointState to {np.rad2deg(self.rotation_rad):.2f} degrees')
            else:
                self.get_logger().warn('JointState message does not contain "yaw" joint')
        except Exception as e:
            self.get_logger().error(f'Error parsing JointState message: {e}')

    def pointcloud_callback(self, msg):
        # Read point cloud data
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Rotation matrix around Z axis
        rot_matrix = transformations.rotation_matrix(self.rotation_rad, (0, 0, 1))[:3, :3]

        # Apply rotation
        rotated_points = []
        for pt in points:
            xyz = np.array([pt[0], pt[1], pt[2]])
            rotated_xyz = np.dot(rot_matrix, xyz)
            rotated_points.append(tuple(rotated_xyz))

        # Create rotated pointcloud message
        rotated_msg = pc2.create_cloud_xyz32(msg.header, rotated_points)
        self.publisher.publish(rotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudYawRotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
