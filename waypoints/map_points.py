import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
import math
import time

class ArrowPublisher(Node):
    def __init__(self):
        super().__init__('arrow_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'arrow_pose', 10)
        self.broadcaster = TransformBroadcaster(self)

    def publish_arrow(self, x, y, yaw):
        # Create PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'  # Assuming the arrow is in the map frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)

        # Publish PoseStamped message
        self.publisher_.publish(pose)

    def yaw_to_quaternion(self, yaw):
        # Convert yaw angle to quaternion
        quaternion = TransformStamped()
        quaternion.transform.rotation.x = 0.0
        quaternion.transform.rotation.y = 0.0
        quaternion.transform.rotation.z = math.sin(yaw / 2.0)
        quaternion.transform.rotation.w = math.cos(yaw / 2.0)

        return quaternion.transform.rotation

def main(args=None):
    rclpy.init(args=args)

    node = ArrowPublisher()

    # Example data: x, y, yaw
    data_list = [
        (1.0, 1.0, math.pi / 4),  # 45 degrees
        (2.0, 2.0, math.pi / 2),  # 90 degrees
        (3.0, 3.0, 3 * math.pi / 4),  # 135 degrees
        (4.0, 4.0, math.pi),  # 180 degrees
        (5.0, 5.0, -math.pi / 4),  # -45 degrees
    ]

    try:
        while rclpy.ok():
            for data in data_list:
                node.publish_arrow(*data)
                time.sleep(3)  # Publish every 3 seconds

            rclpy.spin_once(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
