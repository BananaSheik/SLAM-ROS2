import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from filterpy.kalman import ExtendedKalmanFilter
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.fused_state_pub = self.create_publisher(Odometry, 'fused_state', 10)
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=3)
        self.ekf.x = [0, 0, 0]  # Initial state
        self.ekf.F = np.eye(3)  # State transition matrix
        self.ekf.H = np.eye(3)  # Measurement function matrix
        self.ekf.P *= 1000  # Covariance matrix
        self.ekf.R = np.diag([0.01, 0.01, 0.01])  # Measurement noise covariance matrix
        Q = np.diag([0.001, 0.001, 0.001]) 
        self.ekf.Q = Q


    def odom_callback(self, msg):
    # Process odometry data and update EKF prediction step
    # Extract relevant information from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # Perform the prediction step of the EKF
        self.ekf.predict()
        # Update the state vector with the predicted values
        self.ekf.x = [x, y, z]  # Assuming a 3-dimensional state vector


    def imu_callback(self, msg):
    # Process IMU data and update EKF measurement update step
    # Extract relevant information from the Imu message
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        # Define the measurement vector
        z = [ax, ay, az]
        # Compute the Jacobian matrix of the measurement function
        H = self.compute_measurement_jacobian()
        # Perform the measurement update step of the EKF
        self.ekf.update(z=z, HJacobian=H, Hx=self.ekf.x)

    def compute_measurement_jacobian(self):
        # Define the measurement function matrix (Jacobian)
        # This function computes the Jacobian matrix based on the current state
        # Example:
        H = np.eye(3)  # In this example, the Jacobian matrix is an identity matrix
        return H

    def publish_fused_state(self):
        # Publish fused state estimate using EKF
        fused_state_msg = Odometry()
        # Populate fused_state_msg with fused state estimate
        fused_state_msg.header.stamp = self.get_clock().now().to_msg()
        fused_state_msg.header.frame_id = 'map'  # Adjust frame ID as needed
        fused_state_msg.pose.pose.position.x = self.ekf.x[0]
        fused_state_msg.pose.pose.position.y = self.ekf.x[1]
        fused_state_msg.pose.pose.position.z = self.ekf.x[2]
        # Set covariance values (if available)
        # fused_state_msg.pose.covariance[0] = ...
        # fused_state_msg.pose.covariance[7] = ...
        # fused_state_msg.pose.covariance[14] = ...
        # fused_state_msg.pose.covariance[21] = ...
        # fused_state_msg.pose.covariance[28] = ...
        # fused_state_msg.pose.covariance[35] = ...
        # Publish the fused state estimate
        self.fused_state_pub.publish(fused_state_msg)


def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()
    rclpy.spin(sensor_fusion_node)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
