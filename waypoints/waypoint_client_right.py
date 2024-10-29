import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import random

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from tf_transformations import quaternion_from_euler
from machine_locations import *
import csv  # Import CSV module
import signal  # Import signal module for handling termination signals

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._current_waypoint_index = 0
        self._waypoints = []
        self._timer = None  
        self._start_time = None  # Attribute to hold the start time of waypoint execution
        self.cycle_number = 1

        # Initialize CSV writer
        self.csv_file = open('waypoints_data.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.csv_writer.writerow(['Cycle', 'Waypoint', 'Time Taken'])

    def send_waypoints(self, waypoints):
        self._waypoints = waypoints
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self._current_waypoint_index < len(self._waypoints):
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = [self._waypoints[self._current_waypoint_index]]

            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info('All waypoints have been visited.')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._start_time = self.get_clock().now()  # Record the start time of waypoint execution

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result is not None:
            duration = self.get_clock().now() - self._start_time
            self.get_logger().info('Time taken to reach waypoint {}: {:.2f} seconds'.format(self._current_waypoint_index + 1, duration.nanoseconds / 1e9))
            
            try:
                # Write data to CSV
                self.csv_writer.writerow([self.cycle_number, self._current_waypoint_index + 1, duration.nanoseconds / 1e9])
                self.csv_file.flush()  # Flush after each write
            except Exception as e:
                self.get_logger().error(f"Error writing to CSV: {e}")

            self._current_waypoint_index += 1
            if self._current_waypoint_index % 5 == 0:
                self.cycle_number += 1  # Increment cycle number every 5 waypoints
                
            self.send_next_waypoint()
        else:
            self.get_logger().info('Result failed')

    def destroy(self):
        self.csv_file.flush()
        self.csv_file.close()
        super().destroy_node()

def create_pose(waypoint):
    x, y, yaw = waypoint
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = Time(sec=0, nanosec=0)
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    q = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    return pose

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WaypointClient()

    cm_idx = 0
    df_idx = 0
    df_can_idx = 0
    switch_ab = 'B'

    waypoints = []
    unique_numbers = random.sample(range(18), 18)
    

    for i in range(0,18):
        cm_idx = (i % 36) + 1

        if i % 40 == 0:
            switch_ab = 'B' if switch_ab == 'A' else 'A'

        df_idx = df_idx % 5 + 1
        df2_idx = df_idx+5

        waypoints.append(create_pose(carding_machines[cm_idx][0]))
        waypoints.append(create_pose(carding_machines[cm_idx][1]))
        waypoints.append(create_pose(support_points[switch_ab][df_idx]))
        waypoints.append(create_pose(draw_frames[switch_ab][df_idx][df_can_idx]))
        waypoints.append(create_pose(draw_frames['0' + switch_ab][df_idx]))

        if df_idx % 5 == 0:
            df_can_idx += 1
            df_can_idx = df_can_idx % 4

    waypoint_client.send_waypoints(waypoints)

    def shutdown_handler(sig, frame):
        waypoint_client.destroy_node()
        rclpy.shutdown()

    # Register the shutdown handler for SIGINT (Ctrl+C) and SIGTERM (Termination signal)
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    rclpy.spin(waypoint_client)

    waypoint_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
