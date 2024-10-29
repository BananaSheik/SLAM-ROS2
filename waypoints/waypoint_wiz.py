import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from machine_locations import *
import math

class WaypointPublisher(Node):

    def __init__(self, waypoints):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'way_viz_marker', 10)
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.pose_msg = PoseStamped()  # Initialize pose_msg here
        self.timer = self.create_timer(1.0, self.publish_next_waypoint)

    def publish_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            x, y, yaw = self.waypoints[self.current_waypoint_index]

            # Populate the PoseStamped message
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_msg.header.frame_id = 'map'
            self.pose_msg.pose.position.x = x
            self.pose_msg.pose.position.y = y
            self.pose_msg.pose.position.z = 0.0
            # Assuming yaw is in radians
            self.pose_msg.pose.orientation.x = 0.0
            self.pose_msg.pose.orientation.y = 0.0
            
            self.pose_msg.pose.orientation.z = math.sin(yaw / 2)
            self.pose_msg.pose.orientation.w = math.cos(yaw / 2)

            # Publish the message
            self.publisher_.publish(self.pose_msg)
            print(self.waypoints[self.current_waypoint_index])
            self.get_logger().info(f'Published waypoint {self.current_waypoint_index + 1}')

            self.current_waypoint_index += 1
        else:
            self.get_logger().info('All waypoints published')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    cm_idx = 0
    df_idx = 0
    df_can_idx = 0
    switch_ab = 'B'

    waypoints=[]

    for i in range(100):
        cm_idx = (i%36)+1

        if i%40==0:
            switch_ab = 'B' if switch_ab=='A' else 'A'
    
        df_idx=df_idx%10+1
        
        waypoints.append(create_pose(carding_machines[cm_idx][0]))
        waypoints.append(create_pose(carding_machines[cm_idx][1]))

        waypoints.append(create_pose(support_points[switch_ab][df_idx]))

        # waypoints.append(create_pose(draw_frames[switch_ab][df_idx][df_can_idx]))
        waypoints.append(create_pose(draw_frames[0][df_idx]))

        if df_idx%10==0:
            df_can_idx+=1
            df_can_idx=df_can_idx%4
        
    waypoint_publisher = WaypointPublisher(waypoints)

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
