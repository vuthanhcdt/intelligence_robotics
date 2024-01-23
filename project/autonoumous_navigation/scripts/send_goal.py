#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs  # Import để sử dụng hàm do_transform_pose
import time

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_ = self.create_timer(1.0, self.publish_goal_pose)
        self.goal_poses = self.generate_goal_poses()
        self.current_goal_index = 0

    def generate_goal_poses(self):
        # Tạo danh sách chứa 4 điểm goal đặt trước
        goal_poses = []
        goal_poses.append(self.create_pose_stamped(1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        goal_poses.append(self.create_pose_stamped(0.0, 1.0, 0.0, 0.0, 0.0, 1.57))
        goal_poses.append(self.create_pose_stamped(-1.0, 0.0, 0.0, 0.0, 0.0, 3.14))
        goal_poses.append(self.create_pose_stamped(0.0, -1.0, 0.0, 0.0, 0.0, -1.57))
        return goal_poses

    def create_pose_stamped(self, x, y, z, qx, qy, qz):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = 1.0  # Quaternions must be normalized
        pose_stamped.header.frame_id = 'map'
        return pose_stamped

    def publish_goal_pose(self):
        try:
            # Lấy thông tin biến đổi giữa "map" và "base_link"
            transform_stamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Lấy goal hiện tại và chuyển đổi nó từ "map" sang "base_link"
            current_goal = self.goal_poses[self.current_goal_index]
            transformed_goal = tf2_geometry_msgs.do_transform_pose(current_goal, transform_stamped)

            # Xuất bản goal đã được chuyển đổi
            self.publisher_.publish(transformed_goal)
            self.get_logger().info(f'Publishing goal pose: {transformed_goal}')

            # Kiểm tra xem robot đã đến gần đúng goal hay chưa
            distance_to_goal = transformed_goal.pose.position.x**2 + transformed_goal.pose.position.y**2
            if distance_to_goal < 0.1:  # Điều chỉnh giá trị ngưỡng tùy thuộc vào yêu cầu của bạn
                self.get_logger().info(f'Reached goal {self.current_goal_index + 1}, moving to the next goal...')
                self.current_goal_index = (self.current_goal_index + 1) % len(self.goal_poses)

        except Exception as e:
            self.get_logger().warn(f'Error looking up transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    goal_pose_publisher = GoalPosePublisher()

    try:
        rclpy.spin(goal_pose_publisher)
    except KeyboardInterrupt:
        pass

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

