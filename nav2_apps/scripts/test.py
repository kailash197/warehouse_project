import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Polygon, Point32
from std_msgs.msg import String
from attach_shelf.srv import GoToLoading

from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import numpy as np  # For floating-point range

class WarehouseNavigator(Node):
    def __init__(self):
        super().__init__('warehouse_navigator')
        self.navigator = BasicNavigator()

        self.init_position = [0.0, 0.0, 0.0]
        self.loading_position = [5.65, -0.20, -1.4]
        self.shipping_position = [2.45, 1.45, 1.57]

        self.service_client = self.create_client(GoToLoading, '/approach_shelf')
        
        self.elevator_down_pub_ = self.create_publisher(String, '/elevator_down', 10)
        self.global_footprint_pub_ = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_footprint_pub_ = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

    def create_pose_stamped(self, posex, posey, posetheta, frame_id='map', stamp=None):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = stamp if stamp else self.get_clock().now().to_msg()

        pose_stamped.pose.position.x = posex
        pose_stamped.pose.position.y = posey
        pose_stamped.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, posetheta)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        return pose_stamped

    def set_initial_pose(self, position):
        init_pose = self.create_pose_stamped(position[0], position[1], position[2])
        self.navigator.setInitialPose(init_pose)

    def go_to_pose(self, position):
        target_pose = self.create_pose_stamped(position[0], position[1], position[2])
        self.navigator.goToPose(target_pose)
        self.monitor_task()

    def attach_shelf(self):
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the /approach_shelf service to become available...')

        request = GoToLoading.Request()
        request.attach_to_shelf = True

        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.complete:
                self.get_logger().info('Shelf successfully attached.')
            else:
                self.get_logger().info('Failed to attach to the shelf.')
        else:
            self.get_logger().error('Service call failed.')

    def detach_shelf(self):
        message = String()
        message.data = ''
        self.elevator_down_pub_.publish(message)
        self.get_logger().info('Shelf successfully detached.')

    def monitor_task(self):
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'Estimated time: {Duration.from_msg(feedback.estimated_time_remaining).seconds()} seconds.'
                )

    def exit_on_failure(self, position):
        result = self.navigator.getResult()
        if result in (TaskResult.CANCELED, TaskResult.FAILED):
            self.get_logger().info('Task failed. Returning to initial position...')
            self.go_to_pose(position)
            rclpy.shutdown()
            exit(0)

    def update_robot_footprint(self, radius):
        message = Polygon()
        for angle in np.arange(0, 2 * math.pi, 0.1):
            point = Point32()
            point.x = radius * math.cos(angle)
            point.y = radius * math.sin(angle)
            point.z = 0.0
            message.points.append(point)

        self.global_footprint_pub_.publish(message)
        self.local_footprint_pub_.publish(message)
        self.get_logger().info(f'Robot footprint updated: Radius = {radius} m')

    def main(self, args=None):
        self.set_initial_pose(self.init_position)
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('Navigating to loading position...')
        self.go_to_pose(self.loading_position)
        self.exit_on_failure(self.init_position)

        self.get_logger().info('Arrived at loading position. Attaching shelf...')
        self.attach_shelf()
        self.update_robot_footprint(0.45)

        self.get_logger().info('Navigating to shipping position...')
        self.go_to_pose(self.shipping_position)
        self.exit_on_failure(self.init_position)

        self.detach_shelf()
        self.update_robot_footprint(0.35)

        self.go_to_pose(self.init_position)
        self.exit_on_failure(self.init_position)


if __name__ == '__main__':
    rclpy.init()
    navigator = WarehouseNavigator()
    navigator.main()
    rclpy.shutdown()
