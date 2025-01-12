import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Polygon, Point32
from std_msgs.msg import String
from attach_shelf.srv import GoToLoading

from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import numpy as np


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
        """
        Create a PoseStamped message from 2D pose (x, y, theta).
        
        :param posex: X coordinate
        :param posey: Y coordinate
        :param posetheta: Orientation (yaw) in radians
        :param frame_id: Frame ID for the pose (default: 'map')
        :param stamp: Optional ROS2 timestamp. If None, defaults to 0.
        :return: PoseStamped message
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = stamp if stamp else rclpy.time.Time().to_msg()

        # Set position
        pose_stamped.pose.position.x = posex
        pose_stamped.pose.position.y = posey
        pose_stamped.pose.position.z = 0.0  # Default to 0 for 2D pose

        # Convert yaw (theta) to quaternion
        quaternion = quaternion_from_euler(0, 0, posetheta)  # roll, pitch, yaw
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        return pose_stamped

    def set_initial_pose(self, position):
        init_pose = self.create_pose_stamped(position[0], position[1], position[2])
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.setInitialPose(init_pose)

    def go_to_pose(self, position):
        target_pose = self.create_pose_stamped(position[0], position[1], position[2])
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(target_pose)
        self.monitor_task()

    def attach_shelf(self):
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the /approach_shelf service to become available...')

        request = GoToLoading.Request()
        request.attach_to_shelf = True

        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

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
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 15 == 0:
                self.get_logger().info('Estimated time: ' +
                      '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) +
                      ' seconds.')

    def exit_on_failure(self, position):
        result = self.navigator.getResult()
        if result == TaskResult.CANCELED:
            self.get_logger().info('Task was canceled. Returning to staging point...')
            self.go_to_pose(position)
            self.get_logger().info('Exiting...')
            exit(0)            
        elif result == TaskResult.FAILED:
            self.get_logger().info('Task failed!')
            exit(-1)

    def update_robot_footprint(self, radius):
        message = Polygon()
        point = Point32()
        angles = np.arange(0, 2 * np.pi, 0.1)
        for angle in angles:
            point.x = radius * math.cos(angle)
            point.y = radius * math.sin(angle)
            message.points.append(point)
        
        self.global_footprint_pub_.publish(message)
        self.local_footprint_pub_.publish(message)
        self.get_logger().info(f'Robot footprint updated: Radius = {radius} m')

    def main(self, args=None):
        # Initialize robot position
        self.set_initial_pose(self.init_position)

        # Wait for navigation to activate
        self.navigator.waitUntilNav2Active()

        # Navigate to loading position
        self.get_logger().info('Navigating to loading position...')
        self.go_to_pose(self.loading_position)
        self.exit_on_failure(self.init_position)
        
        # Attach shelf & increase robot footprint
        self.get_logger().info('Arrived at loading position. Calling service to load shelf...')
        self.attach_shelf()
        self.update_robot_footprint(0.50)

        # Navigate to shipping position
        self.get_logger().info('Navigating to shipping position...')
        self.go_to_pose(self.shipping_position)
        self.exit_on_failure(self.init_position)

        # Detach shelf & lower robot footprint
        self.detach_shelf()
        self.update_robot_footprint(0.35)

        # Return to initial position
        self.go_to_pose(self.init_position)
        self.exit_on_failure(self.init_position)


if __name__ == '__main__':
    rclpy.init()
    navigator = WarehouseNavigator()
    navigator.main()
