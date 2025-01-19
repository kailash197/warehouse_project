import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class WarehouseNavigator(Node):
    def __init__(self):
        super().__init__('warehouse_navigator')
        self.navigator = BasicNavigator()

        self.init_position = [-0.10, 0.0, 0.0]
        self.pre_loading_position = [5.55, -0.0, -1.4]
        self.current_result = True

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
    
    def exit_on_failure(self, position):
        result = self.navigator.getResult()
        if result == TaskResult.CANCELED:
            self.get_logger().info('Task was canceled. Returning to provided pose...')
            self.go_to_pose(position)
            self.current_result = False
        elif result == TaskResult.FAILED:
            self.get_logger().info('Task failed!')
            self.current_result = False
        elif result == TaskResult.SUCCEEDED:
            self.get_logger().info('Task succeeded.')
            self.current_result = True

    def monitor_task(self):
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 30 == 0:
                self.get_logger().info('Estimated time: ' +
                      '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) +
                      ' seconds.')
        self.exit_on_failure(self.init_position)

    def main(self, args=None):
        # Initialize robot position & Wait for navigation to activate
        self.set_initial_pose(self.init_position)
        self.navigator.waitUntilNav2Active()

        # Navigate to loading position
        if self.current_result:            
            self.get_logger().info('Navigating to pre loading position...')
            self.go_to_pose(self.pre_loading_position)
        self.get_logger().info('End of program. Shutting down...')


if __name__ == '__main__':
    rclpy.init()
    navigator = WarehouseNavigator()
    navigator.main()
    rclpy.shutdown()
