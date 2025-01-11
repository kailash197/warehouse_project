
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from tf_transformations import quaternion_from_euler

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from attach_shelf.srv import GoToLoading

class WarehouseNavigator:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        self.node = rclpy.create_node('attach_client')
        self.service_client = self.node.create_client(GoToLoading, '/approach_shelf')

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

    def call_attach_service(self):
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for the /approach_shelf service to become available...')

        request = GoToLoading.Request()
        request.attach_to_shelf = True

        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            response = future.result()
            if response.complete:
                self.node.get_logger().info('Shelf successfully attached.')
            else:
                self.node.get_logger().info('Failed to attach to the shelf.')
        else:
            self.node.get_logger().error('Service call failed.')

    def monitor_task(self):
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 15 == 0:
                self.node.get_logger().info('Estimated time of arrival: ' +
                      '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) +
                      ' seconds.')

    def handle_result(self, init_position):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.node.get_logger().info('Arrived at loading position. Calling service to load shelf...')
            self.call_attach_service()
        elif result == TaskResult.CANCELED:
            self.node.get_logger().info('Task was canceled. Returning to staging point...')
            self.go_to_pose(init_position)
        elif result == TaskResult.FAILED:
            self.node.get_logger().info('Task failed!')
            exit(-1)

    def main(self, init_position, loading_position):
        self.set_initial_pose(init_position)

        # Wait for navigation to activate
        self.navigator.waitUntilNav2Active()

        # Navigate to loading position
        self.node.get_logger().info('Navigating to loading position...')
        self.go_to_pose(loading_position)
        self.monitor_task()
        self.handle_result(init_position)


if __name__ == '__main__':
    init_position = [0.0, 0.0, 0.0]
    loading_position = [5.65, -0.20, -1.4]
    navigator = WarehouseNavigator()
    navigator.main(init_position, loading_position)
