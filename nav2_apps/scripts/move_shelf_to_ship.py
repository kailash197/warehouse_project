import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

from geometry_msgs.msg import PoseStamped, Polygon, Point32
from std_msgs.msg import String
from attach_shelf.srv import GoToLoading

from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import numpy as np
import time


class WarehouseNavigator(Node):
    def __init__(self):
        super().__init__('warehouse_navigator')
        self.navigator = BasicNavigator()

        self.init_position = [-0.10, 0.0, 0.0]
        self.pre_loading_position = [5.55, -0.0, -1.4]
        self.loading_position = [5.55, -0.5, -1.4]
        self.post_loading_position = [5.55, 0.25, 3.14]
        self.pre_shipping_position = [2.45, 0.0, 1.5]
        self.shipping_position = [2.45, 1.45, 1.5]
        self.post_shipping_position = [2.45, 0.0, -1.5]   

        self.service_client = self.create_client(GoToLoading, '/approach_shelf')
        self.feedback = None
        self.current_result = True

         # Define the nodes responsible for global and local costmaps
        self.global_costmap_node = '/global_costmap/global_costmap'
        self.local_costmap_node = '/local_costmap/local_costmap'
        self.parameter_name = 'robot_radius'
        
        self.elevator_down_pub_ = self.create_publisher(String, '/elevator_down', 10)

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
            if feedback and i % 15 == 0:
                self.get_logger().info('Estimated time: ' +
                      '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) +
                      ' seconds.')
        self.exit_on_failure(self.init_position)

    def update_robot_footprint(self, node_name, radius):
        # https://www.theconstruct.ai/how-to-set-get-parameters-from-another-node-ros2-humble-python-tutorial/
        client = self.create_client(SetParameters, node_name+'/set_parameters')       
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Parameter service for {node_name} not available.")
        request = SetParameters.Request()
        param = Parameter()
        param.name = self.parameter_name
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = radius

        request.parameters.append(param)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Successfully updated {self.parameter_name} to {radius} for {node_name}.")
        else:
            self.get_logger().error(f"Failed to update {self.parameter_name} for {node_name}.")
    
    def update_robot_footprints(self, radius):
        return
        self.update_robot_footprint(self.global_costmap_node, radius)
        self.update_robot_footprint(self.local_costmap_node, radius)
    

    # def update_robot_footprint(self, radius):
    #     self.robot_radius = radius
    #     self.get_logger().info(f'Robot footprint updated: Radius = {self.robot_radius} m')
    
    # def update_robot_footprints(self):
    #     message = Polygon()
    #     point = Point32()
    #     angles = np.arange(0, 2 * np.pi, 0.1)
    #     for angle in angles:
    #         point.x = self.robot_radius * math.cos(angle)
    #         point.y = self.robot_radius * math.sin(angle)
    #         message.points.append(point)
    #     self.global_footprint_pub_.publish(message)
    #     self.local_footprint_pub_.publish(message)
    #     self.get_logger().info(f'Robot footprint updated: Radius = {self.robot_radius} m')
    
    def monitor_backup(self):
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Distance traveled: {feedback.distance_traveled}')
        self.exit_on_failure(self.init_position)
    
    def monitor_spin(self):
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Spin angle traveled: {feedback.angular_distance_traveled}')
        self.exit_on_failure(self.init_position)

    def main(self, args=None):
        # Initialize robot position & Wait for navigation to activate
        self.set_initial_pose(self.init_position)
        self.navigator.waitUntilNav2Active()

        # Navigate to loading position
        if self.current_result:            
            self.get_logger().info('Navigating to pre loading position...')
            self.go_to_pose(self.pre_loading_position)
        if self.current_result:            
            self.get_logger().info('Navigating to loading position...')
            self.go_to_pose(self.loading_position)
        
        # Attach shelf & increase robot footprint
        if self.current_result:
            self.get_logger().info('Arrived at loading position. Calling service to load shelf...')
            self.attach_shelf()
            self.update_robot_footprints(0.50)
        
        # Navigate to shipping positions
        if self.current_result:
            self.get_logger().info('Navigating to post loading position...')
            self.go_to_pose(self.post_loading_position)
        if self.current_result:
            self.get_logger().info('Navigating to pre-shipping position...')
            self.go_to_pose(self.pre_shipping_position)
        if self.current_result:
            self.get_logger().info('Navigating to shipping position...')
            self.go_to_pose(self.shipping_position)
        
        # Detach shelf & lower robot footprint
        if self.current_result:
            self.detach_shelf()
            self.update_robot_footprints(0.20)
        
        # Backup and spin
        if self.current_result:
            self.get_logger().info(f'Spinning robot around...')
            self.navigator.spin(spin_dist=3.14)
            self.monitor_spin()

        # if self.current_result:
        #     backup_distance = 1.10
        #     time.sleep(1)
        #     self.get_logger().info(f'Going backwards: {backup_distance}m')
        #     self.navigator.backup(backup_dist=backup_distance, backup_speed=0.15)
        #     self.monitor_backup()
        
        # if self.current_result:
        #     self.get_logger().info(f'Spinning robot around...')
        #     self.navigator.spin(spin_dist=1.5)
        #     self.monitor_spin()
        if self.current_result:
            self.get_logger().info('Navigating to post shipping position...')
            self.go_to_pose(self.post_shipping_position)
        # Return to initial position
        if self.current_result:
            self.go_to_pose(self.init_position)
            self.get_logger().info('Returned to initial position. Shutting down...')

if __name__ == '__main__':
    rclpy.init()
    navigator = WarehouseNavigator()
    navigator.main()
    rclpy.shutdown()
