import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
import rclpy
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Poses
init_position = [0.0, 0.0, 0.0]
loading_position = [2.55, 0.0, -1.571]
# loading_position2 = [5.59, -0.956, -1.571]
# loading_position3 = [5.56, -1.41, -1.571]
shipping_position = [2.34, 0.65, 1.571]
# shipping_position2 = [2.42, 1.25, 1.571]


def create_pose_stamped(posex, posey, posetheta, frame_id='map', stamp=None):
    """
    Create a PoseStamped message from 2D pose (x, y, theta).
    
    :param posex: X coordinate
    :param posey: Y coordinate
    :param posetheta: Orientation (yaw) in radians
    :param frame_id: Frame ID for the pose (default: 'map')
    :param stamp: Optional ROS2 timestamp. If None, defaults to 0.
    :return: PoseStamped message
    """
    # Create PoseStamped message
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


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set your demo's initial pose
    init_pose = create_pose_stamped(init_position[0], init_position[1], init_position[2])
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(init_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    loading_pose = create_pose_stamped(loading_position[0], loading_position[1], loading_position[2])
    loading_pose.header.frame_id = 'map'
    loading_pose.header.stamp = navigator.get_clock().now().to_msg()
    print('at 1, 0')
    navigator.goToPose(loading_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 15 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        # Rotate the robot by -90 degrees yaw
        print('rotate now')
        navigator.spin(spin_dist=1.57, time_allowance=10)
        # spin(spin_dist=1.57, time_allowance=10, disable_collision_checks=False)
        print('Arrived loading position. Call service to load shelf...')


    elif result == TaskResult.CANCELED:
        print('Task at was canceled. Returning to staging point...')
        init_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(init_pose)

    elif result == TaskResult.FAILED:
        print('Task at  failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()