import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
import rclpy
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Shelf positions for picking
shelf_positions = {
    "shelf_A": [-3.829, -7.604],
    "shelf_B": [-3.791, -3.287],
    "shelf_C": [-3.791, 1.254],
    "shelf_D": [-3.24, 5.861]}

# Shipping destination for picked products
shipping_destinations = {
    "recycling": [-0.205, 7.403],
    "pallet_jack7": [-0.073, -8.497],
    "conveyer_432": [6.217, 2.153],
    "frieght_bay_3": [-6.349, 9.147]}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''

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
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'shelf_C'
    request_destination = 'pallet_jack7'
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = create_pose_stamped(posex=0.0, posey= 0.0, posetheta=0.0)
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = create_pose_stamped(posex=1.0, posey= 0.0, posetheta=0.0)
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    print('at 1, 0')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ()...')
        shipping_destination = create_pose_stamped(posex=2.0, posey= 0.0, posetheta=0.0)
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(shipping_destination)

    elif result == TaskResult.CANCELED:
        print('Task at was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at  failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()