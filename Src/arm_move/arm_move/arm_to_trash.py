#!/usr/bin/env python3



############# Colission Box Tisch #############
# 0.75  0.75    0.04

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()

    # Create node for this example
    node = Node("move_arm_trash")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Define joint names, base link, and end effector for the Tiago robot
    joint_names = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'torso_lift_joint'
    ]
    base_link_name = 'base_link'
    end_effector_name = 'gripper_grasping_frame'

    # Create MoveIt2 instance for the arm
    moveit2_arm = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name=base_link_name,
        end_effector_name=end_effector_name,
        group_name="arm_torso",
        callback_group=callback_group
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    object_id = "tisch_box"
    position = [0.7, 0.0, 0.0]
    quat_xyzw = [0.0, 0.0, 0.0, 0.0]
    dimensions = [0.75, 0.75, 0.2]

    moveit2_arm.add_collision_box(
                id=object_id, position=position, quat_xyzw=quat_xyzw, size=dimensions
            )
    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.pose.orientation.w = 0.5024070620536804  # unit quaternion (no rotation)
    target_pose.pose.position.x = 0.6122882962226868  # Replace with your desired X position
    target_pose.pose.position.y = -0.01808951425552368 # Replace with your desired Y position
    target_pose.pose.position.z = 0.3752094507217407  # Replace with your desired Z position

    # Set the pose target for the arm
    node.get_logger().info('Moving to new pose...')
    moveit2_arm.move_to_pose(
        position=[target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z],
        quat_xyzw=[target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w],
        cartesian=False,
        cartesian_max_step=0.0025,
        cartesian_fraction_threshold=0.0,
    )

    # Wait for execution to start
    moveit2_arm.wait_until_executed()

    node.get_logger().info('Movement executed')

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == '__main__':
    main()