#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import PoseStamped, PointStamped

class ArmMove(Node):
    def __init__(self):
        super().__init__('arm_move')
        self.subscription = self.create_subscription(
            PointStamped,
            'coke_coordinates',
            self.coordinate_callback,
            10)
        self.callback_group = ReentrantCallbackGroup()

        joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'torso_lift_joint'
        ]
        base_link_name = 'base_link'
        end_effector_name = 'gripper_grasping_frame'

        self.moveit2_arm = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name="arm_torso",
            callback_group=self.callback_group
        )

        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        self.create_rate(1.0).sleep()

    def coordinate_callback(self, msg):
        self.get_logger().info(f'Received coordinates: {msg.point.x}, {msg.point.y}, {msg.point.z}')
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.position.x = msg.point.x
        target_pose.pose.position.y = msg.point.y
        target_pose.pose.position.z = msg.point.z

        self.get_logger().info('Moving to new pose...')
        self.moveit2_arm.move_to_pose(
            position=[target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z],
            quat_xyzw=[target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w],
            cartesian=False,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0,
        )

        self.moveit2_arm.wait_until_executed()
        self.get_logger().info('Movement executed')

def main(args=None):
    rclpy.init(args=args)
    arm_move = ArmMove()
    rclpy.spin(arm_move)
    arm_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

