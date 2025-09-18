#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class InitPosePublisher(Node):
    def __init__(self):
        super().__init__('init_pose_pub')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def publish_initial_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
            'gripper_controller'
        ]
        msg.position = [0.0] * len(msg.name)

        self.get_logger().info('Publishing initial zero joint state...')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InitPosePublisher()

    # Publish multiple times (e.g. 10 times at 10Hz) then exit
    for _ in range(10):
        node.publish_initial_pose()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
