#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class DancingRobot(Node):
    def __init__(self):
        super().__init__("dancing_robot")

        # Publisher
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        # Joint names (same as your GUI / URDF)
        self.names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
            "gripper_controller"
        ]

        # Dance positions (all radians, 7 joints each)
        self.positions_list = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # neutral

        ]

        # Dance speed
        self.steps = 50    # interpolation steps
        self.delay = 0.03  # seconds per step

    def start_dancing(self):
        current = self.positions_list[0]
        while rclpy.ok():
            for next_pose in self.positions_list[1:]:
                self._interpolate_and_publish(current, next_pose,
                                              steps=self.steps,
                                              delay=self.delay)
                current = next_pose
            # loop back to start smoothly
            self._interpolate_and_publish(current,
                                          self.positions_list[0],
                                          steps=self.steps,
                                          delay=self.delay)
            current = self.positions_list[0]

    def _interpolate_and_publish(self, start, end, steps=50, delay=0.03):
        for step in range(steps):
            interp = [
                start[j] + (end[j] - start[j]) * step / (steps - 1)
                for j in range(len(start))
            ]
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.names
            js.position = interp
            self.pub.publish(js)
            time.sleep(delay)


def main():
    rclpy.init()
    node = DancingRobot()
    try:
        node.start_dancing()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
