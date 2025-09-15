#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import messagebox
import numpy as np
import time  # needed for delays


class MyCobotGUI(Node):
    def __init__(self):
        super().__init__("mycobot_gui")

        # Publisher
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        # Waypoints
        self.waypoints = []   # Each waypoint: (id, [joint1..joint6, gripper])
        self.current_id = 1

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("NEBULA KNOWLAB")

        self.sliders = []
        slider_names = ['Joint 1', 'Joint 2', 'Joint 3',
                        'Joint 4', 'Joint 5', 'Joint 6', 'Gripper']
        slider_limits = [(-3.14, 3.14)] * 6 + [(-1.14, 0.3)]

        for i, name in enumerate(slider_names):
            frame = tk.Frame(self.root)
            frame.pack(pady=2)

            label = tk.Label(frame, text=name)
            label.pack(side=tk.LEFT)

            slider = tk.Scale(
                frame,
                from_=slider_limits[i][0],
                to=slider_limits[i][1],
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=300,
                command=lambda val, j=i: self.update_joint(j)
            )
            slider.pack(side=tk.LEFT)
            self.sliders.append(slider)

        # Speed slider
        speed_frame = tk.Frame(self.root)
        speed_frame.pack(pady=5)
        tk.Label(speed_frame, text="Speed (steps/sec)").pack(side=tk.LEFT)
        self.speed_slider = tk.Scale(speed_frame, from_=1, to=100,
                                     orient=tk.HORIZONTAL, length=300)
        self.speed_slider.set(30)  # default 30 steps/sec
        self.speed_slider.pack(side=tk.LEFT)

        # Buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=5)
        tk.Button(btn_frame, text="Record Waypoint",
                  command=self.record_waypoint).grid(row=0, column=0, padx=5)
        tk.Button(btn_frame, text="Delete Selected",
                  command=self.delete_selected).grid(row=0, column=1, padx=5)
        tk.Button(btn_frame, text="Play Selected",
                  command=self.play_selected).grid(row=0, column=2, padx=5)
        tk.Button(btn_frame, text="Play All",
                  command=self.play_all).grid(row=0, column=3, padx=5)

        # Waypoints listbox
        self.listbox = tk.Listbox(self.root, width=40, height=10)
        self.listbox.pack(pady=5)

    def update_joint(self, joint_index):
        pos = [slider.get() for slider in self.sliders]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
            'gripper_controller'
        ]
        js.position = pos
        self.pub.publish(js)

    def record_waypoint(self):
        pos = [slider.get() for slider in self.sliders]
        self.waypoints.append((self.current_id, pos))
        self.listbox.insert(tk.END, f"Waypoint {self.current_id}")
        self.current_id += 1

    def delete_selected(self):
        selected = self.listbox.curselection()
        if not selected:
            messagebox.showinfo("Info", "Select a waypoint to delete")
            return
        idx = selected[0]
        self.listbox.delete(idx)
        self.waypoints.pop(idx)

    def play_selected(self):
        selected = self.listbox.curselection()
        if not selected:
            messagebox.showinfo("Info", "Select a waypoint from the list")
            return
        idx = selected[0]
        self._move_to_waypoint(self.waypoints[idx][1])

    def play_all(self):
        speed = self.speed_slider.get()  # steps per second
        delay = 1.0 / speed
        for i in range(len(self.waypoints) - 1):
            wp_current = self.waypoints[i][1]
            wp_next = self.waypoints[i + 1][1]
            self._interpolate_and_publish(wp_current, wp_next, steps=30, delay=delay)

    def _move_to_waypoint(self, pos):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
            'gripper_controller'
        ]
        js.position = pos
        self.pub.publish(js)

        for i, val in enumerate(pos):
            self.sliders[i].set(val)

    def _interpolate_and_publish(self, start, end, steps=30, delay=0.03):
        for step in range(steps):
            interp = [np.linspace(start[j], end[j], steps)[step]
                      for j in range(len(start))]
            self._move_to_waypoint(interp)
            self.root.update()   # keeps GUI responsive
            time.sleep(delay)    # adjust speed


def main():
    rclpy.init()
    node = MyCobotGUI()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()
