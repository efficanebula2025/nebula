#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import messagebox, simpledialog
import numpy as np
import time

class MyCobotGUI(Node):
    def __init__(self):
        super().__init__("mycobot_gui")

        # Publisher
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        # Waypoints with custom names
        self.waypoints = []   # Each waypoint: (id, name, [joint1..joint6, gripper])
        self.current_id = 1
        self.drag_data = {"item": None, "y": 0}
        
        # Flags to prevent update loops
        self.updating_from_slider = [False] * 7
        self.updating_from_entry = [False] * 7

        # Color scheme
        self.colors = {
            'bg': '#2c3e50',
            'frame_bg': '#34495e',
            'button_bg': '#3498db',
            'button_hover': '#2980b9',
            'slider_bg': '#ecf0f1',
            'text': '#ecf0f1',
            'accent': '#e74c3c',
            'success': '#27ae60'
        }

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("NEBULA KNOWLAB - Enhanced")
        self.root.configure(bg=self.colors['bg'])
        self.root.geometry("800x700")

        self.setup_gui()

    def setup_gui(self):
        # Main container
        main_frame = tk.Frame(self.root, bg=self.colors['bg'])
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Joint controls section
        joint_frame = tk.LabelFrame(main_frame, text="Joint Controls", 
                                   bg=self.colors['frame_bg'], fg=self.colors['text'],
                                   font=('Arial', 12, 'bold'))
        joint_frame.pack(fill=tk.X, pady=(0, 10))

        self.sliders = []
        self.value_entries = []
        slider_names = ['Joint 1', 'Joint 2', 'Joint 3',
                        'Joint 4', 'Joint 5', 'Joint 6', 'Gripper']
        slider_limits = [(-3.14, 3.14)] * 6 + [(-1.14, 0.3)]

        for i, name in enumerate(slider_names):
            frame = tk.Frame(joint_frame, bg=self.colors['frame_bg'])
            frame.pack(pady=3, padx=10, fill=tk.X)

            # Label
            label = tk.Label(frame, text=f"{name}:", width=10,
                           bg=self.colors['frame_bg'], fg=self.colors['text'],
                           font=('Arial', 10))
            label.pack(side=tk.LEFT)

            # Value entry box
            entry = tk.Entry(frame, width=8, font=('Arial', 10), 
                           justify='center', bg='white', fg='black')
            entry.pack(side=tk.LEFT, padx=(5, 10))
            entry.insert(0, "0.00")
            entry.bind('<KeyRelease>', lambda event, j=i: self.update_from_entry(j))
            self.value_entries.append(entry)

            # Slider
            slider = tk.Scale(
                frame,
                from_=slider_limits[i][0],
                to=slider_limits[i][1],
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=400,
                bg=self.colors['slider_bg'],
                troughcolor=self.colors['button_bg'],
                command=lambda val, j=i: self.slider_changed(j, val)
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sliders.append(slider)

        # Speed control
        speed_frame = tk.LabelFrame(main_frame, text="Speed Control",
                                   bg=self.colors['frame_bg'], fg=self.colors['text'],
                                   font=('Arial', 12, 'bold'))
        speed_frame.pack(fill=tk.X, pady=(0, 10))

        speed_inner = tk.Frame(speed_frame, bg=self.colors['frame_bg'])
        speed_inner.pack(pady=5, padx=10, fill=tk.X)

        tk.Label(speed_inner, text="Speed (steps/sec):", 
                bg=self.colors['frame_bg'], fg=self.colors['text'],
                font=('Arial', 10)).pack(side=tk.LEFT)
        
        self.speed_slider = tk.Scale(speed_inner, from_=1, to=100,
                                   orient=tk.HORIZONTAL, length=300,
                                   bg=self.colors['slider_bg'],
                                   troughcolor=self.colors['success'])
        self.speed_slider.set(30)
        self.speed_slider.pack(side=tk.LEFT, padx=10)

        # Control buttons
        btn_frame = tk.LabelFrame(main_frame, text="Controls",
                                 bg=self.colors['frame_bg'], fg=self.colors['text'],
                                 font=('Arial', 12, 'bold'))
        btn_frame.pack(fill=tk.X, pady=(0, 10))

        btn_inner = tk.Frame(btn_frame, bg=self.colors['frame_bg'])
        btn_inner.pack(pady=5)

        buttons = [
            ("Record Waypoint", self.record_waypoint, self.colors['success']),
            ("Delete Selected", self.delete_selected, self.colors['accent']),
            ("Play Selected", self.play_selected, self.colors['button_bg']),
            ("Play All", self.play_all, self.colors['button_bg'])
        ]

        for i, (text, command, color) in enumerate(buttons):
            btn = tk.Button(btn_inner, text=text, command=command,
                          bg=color, fg='white', font=('Arial', 10, 'bold'),
                          relief=tk.FLAT, padx=15, pady=5)
            btn.grid(row=0, column=i, padx=5)
            self.bind_button_hover(btn, color)

        # Waypoints section
        waypoint_frame = tk.LabelFrame(main_frame, text="Waypoints (Double-click to rename, Drag to reorder)",
                                      bg=self.colors['frame_bg'], fg=self.colors['text'],
                                      font=('Arial', 12, 'bold'))
        waypoint_frame.pack(fill=tk.BOTH, expand=True)

        # Listbox with scrollbar
        listbox_frame = tk.Frame(waypoint_frame, bg=self.colors['frame_bg'])
        listbox_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        scrollbar = tk.Scrollbar(listbox_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.listbox = tk.Listbox(listbox_frame, width=50, height=8,
                                 yscrollcommand=scrollbar.set,
                                 font=('Arial', 11),
                                 selectbackground=self.colors['button_bg'])
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.listbox.yview)

        # Bind events for drag and drop and double-click
        self.listbox.bind('<Double-Button-1>', self.rename_waypoint)
        self.listbox.bind('<Button-1>', self.on_drag_start)
        self.listbox.bind('<B1-Motion>', self.on_drag_motion)
        self.listbox.bind('<ButtonRelease-1>', self.on_drag_release)

    def bind_button_hover(self, button, original_color):
        """Add hover effect to buttons"""
        def on_enter(e):
            button.config(bg=self.colors['button_hover'])
        def on_leave(e):
            button.config(bg=original_color)
        
        button.bind('<Enter>', on_enter)
        button.bind('<Leave>', on_leave)

    def slider_changed(self, joint_index, value):
        """Called when slider value changes"""
        if self.updating_from_entry[joint_index]:
            return
            
        self.updating_from_slider[joint_index] = True
        # Update the corresponding entry box
        self.value_entries[joint_index].delete(0, tk.END)
        self.value_entries[joint_index].insert(0, f"{float(value):.2f}")
        # Update joint position
        self.update_joint()
        self.updating_from_slider[joint_index] = False

    def update_from_entry(self, joint_index):
        """Called when entry box value changes (KeyRelease event)"""
        try:
            value = float(self.value_entries[joint_index].get())
        except ValueError:
            return
            
        slider = self.sliders[joint_index]
        # Clamp value to slider limits
        min_val = slider['from']
        max_val = slider['to']
        value = max(min_val, min(max_val, value))
        
        # Update slider - this will trigger slider_changed
        self.updating_from_entry[joint_index] = True
        slider.set(value)
        self.updating_from_entry[joint_index] = False

    def update_joint(self, joint_index=None):
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
        name = f"Waypoint {self.current_id}"
        self.waypoints.append((self.current_id, name, pos))
        self.listbox.insert(tk.END, name)
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
        self._move_to_waypoint(self.waypoints[idx][2])  # position is at index 2

    def play_all(self):
        if len(self.waypoints) < 2:
            messagebox.showinfo("Info", "Need at least 2 waypoints to play all")
            return
            
        speed = self.speed_slider.get()
        delay = 1.0 / speed
        
        for i in range(len(self.waypoints) - 1):
            wp_current = self.waypoints[i][2]  # position is at index 2
            wp_next = self.waypoints[i + 1][2]
            self._interpolate_and_publish(wp_current, wp_next, steps=30, delay=delay)

    def rename_waypoint(self, event):
        """Handle double-click to rename waypoint"""
        selected = self.listbox.curselection()
        if not selected:
            return
        
        idx = selected[0]
        current_name = self.waypoints[idx][1]
        
        new_name = simpledialog.askstring("Rename Waypoint", 
                                         "Enter new name:", 
                                         initialvalue=current_name)
        
        if new_name and new_name.strip():
            self.waypoints[idx] = (self.waypoints[idx][0], new_name.strip(), self.waypoints[idx][2])
            self.listbox.delete(idx)
            self.listbox.insert(idx, new_name.strip())
            self.listbox.selection_set(idx)

    def on_drag_start(self, event):
        """Start drag operation"""
        self.drag_data["item"] = self.listbox.nearest(event.y)
        self.drag_data["y"] = event.y

    def on_drag_motion(self, event):
        """Handle drag motion"""
        if self.drag_data["item"] is not None:
            # Visual feedback could be added here
            pass

    def on_drag_release(self, event):
        """Handle drag release - reorder items"""
        if self.drag_data["item"] is not None:
            target = self.listbox.nearest(event.y)
            source = self.drag_data["item"]
            
            if source != target and 0 <= target < len(self.waypoints):
                # Move waypoint in the list
                waypoint = self.waypoints.pop(source)
                self.waypoints.insert(target, waypoint)
                
                # Update listbox
                self.refresh_listbox()
                self.listbox.selection_set(target)
        
        self.drag_data = {"item": None, "y": 0}

    def refresh_listbox(self):
        """Refresh the listbox display"""
        self.listbox.delete(0, tk.END)
        for _, name, _ in self.waypoints:
            self.listbox.insert(tk.END, name)

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

        # Update sliders and entry boxes
        for i, val in enumerate(pos):
            self.sliders[i].set(val)
            self.value_entries[i].delete(0, tk.END)
            self.value_entries[i].insert(0, f"{val:.2f}")

    def _interpolate_and_publish(self, start, end, steps=30, delay=0.03):
        for step in range(steps):
            interp = [np.linspace(start[j], end[j], steps)[step]
                      for j in range(len(start))]
            self._move_to_waypoint(interp)
            self.root.update()
            time.sleep(delay)


def main():
    rclpy.init()
    node = MyCobotGUI()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
