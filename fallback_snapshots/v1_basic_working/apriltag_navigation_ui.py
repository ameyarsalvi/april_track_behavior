#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple ROS1 GUI for selecting an AprilTag destination.

Buttons:
- Parking   -> tag id 0
- Inventory -> tag id 1
- Assembly  -> tag id 3

The GUI publishes the selected tag ID and listens for an "at destination"
status signal from the docking controller. It can also launch the detector
and controller nodes for a one-command workflow.
"""

import os
import subprocess
import sys
import tkinter as tk

import rospy
from std_msgs.msg import Bool, Int32


DESTINATIONS = {
    "Parking": 3,
    "Inventory": 1,
    "Assembly": 0,
}


class AprilTagNavigationUI(object):
    """Small Tkinter front end for destination selection."""

    def __init__(self):
        rospy.init_node("apriltag_navigation_ui", disable_signals=True)

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.target_tag_topic = rospy.get_param("~target_tag_topic", "/apriltag_navigation/target_tag_id")
        self.at_destination_topic = rospy.get_param("~at_destination_topic", "/apriltag_navigation/at_destination")
        self.window_title = rospy.get_param("~window_title", "AprilTag Navigation")
        self.auto_launch_nodes = bool(rospy.get_param("~auto_launch_nodes", True))
        self.config_file = rospy.get_param(
            "~config_file",
            os.path.join(self.script_dir, "apriltag_system_config.yaml"),
        )
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.detector_family = rospy.get_param("~family", "tag36h11")
        self.detector_tag_size = float(rospy.get_param("~tag_size", 0.450))

        self.selected_name = None
        self.at_destination = False
        self.detector_process = None
        self.controller_process = None

        self.target_tag_pub = rospy.Publisher(self.target_tag_topic, Int32, queue_size=1, latch=True)
        self.status_sub = rospy.Subscriber(self.at_destination_topic, Bool, self.status_callback, queue_size=1)

        if self.auto_launch_nodes:
            self.launch_backend_nodes()

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry("420x280")
        self.root.configure(bg="#f4efe8")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.title_label = tk.Label(
            self.root,
            text="Choose Destination",
            font=("Helvetica", 18, "bold"),
            bg="#f4efe8",
            fg="#2c2c2c",
        )
        self.title_label.pack(pady=(18, 10))

        self.subtitle_label = tk.Label(
            self.root,
            text="Select where the robot should go.",
            font=("Helvetica", 11),
            bg="#f4efe8",
            fg="#555555",
        )
        self.subtitle_label.pack(pady=(0, 18))

        self.button_frame = tk.Frame(self.root, bg="#f4efe8")
        self.button_frame.pack(fill="x", padx=30)

        self.buttons = {}
        for name in DESTINATIONS:
            button = tk.Button(
                self.button_frame,
                text=name,
                font=("Helvetica", 14, "bold"),
                bg="#d8dee9",
                fg="#1f2933",
                activebackground="#b7c5d3",
                activeforeground="#1f2933",
                relief="flat",
                padx=16,
                pady=12,
                command=lambda destination_name=name: self.select_destination(destination_name),
            )
            button.pack(fill="x", pady=6)
            self.buttons[name] = button

        self.selected_label = tk.Label(
            self.root,
            text="Selected: None",
            font=("Helvetica", 12),
            bg="#f4efe8",
            fg="#2c2c2c",
        )
        self.selected_label.pack(pady=(18, 8))

        self.destination_status_label = tk.Label(
            self.root,
            text="At Destination",
            font=("Helvetica", 14, "bold"),
            bg="#b23a48",
            fg="white",
            width=18,
            padx=8,
            pady=8,
        )
        self.destination_status_label.pack(pady=(6, 10))

        self.root.after(100, self.spin_once)

    def launch_backend_nodes(self):
        """Start the AprilTag detector and controller as child processes."""
        detector_cmd = [
            sys.executable,
            os.path.join(self.script_dir, "apriltag_min_pose_full.py"),
            "_config_file:={}".format(self.config_file),
            "_image_topic:={}".format(self.image_topic),
            "_family:={}".format(self.detector_family),
            "_tag_size:={}".format(self.detector_tag_size),
        ]
        controller_cmd = [
            sys.executable,
            os.path.join(self.script_dir, "apriltag_dock_controller.py"),
            "_config_file:={}".format(self.config_file),
        ]

        self.detector_process = subprocess.Popen(detector_cmd, cwd=self.script_dir)
        self.controller_process = subprocess.Popen(controller_cmd, cwd=self.script_dir)
        rospy.loginfo("Launched detector and controller from the UI")

    def select_destination(self, name):
        """Publish the selected destination tag ID and refresh the UI."""
        self.selected_name = name
        self.at_destination = False
        self.target_tag_pub.publish(Int32(data=DESTINATIONS[name]))
        self.refresh_ui()

    def status_callback(self, msg):
        """Update destination status from the controller."""
        self.at_destination = bool(msg.data)
        self.refresh_ui()

    def refresh_ui(self):
        """Update button styles and the destination status display."""
        selected_text = "Selected: None"
        if self.selected_name is not None:
            selected_text = "Selected: {} (Tag {})".format(
                self.selected_name,
                DESTINATIONS[self.selected_name],
            )
        self.selected_label.config(text=selected_text)

        for name, button in self.buttons.items():
            if name == self.selected_name:
                button.config(bg="#c08457", fg="white", activebackground="#a96c41", activeforeground="white")
            else:
                button.config(bg="#d8dee9", fg="#1f2933", activebackground="#b7c5d3", activeforeground="#1f2933")

        if self.at_destination and self.selected_name is not None:
            self.destination_status_label.config(bg="#2f855a", text="At Destination")
        else:
            self.destination_status_label.config(bg="#b23a48", text="At Destination")

    def spin_once(self):
        """Keep rospy callbacks flowing while Tkinter owns the main loop."""
        if rospy.is_shutdown():
            self.root.destroy()
            return

        self.root.after(100, self.spin_once)

    def on_close(self):
        """Close the Tkinter window cleanly."""
        self.shutdown_backend_nodes()
        self.root.destroy()

    def run(self):
        """Start the GUI event loop."""
        self.refresh_ui()
        self.root.mainloop()

    def shutdown_backend_nodes(self):
        """Stop child processes started by the UI."""
        for process in (self.controller_process, self.detector_process):
            if process is None:
                continue
            if process.poll() is None:
                process.terminate()

        for process in (self.controller_process, self.detector_process):
            if process is None:
                continue
            if process.poll() is None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()


def main():
    """Node entry point."""
    app = AprilTagNavigationUI()
    app.run()


if __name__ == "__main__":
    main()
