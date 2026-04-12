#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 AprilTag alignment and approach controller.

What it does:
1. Subscribes to AprilTag detections published as std_msgs/String lines:
     id=<id> pos(m)=(x,y,z) quat(xyzw)=(qx,qy,qz,qw)
2. Extracts the requested tag ID and its pose in the camera frame.
3. Uses the known transform between the robot center and the camera to compute:
   - tag pose in robot frame
   - robot pose in tag frame
4. Rotates the robot until the target tag lies on the robot center line.
5. Drives forward until the robot center is within the requested stop distance.

Expected input:
- The AprilTag detector script from this workspace publishes `~tag_detections`
  strings that include both tag ID and pose.
- A separate UI or command source publishes the desired destination tag ID.

Typical usage:
  rosrun <your_pkg> apriltag_dock_controller.py _tag_topic:=/apriltag_min_pose/tag_detections _config_file:=/path/to/config.yaml

Important assumptions:
- The robot motion is planar.
- The robot forward axis is +X in the robot center frame.
- The provided camera offset is from robot center -> camera frame.
"""

import math
import os
import re

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Int32, String


def quat_xyzw_to_rotmat(qx, qy, qz, qw):
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)

    s = 2.0 / n
    xx = qx * qx * s
    yy = qy * qy * s
    zz = qz * qz * s
    xy = qx * qy * s
    xz = qx * qz * s
    yz = qy * qz * s
    wx = qw * qx * s
    wy = qw * qy * s
    wz = qw * qz * s

    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def rotmat_to_quat_xyzw(rot):
    """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""
    m00, m01, m02 = rot[0, 0], rot[0, 1], rot[0, 2]
    m10, m11, m12 = rot[1, 0], rot[1, 1], rot[1, 2]
    m20, m21, m22 = rot[2, 0], rot[2, 1], rot[2, 2]
    tr = m00 + m11 + m22

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return np.array([qx, qy, qz, qw], dtype=float)


def yaw_to_rotmat(yaw):
    """Build a 3x3 rotation matrix for a planar yaw angle."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def optical_to_body_rotmat():
    """Map a ROS optical frame (x right, y down, z forward) to a body frame (x forward, y left, z up)."""
    return np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])


def make_transform(rot, trans):
    """Build a 4x4 homogeneous transform from rotation and translation."""
    tf = np.eye(4)
    tf[:3, :3] = rot
    tf[:3, 3] = np.asarray(trans, dtype=float).reshape(3)
    return tf


def invert_transform(tf):
    """Invert a 4x4 rigid transform."""
    rot = tf[:3, :3]
    trans = tf[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T.dot(trans)
    return inv


def transform_to_pose_components(tf):
    """Return position and quaternion from a 4x4 transform."""
    pos = tf[:3, 3].copy()
    quat = rotmat_to_quat_xyzw(tf[:3, :3])
    return pos, quat


def load_yaml_config(config_path):
    """Load a YAML config file and return a dictionary."""
    if not config_path:
        return {}

    expanded_path = os.path.expanduser(config_path)
    if not os.path.isfile(expanded_path):
        rospy.logwarn("Config file not found: %s", expanded_path)
        return {}

    try:
        with open(expanded_path, "r") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception as exc:
        rospy.logwarn("Failed to read config file %s: %s", expanded_path, exc)
        return {}

    if not isinstance(data, dict):
        rospy.logwarn("Config file %s does not contain a YAML dictionary", expanded_path)
        return {}

    rospy.loginfo("Loaded config file: %s", expanded_path)
    return data


def get_nested(config, keys, default_value):
    """Fetch a nested config value using a list of keys."""
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default_value
        value = value[key]
    return value


class AprilTagDockController(object):
    """Align and approach a selected AprilTag using a simple reactive state machine."""

    def __init__(self, target_tag_id=None, config_file=None):
        rospy.init_node("apriltag_dock_controller")

        self.config_file = config_file if config_file is not None else rospy.get_param("~config_file", "")
        self.config = load_yaml_config(self.config_file)

        self.tag_topic = rospy.get_param("~tag_topic", "/apriltag_min_pose/tag_detections")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/husky_velocity_controller/cmd_vel")
        self.robot_pose_topic = rospy.get_param("~robot_pose_topic", "~robot_pose_in_tag")
        self.target_tag_topic = rospy.get_param("~target_tag_topic", "/apriltag_navigation/target_tag_id")
        self.at_destination_topic = rospy.get_param("~at_destination_topic", "/apriltag_navigation/at_destination")
        self.target_tag_id = None if target_tag_id is None else int(target_tag_id)
        self.stop_distance = float(rospy.get_param("~stop_distance", 1.5))
        self.align_tolerance_rad = float(rospy.get_param("~align_tolerance_rad", 0.03))
        self.distance_tolerance = float(rospy.get_param("~distance_tolerance", 0.1))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.25))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.8))
        self.angular_gain = float(rospy.get_param("~angular_gain", 1.6))
        self.linear_gain = float(rospy.get_param("~linear_gain", 0.5))
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 10.0))
        self.search_turn_speed = float(rospy.get_param("~search_turn_speed", 0.25))
        self.final_turn_speed = float(rospy.get_param("~final_turn_speed", 0.4))
        self.termination_tag_id = int(rospy.get_param("~termination_tag_id", 5))
        self.camera_frame_convention = str(rospy.get_param("~camera_frame_convention", "optical")).lower()

        # Transform from robot center frame -> camera frame.
        self.robot_to_camera_x = float(get_nested(
            self.config,
            ["robot_body_to_camera", "translation", "x"],
            rospy.get_param("~robot_to_camera_x", 0.0),
        ))
        self.robot_to_camera_y = float(get_nested(
            self.config,
            ["robot_body_to_camera", "translation", "y"],
            rospy.get_param("~robot_to_camera_y", 0.0),
        ))
        self.robot_to_camera_z = float(get_nested(
            self.config,
            ["robot_body_to_camera", "translation", "z"],
            rospy.get_param("~robot_to_camera_z", 0.0),
        ))
        self.robot_to_camera_yaw = float(get_nested(
            self.config,
            ["robot_body_to_camera", "rotation", "yaw"],
            rospy.get_param("~robot_to_camera_yaw", 0.0),
        ))
        self.camera_frame_convention = str(get_nested(
            self.config,
            ["camera", "frame_convention"],
            self.camera_frame_convention,
        )).lower()

        self.tag_detection_re = re.compile(
            r"id=(?P<tag_id>-?\d+)\s+pos\(m\)=\((?P<px>[-+0-9.eE]+),(?P<py>[-+0-9.eE]+),(?P<pz>[-+0-9.eE]+)\)\s+"
            r"quat\(xyzw\)=\((?P<qx>[-+0-9.eE]+),(?P<qy>[-+0-9.eE]+),(?P<qz>[-+0-9.eE]+),(?P<qw>[-+0-9.eE]+)\)"
        )

        self.latest_tag_in_camera = None
        self.latest_robot_in_tag = None
        self.last_detection_time = None
        self.current_state = "IDLE"
        self.destination_reached = False
        self.last_selected_tag_id = None
        self.final_turn_active = False
        self.final_turn_end_time = None

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.robot_pose_pub = rospy.Publisher(self.robot_pose_topic, PoseStamped, queue_size=1)
        self.at_destination_pub = rospy.Publisher(self.at_destination_topic, Bool, queue_size=1, latch=True)
        self.tag_sub = rospy.Subscriber(self.tag_topic, String, self.tag_callback, queue_size=1)
        self.target_tag_sub = rospy.Subscriber(self.target_tag_topic, Int32, self.target_tag_callback, queue_size=1)

        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("AprilTag dock controller started")
        rospy.loginfo("Listening for AprilTag detections on %s", self.tag_topic)
        rospy.loginfo("Waiting for target tag commands on %s", self.target_tag_topic)
        rospy.loginfo("Termination tag id: %d", self.termination_tag_id)
        rospy.loginfo(
            "Robot->camera transform: x=%.3f y=%.3f z=%.3f yaw=%.3f",
            self.robot_to_camera_x,
            self.robot_to_camera_y,
            self.robot_to_camera_z,
            self.robot_to_camera_yaw,
        )
        rospy.loginfo("Camera frame convention: %s", self.camera_frame_convention)
        self.publish_destination_status(False)

    def tag_callback(self, msg):
        """Parse the detector string message and store the selected tag pose."""
        if self.target_tag_id is None:
            return

        detected_tag_ids = self.extract_detected_tag_ids(msg.data)
        if (
            self.termination_tag_id in detected_tag_ids
            and self.is_alignment_state()
        ):
            self.complete_destination(
                "Termination tag %d observed during alignment" % self.termination_tag_id
            )
            return

        selected_detection = self.extract_target_tag_pose(msg.data, self.target_tag_id)
        if selected_detection is None:
            return

        self.latest_tag_in_camera = selected_detection
        self.last_detection_time = rospy.Time.now()
        self.latest_robot_in_tag = self.compute_robot_pose_in_tag(selected_detection)
        self.publish_robot_pose(self.latest_robot_in_tag)

    def target_tag_callback(self, msg):
        """Handle destination selection updates from the GUI."""
        new_target_tag_id = int(msg.data)
        if (
            self.target_tag_id == new_target_tag_id
            and not self.destination_reached
            and self.current_state != "IDLE"
        ):
            return

        self.target_tag_id = new_target_tag_id
        self.last_selected_tag_id = new_target_tag_id
        self.latest_tag_in_camera = None
        self.latest_robot_in_tag = None
        self.last_detection_time = None
        self.destination_reached = False
        self.final_turn_active = False
        self.final_turn_end_time = None
        self.current_state = "SEARCH"
        self.publish_destination_status(False)
        rospy.loginfo("New destination selected: AprilTag id=%d", self.target_tag_id)

    def start_final_turn(self):
        """Begin the 180-degree turnaround that runs after the robot reaches the target."""
        turn_speed = max(abs(self.final_turn_speed), 1e-3)
        turn_duration = math.pi / turn_speed
        self.final_turn_active = True
        self.final_turn_end_time = rospy.Time.now() + rospy.Duration.from_sec(turn_duration)
        self.set_state("FINAL_TURN")
        rospy.loginfo(
            "Target distance reached for tag %d. Starting final 180-degree turn at %.3f rad/s",
            self.target_tag_id,
            self.final_turn_speed,
        )

    def extract_detected_tag_ids(self, message_text):
        """Return the set of tag IDs present in the detector message."""
        detected_ids = set()
        for line in message_text.splitlines():
            match = self.tag_detection_re.search(line.strip())
            if match:
                detected_ids.add(int(match.group("tag_id")))
        return detected_ids

    def extract_target_tag_pose(self, message_text, target_id):
        """Find the target tag line and return its transform camera -> tag."""
        for line in message_text.splitlines():
            match = self.tag_detection_re.search(line.strip())
            if not match:
                continue

            tag_id = int(match.group("tag_id"))
            if tag_id != target_id:
                continue

            pos = np.array([
                float(match.group("px")),
                float(match.group("py")),
                float(match.group("pz")),
            ], dtype=float)
            quat = np.array([
                float(match.group("qx")),
                float(match.group("qy")),
                float(match.group("qz")),
                float(match.group("qw")),
            ], dtype=float)

            rot = quat_xyzw_to_rotmat(quat[0], quat[1], quat[2], quat[3])
            return make_transform(rot, pos)

        return None

    def get_robot_to_camera_transform(self):
        """Return the configured fixed transform from robot center to camera."""
        yaw_rot = yaw_to_rotmat(self.robot_to_camera_yaw)

        if self.camera_frame_convention == "optical":
            rot = yaw_rot.dot(optical_to_body_rotmat())
        elif self.camera_frame_convention == "body":
            rot = yaw_rot
        else:
            rospy.logwarn_throttle(
                5.0,
                "Unknown camera_frame_convention='%s'. Falling back to 'optical'.",
                self.camera_frame_convention,
            )
            rot = yaw_rot.dot(optical_to_body_rotmat())

        trans = np.array([
            self.robot_to_camera_x,
            self.robot_to_camera_y,
            self.robot_to_camera_z,
        ], dtype=float)
        return make_transform(rot, trans)

    def compute_tag_pose_in_robot(self, camera_to_tag):
        """Convert the observed tag pose from camera frame into robot frame."""
        robot_to_camera = self.get_robot_to_camera_transform()
        return robot_to_camera.dot(camera_to_tag)

    def compute_robot_pose_in_tag(self, camera_to_tag):
        """Compute robot pose expressed in the tag frame."""
        robot_to_tag = self.compute_tag_pose_in_robot(camera_to_tag)
        tag_to_robot = invert_transform(robot_to_tag)
        return tag_to_robot

    def publish_robot_pose(self, robot_in_tag):
        """Publish the robot pose in the target tag frame."""
        if robot_in_tag is None or self.target_tag_id is None:
            return

        pos, quat = transform_to_pose_components(robot_in_tag)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "tag_{}".format(self.target_tag_id)
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.robot_pose_pub.publish(pose_msg)

    def publish_destination_status(self, is_at_destination):
        """Publish whether the robot has reached the selected destination."""
        self.at_destination_pub.publish(Bool(data=bool(is_at_destination)))

    def get_alignment_error(self):
        """Return the heading error to the tag center in the robot frame."""
        if self.latest_tag_in_camera is None:
            return None

        robot_to_tag = self.compute_tag_pose_in_robot(self.latest_tag_in_camera)
        tag_pos_in_robot = robot_to_tag[:3, 3]
        return math.atan2(tag_pos_in_robot[1], tag_pos_in_robot[0])

    def get_distance_to_tag(self):
        """Return planar distance from robot center to tag center."""
        if self.latest_tag_in_camera is None:
            return None

        robot_to_tag = self.compute_tag_pose_in_robot(self.latest_tag_in_camera)
        tag_pos_in_robot = robot_to_tag[:3, 3]
        return math.hypot(tag_pos_in_robot[0], tag_pos_in_robot[1])

    def clamp(self, value, min_value, max_value):
        """Clamp a scalar to a closed interval."""
        return max(min_value, min(max_value, value))

    def send_velocity(self, linear_x, angular_z):
        """Publish a base velocity command."""
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot safely."""
        self.send_velocity(0.0, 0.0)

    def tag_recently_seen(self):
        """Check whether the target tag was detected recently enough to trust control."""
        if self.last_detection_time is None:
            return False
        age = (rospy.Time.now() - self.last_detection_time).to_sec()
        return age < 1.0

    def set_state(self, new_state):
        """Update the high-level controller state and log transitions."""
        if self.current_state != new_state:
            rospy.loginfo("Controller state: %s -> %s", self.current_state, new_state)
            self.current_state = new_state

    def is_alignment_state(self):
        """Return True when the robot is performing a pure rotation/alignment maneuver."""
        return self.current_state in ("ALIGN", "FINAL_TURN")

    def complete_destination(self, reason):
        """Finish the current destination and stop the robot."""
        self.final_turn_active = False
        self.final_turn_end_time = None
        self.destination_reached = True
        self.set_state("DONE")
        self.publish_destination_status(True)
        self.stop_robot()
        rospy.loginfo("%s", reason)

    def run_control_step(self):
        """Execute one control cycle based on the active destination and current observations."""
        if self.target_tag_id is None:
            self.set_state("IDLE")
            self.destination_reached = False
            self.publish_destination_status(False)
            self.stop_robot()
            return

        if self.destination_reached:
            self.set_state("DONE")
            self.publish_destination_status(True)
            self.stop_robot()
            return

        if self.final_turn_active:
            if rospy.Time.now() >= self.final_turn_end_time:
                self.complete_destination(
                    "Final 180-degree turn complete for tag %d" % self.target_tag_id
                )
                return

            self.set_state("FINAL_TURN")
            self.publish_destination_status(False)
            self.send_velocity(0.0, self.final_turn_speed)
            return

        if not self.tag_recently_seen():
            self.set_state("SEARCH")
            self.publish_destination_status(False)
            self.send_velocity(0.0, self.search_turn_speed)
            return

        heading_error = self.get_alignment_error()
        distance = self.get_distance_to_tag()
        if heading_error is None or distance is None:
            self.set_state("SEARCH")
            self.publish_destination_status(False)
            self.send_velocity(0.0, self.search_turn_speed)
            return

        distance_error = distance - self.stop_distance
        if distance_error <= self.distance_tolerance:
            self.publish_destination_status(False)
            self.stop_robot()
            self.start_final_turn()
            return

        self.publish_destination_status(False)

        if abs(heading_error) > self.align_tolerance_rad:
            self.set_state("ALIGN")
            angular_cmd = self.clamp(
                self.angular_gain * heading_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.send_velocity(0.0, angular_cmd)
            return

        self.set_state("APPROACH")
        angular_cmd = self.clamp(
            self.angular_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        linear_cmd = self.clamp(
            self.linear_gain * distance_error,
            0.0,
            self.max_linear_speed,
        )

        if abs(heading_error) > 0.15:
            linear_cmd = 0.0

        self.send_velocity(linear_cmd, angular_cmd)

    def run(self):
        """Main control loop that reacts to destination selections from the GUI."""
        rate = rospy.Rate(self.control_rate_hz)
        while not rospy.is_shutdown():
            self.run_control_step()
            rate.sleep()


def main():
    """Node entry point."""
    controller = AprilTagDockController()
    controller.run()


if __name__ == "__main__":
    main()
