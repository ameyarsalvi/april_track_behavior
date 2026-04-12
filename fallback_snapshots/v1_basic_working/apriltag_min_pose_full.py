#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal AprilTag 6-DoF pose (ROS1) using pupil_apriltags.

Publishes:
  - ~poses           (geometry_msgs/PoseArray)  # camera -> tag poses (position + orientation)
  - ~tag_detections  (std_msgs/String)          # lines with: id, position (m), quaternion (xyzw)
  - ~image           (sensor_msgs/Image)        # debug overlay (optional)

Params (~):
  config_file: ""                      # optional shared YAML config
  image_topic: "/usb_cam/image_raw"    # sensor_msgs/Image
  family: "tag36h11"
  tag_size: 0.162                      # meters (outer black square)
  camera_frame: "camera_link"
  fx, fy, cx, cy:                      # fallback intrinsics (pixels)
  publish_debug_image: true
  min_decision_margin: 0.0             # set e.g. 25.0 to filter weak detections
"""

import os

import rospy
import numpy as np
import cv2
import yaml
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header, String
from cv_bridge import CvBridge

try:
    import pupil_apriltags as apriltag
except ImportError:
    raise RuntimeError("Missing pupil_apriltags. Install with: pip3 install pupil-apriltags")


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

def rotmat_to_quat_xyzw(R):
    """Convert 3x3 rotation matrix to quaternion (x,y,z,w)."""
    m00, m01, m02 = R[0,0], R[0,1], R[0,2]
    m10, m11, m12 = R[1,0], R[1,1], R[1,2]
    m20, m21, m22 = R[2,0], R[2,1], R[2,2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    return float(x), float(y), float(z), float(w)

class Node:
    def __init__(self):
        rospy.init_node("apriltag_min_pose")

        self.config_file = rospy.get_param("~config_file", "")
        self.config = load_yaml_config(self.config_file)

        # Params
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.family      = rospy.get_param("~family", "tag36h11")
        self.tag_size    = float(rospy.get_param("~tag_size", 0.450))  # meters
        self.frame_id    = str(get_nested(
            self.config,
            ["camera", "frame_id"],
            rospy.get_param("~camera_frame", "camera_link"),
        ))
        self.fx          = float(get_nested(
            self.config,
            ["camera", "intrinsics", "fx"],
            rospy.get_param("~fx", 615.0),
        ))
        self.fy          = float(get_nested(
            self.config,
            ["camera", "intrinsics", "fy"],
            rospy.get_param("~fy", 615.0),
        ))
        self.cx          = float(get_nested(
            self.config,
            ["camera", "intrinsics", "cx"],
            rospy.get_param("~cx", 320.0),
        ))
        self.cy          = float(get_nested(
            self.config,
            ["camera", "intrinsics", "cy"],
            rospy.get_param("~cy", 240.0),
        ))
        self.pub_debug   = bool(rospy.get_param("~publish_debug_image", True))
        self.min_margin  = float(rospy.get_param("~min_decision_margin", 0.0))

        # Detector (single-thread for stability)
        self.detector = apriltag.Detector(
            families=self.family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.bridge    = CvBridge()
        self.pose_pub  = rospy.Publisher("~poses", PoseArray, queue_size=1)
        self.text_pub  = rospy.Publisher("~tag_detections", String, queue_size=1)
        self.image_pub = rospy.Publisher("~image", Image, queue_size=1)

        rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)

        rospy.loginfo("apriltag_min_pose running | topic=%s | family=%s | tag=%.3fm",
                      self.image_topic, self.family, self.tag_size)
        rospy.loginfo("Camera frame: %s", self.frame_id)
        rospy.logwarn("Using configured intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                      self.fx, self.fy, self.cx, self.cy)

    def cb(self, msg):
        # Decode to BGR + gray
        try:
            if msg.encoding in ("bgr8", "rgb8"):
                bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg.encoding in ("mono8", "8UC1"):
                gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
                bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = np.ascontiguousarray(gray, dtype=np.uint8)
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)
            return

        # Detect with pose
        try:
            dets = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=self.tag_size
            )
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Detector error: %s", e)
            return

        # Prepare outputs
        pa = PoseArray()
        pa.header = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
        text_lines = []
        draw_overlay = self.pub_debug and self.image_pub.get_num_connections() > 0
        if draw_overlay:
            canvas = bgr.copy()

        for d in dets:
            if self.min_margin > 0.0 and hasattr(d, "decision_margin"):
                if d.decision_margin < self.min_margin:
                    continue

            # pupil_apriltags returns the pose from camera optical frame -> tag frame.
            # Keep that convention here so the downstream controller can map the
            # optical frame into the robot body frame correctly.
            R_ct = d.pose_R.astype(float)
            t_ct = d.pose_t.reshape(3).astype(float)
            qx, qy, qz, qw = rotmat_to_quat_xyzw(R_ct)

            # Pose (position + orientation)
            p = Pose()
            p.position.x = float(t_ct[0])
            p.position.y = float(t_ct[1])
            p.position.z = float(t_ct[2])
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            pa.poses.append(p)

            # Human-readable line with id + 6DoF
            text_lines.append(
                "id={} pos(m)=({:.3f},{:.3f},{:.3f}) quat(xyzw)=({:.3f},{:.3f},{:.3f},{:.3f})".format(
                    int(d.tag_id), p.position.x, p.position.y, p.position.z, qx, qy, qz, qw
                )
            )

            # Draw overlay
            if draw_overlay:
                corners = d.corners.astype(np.int32)
                center  = d.center.astype(np.int32)
                cv2.polylines(canvas, [corners.reshape(-1,1,2)], True, (0,255,0), 2)
                cv2.circle(canvas, tuple(center), 4, (0,0,255), -1)
                cv2.putText(canvas, "ID:{}".format(int(d.tag_id)),
                            (center[0] + 8, center[1] + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Publish
        self.pose_pub.publish(pa)
        if text_lines:
            self.text_pub.publish(String(data="\n".join(text_lines)))
            # Lightweight log
            rospy.loginfo_throttle(1.0, text_lines[0])

        if draw_overlay:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
            except Exception as e:
                rospy.logwarn_throttle(2.0, "debug image publish error: %s", e)

def main():
    Node()
    rospy.spin()

if __name__ == "__main__":
    main()
