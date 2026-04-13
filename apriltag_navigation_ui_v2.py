#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag navigation UI v2 with optional voice-based station selection.

This keeps the existing button-driven workflow and adds a "Listen for Station"
action that records a short microphone sample, transcribes it, and maps the
spoken phrase to one of the known destinations.

Notes:
- The published PyPI package named "whispr" is not a speech recognizer.
- This UI therefore uses a configurable transcription backend module and
  defaults to the commonly used `whisper` Python API shape:
      model = whisper.load_model("base")
      result = model.transcribe("/path/to/audio.wav")
- If you have a different local package that exposes a compatible API, you can
  point the node parameter `~voice_backend_module` at it.
"""

import importlib
import os
import subprocess
import sys
import tempfile
import threading
import tkinter as tk

import rospy
from std_msgs.msg import Bool, Int32


DESTINATIONS = {
    "Parking": 3,
    "Inventory": 1,
    "Assembly": 0,
}

DESTINATION_ALIASES = {
    "Parking": ("parking", "dock", "station 3", "tag 3"),
    "Inventory": ("inventory", "stock", "storage", "station 1", "tag 1"),
    "Assembly": ("assembly", "build", "manufacturing", "station 0", "tag 0"),
}


class AprilTagNavigationUIV2(object):
    """Tkinter front end for destination selection with optional voice input."""

    def __init__(self):
        rospy.init_node("apriltag_navigation_ui_v2", disable_signals=True)

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.target_tag_topic = rospy.get_param("~target_tag_topic", "/apriltag_navigation/target_tag_id")
        self.at_destination_topic = rospy.get_param("~at_destination_topic", "/apriltag_navigation/at_destination")
        self.window_title = rospy.get_param("~window_title", "AprilTag Navigation v2")
        self.auto_launch_nodes = bool(rospy.get_param("~auto_launch_nodes", True))
        self.config_file = rospy.get_param(
            "~config_file",
            os.path.join(self.script_dir, "apriltag_system_config.yaml"),
        )
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.detector_family = rospy.get_param("~family", "tag36h11")
        self.detector_tag_size = float(rospy.get_param("~tag_size", 0.450))

        self.voice_enabled = bool(rospy.get_param("~voice_enabled", True))
        self.voice_backend_module = rospy.get_param("~voice_backend_module", "whisper")
        self.voice_model_name = rospy.get_param("~voice_model", "base")
        self.voice_record_seconds = int(rospy.get_param("~voice_record_seconds", 4))
        self.voice_sample_rate = int(rospy.get_param("~voice_sample_rate", 16000))
        self.voice_language = rospy.get_param("~voice_language", "en")
        self.arecord_path = rospy.get_param("~arecord_path", "arecord")

        self.selected_name = None
        self.at_destination = False
        self.detector_process = None
        self.controller_process = None
        self.voice_model = None
        self.voice_busy = False
        self.voice_status_text = "Voice: ready"
        self.last_transcript = ""

        self.target_tag_pub = rospy.Publisher(self.target_tag_topic, Int32, queue_size=1, latch=True)
        self.status_sub = rospy.Subscriber(self.at_destination_topic, Bool, self.status_callback, queue_size=1)

        if self.auto_launch_nodes:
            self.launch_backend_nodes()

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry("460x420")
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
            text="Use the buttons or select a station by voice.",
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

        self.voice_button = tk.Button(
            self.root,
            text="Listen for Station",
            font=("Helvetica", 13, "bold"),
            bg="#7c9a92",
            fg="white",
            activebackground="#68857d",
            activeforeground="white",
            relief="flat",
            padx=16,
            pady=12,
            command=self.on_voice_button_pressed,
        )
        self.voice_button.pack(fill="x", padx=30, pady=(14, 8))

        self.voice_status_label = tk.Label(
            self.root,
            text=self.voice_status_text,
            font=("Helvetica", 11),
            bg="#f4efe8",
            fg="#2c2c2c",
            wraplength=390,
            justify="center",
        )
        self.voice_status_label.pack(pady=(2, 6))

        self.transcript_label = tk.Label(
            self.root,
            text="Heard: None",
            font=("Helvetica", 10),
            bg="#f4efe8",
            fg="#666666",
            wraplength=390,
            justify="center",
        )
        self.transcript_label.pack(pady=(0, 12))

        self.selected_label = tk.Label(
            self.root,
            text="Selected: None",
            font=("Helvetica", 12),
            bg="#f4efe8",
            fg="#2c2c2c",
        )
        self.selected_label.pack(pady=(8, 8))

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
        rospy.loginfo("Selected destination %s -> tag %s", name, DESTINATIONS[name])
        self.refresh_ui()

    def status_callback(self, msg):
        """Update destination status from the controller."""
        self.at_destination = bool(msg.data)
        self.refresh_ui()

    def set_voice_status(self, text):
        """Update the voice status label safely from the Tk thread."""
        self.voice_status_text = text
        self.refresh_ui()

    def on_voice_button_pressed(self):
        """Start a background voice capture/transcription pass."""
        if not self.voice_enabled:
            self.set_voice_status("Voice mode is disabled by ROS parameter.")
            return

        if self.voice_busy:
            self.set_voice_status("Voice capture already in progress.")
            return

        self.voice_busy = True
        self.last_transcript = ""
        self.set_voice_status("Listening for {} seconds...".format(self.voice_record_seconds))
        worker = threading.Thread(target=self.capture_and_process_voice_command)
        worker.daemon = True
        worker.start()

    def capture_and_process_voice_command(self):
        """Record audio, transcribe it, and map the result to a destination."""
        audio_path = None
        try:
            audio_path = self.record_audio_sample()
            transcript = self.transcribe_audio(audio_path)
            self.last_transcript = transcript
            destination_name = self.match_destination(transcript)
            if destination_name is None:
                self.root.after(
                    0,
                    lambda: self.set_voice_status(
                        "No station match found in: '{}'".format(transcript.strip() or "empty transcript")
                    ),
                )
                return

            self.root.after(0, lambda: self.select_destination(destination_name))
            self.root.after(
                0,
                lambda: self.set_voice_status(
                    "Voice selected {} from '{}'".format(destination_name, transcript.strip())
                ),
            )
        except Exception as exc:
            rospy.logwarn("Voice selection failed: %s", exc)
            self.root.after(0, lambda: self.set_voice_status("Voice selection failed: {}".format(exc)))
        finally:
            if audio_path and os.path.exists(audio_path):
                try:
                    os.remove(audio_path)
                except OSError:
                    pass
            self.voice_busy = False
            self.root.after(0, self.refresh_ui)

    def record_audio_sample(self):
        """Capture a mono WAV sample using arecord."""
        if not self.arecord_path:
            raise RuntimeError("No arecord executable configured")

        with tempfile.NamedTemporaryFile(prefix="apriltag_voice_", suffix=".wav", delete=False) as temp_file:
            audio_path = temp_file.name

        cmd = [
            self.arecord_path,
            "-q",
            "-f",
            "S16_LE",
            "-r",
            str(self.voice_sample_rate),
            "-c",
            "1",
            "-d",
            str(self.voice_record_seconds),
            audio_path,
        ]
        rospy.loginfo("Recording voice command to %s", audio_path)
        try:
            subprocess.check_call(cmd, cwd=self.script_dir)
        except FileNotFoundError:
            raise RuntimeError("arecord is not installed or not on PATH")
        except subprocess.CalledProcessError as exc:
            raise RuntimeError("audio recording failed with exit code {}".format(exc.returncode))
        return audio_path

    def load_voice_backend(self):
        """Load and cache the configured transcription backend."""
        if self.voice_model is not None:
            return self.voice_model

        try:
            backend_module = importlib.import_module(self.voice_backend_module)
        except ImportError:
            raise RuntimeError(
                "transcription backend '{}' is not installed".format(self.voice_backend_module)
            )

        if not hasattr(backend_module, "load_model"):
            raise RuntimeError(
                "backend '{}' does not expose load_model()".format(self.voice_backend_module)
            )

        self.voice_model = backend_module.load_model(self.voice_model_name)
        rospy.loginfo(
            "Loaded voice backend module '%s' with model '%s'",
            self.voice_backend_module,
            self.voice_model_name,
        )
        return self.voice_model

    def transcribe_audio(self, audio_path):
        """Transcribe an audio file using the configured speech backend."""
        model = self.load_voice_backend()

        if not hasattr(model, "transcribe"):
            raise RuntimeError(
                "loaded model from '{}' does not expose transcribe()".format(self.voice_backend_module)
            )

        result = model.transcribe(audio_path, language=self.voice_language)
        if isinstance(result, dict):
            transcript = result.get("text", "")
        else:
            transcript = str(result)

        transcript = transcript.strip()
        if not transcript:
            raise RuntimeError("transcription returned empty text")
        rospy.loginfo("Voice transcript: %s", transcript)
        return transcript

    def match_destination(self, transcript):
        """Return the destination name that best matches the spoken text."""
        lowered = transcript.lower()

        for destination_name, aliases in DESTINATION_ALIASES.items():
            for alias in aliases:
                if alias in lowered:
                    return destination_name

        for destination_name in DESTINATIONS:
            if destination_name.lower() in lowered:
                return destination_name

        return None

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

        if self.voice_busy:
            self.voice_button.config(
                state="disabled",
                bg="#94a3a1",
                activebackground="#94a3a1",
            )
        else:
            self.voice_button.config(
                state="normal" if self.voice_enabled else "disabled",
                bg="#7c9a92" if self.voice_enabled else "#b9c4c1",
                activebackground="#68857d" if self.voice_enabled else "#b9c4c1",
            )

        self.voice_status_label.config(text=self.voice_status_text)
        transcript_text = self.last_transcript if self.last_transcript else "None"
        self.transcript_label.config(text="Heard: {}".format(transcript_text))

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
    app = AprilTagNavigationUIV2()
    app.run()


if __name__ == "__main__":
    main()
