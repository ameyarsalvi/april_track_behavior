## AprilPose V1 Basic Working Snapshot

This directory is a fallback copy of the basic working AprilTag navigation stack
as of 2026-04-12.

Contents:
- `apriltag_min_pose_full.py`
- `apriltag_dock_controller.py`
- `apriltag_navigation_ui.py`
- `apriltag_system_config.yaml`
- `run_apriltag_navigation_ui.sh`
- `run_apriltag_dock_controller.sh`

Notes:
- This snapshot preserves the current detector/controller/UI flow that is using
  the shared YAML config for camera intrinsics and robot-to-camera relation.
- Treat this folder as a restore point before future tuning or refactors.

Restore approach:
1. Copy the needed file(s) from this directory back into the repo root.
2. Re-run the launch script or the relevant node.

Example restore commands:
```bash
cp fallback_snapshots/v1_basic_working/apriltag_dock_controller.py .
cp fallback_snapshots/v1_basic_working/apriltag_min_pose_full.py .
cp fallback_snapshots/v1_basic_working/apriltag_navigation_ui.py .
cp fallback_snapshots/v1_basic_working/apriltag_system_config.yaml .
cp fallback_snapshots/v1_basic_working/run_apriltag_navigation_ui.sh .
cp fallback_snapshots/v1_basic_working/run_apriltag_dock_controller.sh .
```
