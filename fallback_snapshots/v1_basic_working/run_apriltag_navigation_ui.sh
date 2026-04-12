#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG_FILE="${SCRIPT_DIR}/apriltag_system_config.yaml"
IMAGE_TOPIC="/camera/color/image_raw"
TAG_FAMILY="tag36h11"
TAG_SIZE="0.450"

python3 "${SCRIPT_DIR}/apriltag_navigation_ui.py" \
  _config_file:="${CONFIG_FILE}" \
  _image_topic:="${IMAGE_TOPIC}" \
  _family:="${TAG_FAMILY}" \
  _tag_size:="${TAG_SIZE}"
