#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG_FILE="${SCRIPT_DIR}/apriltag_system_config.yaml"
IMAGE_TOPIC="/camera/color/image_raw"
TAG_FAMILY="tag36h11"
TAG_SIZE="0.450"
VOICE_BACKEND_MODULE="whisper"
VOICE_MODEL="base"
VOICE_RECORD_SECONDS="4"
VOICE_LANGUAGE="en"

python3 "${SCRIPT_DIR}/apriltag_navigation_ui_v2.py" \
  _config_file:="${CONFIG_FILE}" \
  _image_topic:="${IMAGE_TOPIC}" \
  _family:="${TAG_FAMILY}" \
  _tag_size:="${TAG_SIZE}" \
  _voice_backend_module:="${VOICE_BACKEND_MODULE}" \
  _voice_model:="${VOICE_MODEL}" \
  _voice_record_seconds:="${VOICE_RECORD_SECONDS}" \
  _voice_language:="${VOICE_LANGUAGE}"
