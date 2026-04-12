#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG_FILE="${SCRIPT_DIR}/apriltag_system_config.yaml"

python3 "${SCRIPT_DIR}/apriltag_dock_controller.py" \
  _config_file:="${CONFIG_FILE}"
