#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(dirname -- "$(dirname -- "$package_dir")")"
# ROS environment scripts read optional variables that are unset in a clean shell.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}"
if ! /usr/bin/python3 -c 'import cv2, yaml, numpy, tkinter, websocket, rosbag2_py; from PIL import ImageTk; from sensor_msgs.msg import Image' 2>/dev/null; then
  printf '%s\n' '필수 패키지를 확인하세요. WORKBENCH.md의 최초 설치 절차를 따라 주세요.' >&2
  printf '%s\n' 'GUI 의존성: sudo apt install python3-websocket python3-tk python3-pil.imagetk' >&2
  exit 1
fi
cd -- "$workspace_dir"
exec /usr/bin/python3 -m pinky_soccer_calibration.workbench "$@"
