#!/usr/bin/env bash
# Local entrypoint. Defaults to Fake; hardware requires an explicit webpage click.
set -e
STUDIO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname -- "$STUDIO_DIR")"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo '未找到 ROS 2 Jazzy / Humble，请先安装项目依赖。' >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  echo '工作空间尚未构建，请先构建项目。' >&2
  exit 1
fi
source "$WORKSPACE_DIR/install/setup.bash"
# Login shells and user services often have different environments. Keep this
# Local sessions on loopback discovery regardless of the user's ROS setup.
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1
cd -- "$WORKSPACE_DIR"
exec /usr/bin/python3 "$STUDIO_DIR/server.py" --managed-sim "$@"
