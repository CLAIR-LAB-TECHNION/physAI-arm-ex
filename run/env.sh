# Sourced by the run/ scripts: ROS 2, this workspace, and the check of the
# first argument (sim or real).

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    sed -n '2,/^$/s/^# \{0,1\}//p' "$0"
    exit 2
}

case "${1:-}" in
    sim|real) MODE="$1" ;;
    *) usage ;;
esac

source /opt/ros/humble/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "The workspace is not set up yet:  $WS/run/0_setup.sh"
    exit 1
fi
source "$WS/install/setup.bash"
echo "[$(basename "$0")] mode: $MODE, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} (all four terminals must use the same)"

use_venv() {
    if [ ! -f "$WS/.venv/bin/activate" ]; then
        echo "The Python environment (.venv) is missing:  $WS/run/0_setup.sh"
        exit 1
    fi
    source "$WS/.venv/bin/activate"
}
