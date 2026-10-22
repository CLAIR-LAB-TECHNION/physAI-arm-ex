# Sourced by the run/ scripts: ROS 2 and this workspace. read_mode checks a
# console's first argument (sim or real).

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    sed -n '2,/^$/s/^# \{0,1\}//p' "$0"
    exit 2
}

case "${1:-}" in
    -h|--help) usage ;;
esac

source /opt/ros/humble/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "The workspace is not set up yet:  $WS/run/setup.sh"
    exit 1
fi
source "$WS/install/setup.bash"

# The consoles' first argument: sim or real.
read_mode() {
    case "${1:-}" in
        sim|real) MODE="$1" ;;
        *) usage ;;
    esac
}

banner() {
    echo "[$(basename "$0")] ${MODE:+mode: $MODE, }ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} (every terminal must use the same)"
}

use_venv() {
    if [ ! -f "$WS/.venv/bin/activate" ]; then
        echo "The Python environment (.venv) is missing:  $WS/run/setup.sh"
        exit 1
    fi
    source "$WS/.venv/bin/activate"
}
