#!/usr/bin/env bash
# Step 0, once (and again after pulling new code): set up this PC to run the
# consoles. Needs no sudo. Every step skips what is done already:
#
#   1. a check of the system packages (it installs none: if some are
#      missing, it prints the apt command for an administrator and stops)
#   2. the Python environment .venv (requirements.txt: SAM 3, the task planner)
#   3. the Hugging Face token (.env) and the SAM 3 checkpoint (3.4 GB)
#   4. the ROS workspace (colcon build)
#   5. a check that everything is in place
#
#   run/0_setup.sh [--no-model] [--fresh-venv]
#
#   --no-model    do not download SAM 3 now (it is downloaded on first use)
#   --fresh-venv  delete .venv and make it again
#
# Needs Ubuntu 22.04 with ROS 2 Humble in /opt/ros/humble (ros-humble-desktop)
# and the packages below. The robot's drivers and the camera node run on
# other computers; the simulation needs more packages (README). The Hugging
# Face token is taken from .env, else from $HF_TOKEN, else asked for.

set -eo pipefail
trap 'echo "ERROR: run/0_setup.sh stopped at line $LINENO, in: $BASH_COMMAND" >&2' ERR

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$WS/.venv"
MODEL_ID="facebook/sam3"
# What the consoles need beyond ros-humble-desktop
APT_PACKAGES=(
    ros-humble-control-msgs             # arm and gripper actions
    ros-humble-controller-manager-msgs  # which arm controller is active
    ros-humble-xacro                    # the robot model (camera pose, motion planning)
    ros-humble-ur-description           # the UR5e in that model
    ros-humble-moveit-ros-move-group    # MoveIt: motion planning (move_group)
    ros-humble-moveit-planners-ompl     # MoveIt: its planners (RRT-Connect)
    ros-humble-moveit-kinematics        # MoveIt: inverse kinematics (KDL)
    ros-humble-moveit-simple-controller-manager  # MoveIt: loaded by move_group (it executes nothing here)
    python3.10-venv                     # python3 -m venv
    python3-colcon-common-extensions    # colcon build
)

MODEL=1 FRESH=0
for a in "$@"; do
    case "$a" in
        --no-model) MODEL=0 ;;
        --fresh-venv) FRESH=1 ;;
        -h|--help) sed -n '2,/^$/s/^# \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "unknown option $a (see run/0_setup.sh --help)"; exit 2 ;;
    esac
done

step() { echo; echo "== $*"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

if [ ! -f /opt/ros/humble/setup.bash ]; then
    fail "ROS 2 Humble is not installed (/opt/ros/humble): see
       https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html (ros-humble-desktop)"
fi

# ---- 1. system packages ----------------------------------------------------------------------
step "1/5 system packages"
missing=()
for p in "${APT_PACKAGES[@]}"; do
    dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed" || missing+=("$p")
done
if [ ${#missing[@]} -gt 0 ]; then
    fail "missing system packages: ${missing[*]}
       An administrator installs them with:
       sudo apt-get install ${missing[*]}"
fi
echo "all ${#APT_PACKAGES[@]} installed"

# ---- 2. the Python environment -----------------------------------------------------------------
step "2/5 the Python environment ($VENV)"
if [ $FRESH -eq 1 ] && [ -d "$VENV" ]; then
    echo "deleting the old .venv"
    rm -rf "$VENV"
fi
if [ -d "$VENV" ] && ! grep -q "include-system-site-packages = true" "$VENV/pyvenv.cfg" 2>/dev/null; then
    fail ".venv does not see the ROS packages (made without --system-site-packages): run/0_setup.sh --fresh-venv"
fi
if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv --system-site-packages "$VENV"
    echo "made $VENV"
fi
if command -v nvidia-smi >/dev/null; then
    if gpu=$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1 | head -1) \
            && [[ "$gpu" == *,* ]]; then
        driver=${gpu##*, }
        echo "GPU: ${gpu%%,*}, driver $driver"
        [ "${driver%%.*}" -ge 570 ] 2>/dev/null || echo "WARNING: torch (CUDA 12.8) needs NVIDIA driver 570 or newer"
    else
        # Typically "Driver/library version mismatch": the driver was
        # updated (apt upgrade) and the old one is still loaded.
        echo "WARNING: nvidia-smi fails: $gpu"
        echo "         torch will not see the GPU until this is fixed; after a driver update, reboot"
    fi
else
    echo "WARNING: no NVIDIA GPU found (nvidia-smi): SAM 3 would run on the CPU, far too slowly"
fi
echo "installing requirements.txt (the first time: about 5 GB, mostly torch)"
"$VENV/bin/pip" install --disable-pip-version-check -r "$WS/requirements.txt"

# ---- 3. Hugging Face token and SAM 3 ------------------------------------------------------------
step "3/5 SAM 3 ($MODEL_ID)"
token=""
[ -f "$WS/.env" ] && token=$(sed -n 's/^HF_TOKEN=//p' "$WS/.env" | tail -1 | tr -d "\"' ")
case "$token" in "<"*) token="" ;; esac           # the README's placeholder
if [ -z "$token" ]; then
    token="${HF_TOKEN:-}"
    if [ -z "$token" ] && [ -t 0 ]; then
        echo "SAM 3 is a gated model: accept its licence at https://huggingface.co/$MODEL_ID, then make a"
        echo "read token at https://huggingface.co/settings/tokens"
        read -r -s -p "Hugging Face token (Enter to skip): " token
        echo
    fi
    if [ -n "$token" ]; then
        sed -i '/^HF_TOKEN=/d' "$WS/.env" 2>/dev/null || true
        echo "HF_TOKEN=$token" >> "$WS/.env"
        chmod 600 "$WS/.env"
        echo "token saved in .env (git-ignored)"
    fi
fi
if [ $MODEL -eq 1 ]; then
    HF_TOKEN="$token" "$VENV/bin/python" - "$MODEL_ID" <<'EOF'
import os
import sys

from huggingface_hub import snapshot_download
from huggingface_hub.errors import GatedRepoError, RepositoryNotFoundError

model, token = sys.argv[1], os.environ.get("HF_TOKEN") or None
files = ["*.json", "*.txt", "*.safetensors"]
try:
    path = snapshot_download(model, allow_patterns=files, local_files_only=True)
    print(f"in the Hugging Face cache: {path}")
except Exception:  # noqa: BLE001  (not cached yet)
    if token is None:
        sys.exit("ERROR: no Hugging Face token: SAM 3 cannot be downloaded (run/0_setup.sh again to give one)")
    print("downloading (3.4 GB) ...")
    try:
        path = snapshot_download(model, allow_patterns=files, token=token)
    except (GatedRepoError, RepositoryNotFoundError):
        sys.exit(f"ERROR: this token cannot read {model}: a wrong token, or the licence is not accepted yet "
                 f"(https://huggingface.co/{model}); run/0_setup.sh again for another token")
    print(f"downloaded to {path}")
EOF
else
    echo "skipped (--no-model): SAM 3 is downloaded when a console first starts"
fi

# ---- 4. the ROS workspace -------------------------------------------------------------------------
step "4/5 the ROS workspace (colcon build)"
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# ---- 5. check -------------------------------------------------------------------------------------
step "5/5 check"
source "$WS/install/setup.bash"
source "$VENV/bin/activate"
python - <<'EOF'
import control_msgs.action  # noqa: F401
import controller_manager_msgs.srv  # noqa: F401
import cv2  # noqa: F401
import numpy
import rclpy  # noqa: F401  (ROS from the .venv)
import scipy  # noqa: F401
import torch
import transformers
import yaml  # noqa: F401

from lab_replica import config
from lab_replica.tamp import planner, task
from lab_replica.tamp.goal import parse_goal

assert numpy.__version__.startswith("1."), f"numpy {numpy.__version__}: ROS Humble needs numpy 1"
from lab_replica.robot.kinematics import URKinematics

print(f"lab_replica from {config.workspace_root()}")
URKinematics("ur5e")        # the robot model: xacro and ur_description
gpu = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO GPU (CPU only)"
print(f"torch {torch.__version__}, CUDA: {gpu}; transformers {transformers.__version__}")
if not torch.cuda.is_available():
    print("WARNING: torch cannot use the GPU, so SAM 3 would run on the CPU, far too slowly. "
          "Check nvidia-smi (after a driver update: reboot), then run run/0_setup.sh again.")
plan = task.solve(task.most_permissive_init(), parse_goal("(in-basket green-mug)")[0])
print(f"planner (UPF, {planner.ENGINE}): (in-basket green-mug) -> {' '.join(plan)}")
EOF

echo
echo "Set up. Next, with the robot's drivers and the camera running on their computers:"
echo "  run/3_actions.sh real    or    run/4_goals.sh real    (README, \"Run it on the real robot\")"
