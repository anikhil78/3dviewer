#!/usr/bin/env bash
# =============================================================================
# setup_gz_ros2_control.sh
# =============================================================================
# Builds gz_ros2_control from source against gz-sim 8 (Harmonic / gz-plugin 2.x).
#
# Why: The apt package ros-humble-gz-ros2-control 0.7.17 was compiled against
# gz-plugin 1.x (Ignition Fortress).  It exports IgnitionPluginHook but gz-sim 8
# requires GzPluginHook — the plugin silently fails to load, /controller_manager
# never appears, and the arm falls under gravity with no control.
#
# After running this script:
#   source ~/ros2_ws/install/setup.bash
#   ros2 launch arctos_gazebo sim.launch.py
# =============================================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="$HOME/ros2_ws"
REPO_URL="https://github.com/ros-controls/gz_ros2_control.git"
BRANCH="humble"

echo "======================================================"
echo "  gz_ros2_control source build for ROS 2 $ROS_DISTRO"
echo "======================================================"

# --------------------------------------------------------------------------- #
# 0. Prerequisites                                                             #
# --------------------------------------------------------------------------- #
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "[ERROR] ROS 2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
    echo "        Install it first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

# Check gz-sim 8 is installed
if ! pkg-config --exists gz-sim8 2>/dev/null; then
    echo "[ERROR] gz-sim8 not found. Install Gazebo Harmonic:"
    echo "        sudo apt install gz-harmonic"
    exit 1
fi
GZ_PLUGIN_VERSION=$(pkg-config --modversion gz-plugin2 2>/dev/null || echo "NOT FOUND")
echo "[INFO] gz-plugin version: $GZ_PLUGIN_VERSION"

# --------------------------------------------------------------------------- #
# 1. Remove the broken apt package (won't error if not installed)             #
# --------------------------------------------------------------------------- #
if dpkg -l ros-humble-gz-ros2-control 2>/dev/null | grep -q '^ii'; then
    echo ""
    echo "[STEP 1] Removing incompatible apt package ros-humble-gz-ros2-control..."
    sudo apt remove -y ros-humble-gz-ros2-control
else
    echo "[STEP 1] ros-humble-gz-ros2-control not installed via apt — skipping removal."
fi

# --------------------------------------------------------------------------- #
# 2. Create workspace                                                          #
# --------------------------------------------------------------------------- #
echo ""
echo "[STEP 2] Creating workspace at $WS ..."
mkdir -p "$WS/src"

# --------------------------------------------------------------------------- #
# 3. Clone or update gz_ros2_control                                          #
# --------------------------------------------------------------------------- #
echo ""
if [ -d "$WS/src/gz_ros2_control/.git" ]; then
    echo "[STEP 3] gz_ros2_control already cloned — pulling latest..."
    git -C "$WS/src/gz_ros2_control" fetch origin
    git -C "$WS/src/gz_ros2_control" checkout "$BRANCH"
    git -C "$WS/src/gz_ros2_control" pull origin "$BRANCH"
else
    echo "[STEP 3] Cloning gz_ros2_control ($BRANCH branch)..."
    git clone --branch "$BRANCH" --depth 1 "$REPO_URL" "$WS/src/gz_ros2_control"
fi

# --------------------------------------------------------------------------- #
# 4. rosdep — install any missing system dependencies                         #
# --------------------------------------------------------------------------- #
echo ""
echo "[STEP 4] Installing dependencies via rosdep..."
cd "$WS"
rosdep install --from-paths src/gz_ros2_control --ignore-src -r -y \
    --rosdistro "$ROS_DISTRO" || true   # 'true' so script continues if some
                                         # deps are already satisfied

# --------------------------------------------------------------------------- #
# 5. Build                                                                     #
# --------------------------------------------------------------------------- #
echo ""
echo "[STEP 5] Building gz_ros2_control (this takes ~2 min)..."
cd "$WS"
# GZ_VERSION=harmonic is required so CMakeLists.txt takes the gz-sim8 / gz-plugin2
# branch (line 30: if("$ENV{GZ_VERSION}" STREQUAL "harmonic")).
# Without it the build silently falls into the ignition-plugin1 else-branch and
# the resulting .so is ABI-incompatible with gz-sim 8.
GZ_VERSION=harmonic colcon build \
    --symlink-install \
    --packages-select gz_ros2_control \
    --allow-overriding gz_ros2_control \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    2>&1 | tee /tmp/gz_ros2_control_build.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo ""
    echo "[ERROR] Build failed. See /tmp/gz_ros2_control_build.log"
    echo ""
    echo "  Common causes:"
    echo "  - GZ_VERSION not set: the CMakeLists.txt gates on \$ENV{GZ_VERSION}."
    echo "    This script sets it automatically; if running colcon manually, prefix:"
    echo "      GZ_VERSION=harmonic colcon build ..."
    echo "  - Missing gz-sim8 dev headers: sudo apt install libgz-sim8-dev"
    echo "  - Missing gz-plugin2 dev headers: sudo apt install libgz-plugin2-dev"
    exit 1
fi

# --------------------------------------------------------------------------- #
# 6. Verify the built plugin links gz-plugin2, not gz-plugin1/IgnitionPlugin  #
# --------------------------------------------------------------------------- #
echo ""
echo "[STEP 6] Verifying plugin linkage..."
PLUGIN_SO=$(find "$WS/install" -name 'libgz_ros2_control*.so' 2>/dev/null | head -1)
if [ -z "$PLUGIN_SO" ]; then
    echo "[WARN] Could not find libgz_ros2_control*.so in $WS/install"
else
    echo "[INFO] Plugin path: $PLUGIN_SO"
    if ldd "$PLUGIN_SO" 2>/dev/null | grep -q 'libgz-plugin2\|libgz_plugin2'; then
        echo "[OK]   Links gz-plugin2 (correct for gz-sim 8)"
    elif ldd "$PLUGIN_SO" 2>/dev/null | grep -q 'libignition-plugin\|libign-plugin'; then
        echo "[WARN] Still links ignition-plugin (gz-plugin 1.x) — wrong version built!"
        echo "       Ensure GZ_VERSION=harmonic was set. Re-run this script."
    else
        # ldd output may use the versioned .so name — just print it
        echo "[INFO] ldd output:"
        ldd "$PLUGIN_SO" 2>/dev/null | grep -i 'gz\|ign' | head -10 || true
    fi
fi

# --------------------------------------------------------------------------- #
# 7. Symlink arctos_gazebo into workspace (if not already there)              #
# --------------------------------------------------------------------------- #
echo ""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
ARCTOS_PKG="$REPO_ROOT/arctos_gazebo"
SYMLINK="$WS/src/arctos_gazebo"

if [ ! -L "$SYMLINK" ] && [ ! -d "$SYMLINK" ]; then
    echo "[STEP 7] Symlinking arctos_gazebo into workspace..."
    ln -s "$ARCTOS_PKG" "$SYMLINK"
else
    echo "[STEP 7] arctos_gazebo already in workspace — skipping symlink."
fi

# --------------------------------------------------------------------------- #
# 8. Build arctos_gazebo                                                       #
# --------------------------------------------------------------------------- #
echo ""
echo "[STEP 8] Building arctos_gazebo..."
source "$WS/install/setup.bash"
cd "$WS"
colcon build \
    --symlink-install \
    --packages-select arctos_gazebo \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# --------------------------------------------------------------------------- #
# Done                                                                         #
# --------------------------------------------------------------------------- #
echo ""
echo "======================================================"
echo "  Build complete!"
echo "======================================================"
echo ""
echo "  Next steps:"
echo ""
echo "  1. Source the workspace (add to ~/.bashrc to make permanent):"
echo "       source /opt/ros/$ROS_DISTRO/setup.bash"
echo "       source $WS/install/setup.bash"
echo ""
echo "  2. Launch the simulation:"
echo "       ros2 launch arctos_gazebo sim.launch.py"
echo ""
echo "  3. After ~35 seconds, verify controllers are active:"
echo "       ros2 control list_controllers"
echo "     Expected output:"
echo "       joint_state_broadcaster   active"
echo "       arctos_arm_controller     active"
echo "       arctos_hand_controller    active"
echo ""
echo "  4. Send a test joint position goal:"
echo "       ros2 action send_goal /arctos_arm_controller/follow_joint_trajectory \\"
echo "         control_msgs/action/FollowJointTrajectory \\"
echo "         '{trajectory: {joint_names: [X_joint, Y_joint, Z_joint, A_joint, B_joint, C_joint],
  points: [{positions: [0.0, 0.5, -0.3, 0.0, 0.3, 0.0], time_from_start: {sec: 3}}]}}'"
echo ""
