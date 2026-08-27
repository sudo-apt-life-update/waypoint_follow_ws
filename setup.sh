#!/usr/bin/env bash
#
# Build waypoint_follow_ws from a clean checkout.
#
#   source /opt/ros/jazzy/setup.bash
#   ./setup.sh
#
# There is no .repos file: this workspace vendors nothing. Its one external
# dependency is unitree_sdk2, which is installed by hand (see README) and is
# checked for below rather than fetched, because building it also installs
# CycloneDDS into /opt and that is not something a setup script should do
# behind your back.

set -e

echo "========================================="
echo "Setting up waypoint_follow_ws"
echo "========================================="

WORKSPACE=$(cd "$(dirname "$0")" && pwd)
cd "$WORKSPACE"

###########################################################
# Check ROS
###########################################################

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "This workspace requires ROS 2 Jazzy."
    echo "Current ROS_DISTRO = ${ROS_DISTRO:-<not sourced>}"
    exit 1
fi

echo "ROS_DISTRO = $ROS_DISTRO"

###########################################################
# Check the Unitree SDK
###########################################################

UNITREE_SDK2_ROOT=${UNITREE_SDK2_ROOT:-/opt/unitree_robotics}

if [ ! -f "$UNITREE_SDK2_ROOT/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]; then
    echo ""
    echo "unitree_sdk2 not found at $UNITREE_SDK2_ROOT."
    echo "g1_loco_server cannot build without it, and without that binary"
    echo "nothing in this workspace can move the robot."
    echo ""
    echo "Install it:"
    echo "  git clone https://github.com/unitreerobotics/unitree_sdk2.git"
    echo "  cd unitree_sdk2 && mkdir build && cd build"
    echo "  cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics"
    echo "  sudo make install"
    echo ""
    echo "Set UNITREE_SDK2_ROOT if it lives somewhere else."
    exit 1
fi

echo "unitree_sdk2 = $UNITREE_SDK2_ROOT"

###########################################################
# Check cppzmq
###########################################################

if [ ! -f /usr/include/zmq.hpp ]; then
    echo ""
    echo "cppzmq not found. Install it:"
    echo "  sudo apt install libzmq3-dev cppzmq-dev"
    exit 1
fi

###########################################################
# Install dependencies
###########################################################

echo ""
echo "Installing rosdep dependencies..."

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y

###########################################################
# Build
###########################################################

echo ""
echo "Building workspace..."

colcon build \
    --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DUNITREE_SDK2_ROOT="$UNITREE_SDK2_ROOT"

###########################################################
# Verify the DDS pinning
###########################################################

# The one build detail that fails silently and expensively: if the loader pairs
# the SDK's libddscxx with ROS's libddsc, the binary corrupts its own heap the
# first time it touches DDS. Catch it here rather than on the robot.
echo ""
echo "Checking g1_loco_server's DDS libraries..."

if ldd install/g1_loco_server/lib/g1_loco_server/g1_loco_server | grep ddsc | grep -qv "$UNITREE_SDK2_ROOT"; then
    echo ""
    echo "[WARNING] g1_loco_server is linked against a libddsc outside"
    echo "$UNITREE_SDK2_ROOT. That is the ABI mismatch that aborts inside"
    echo "ChannelFactory::Init with 'corrupted size vs. prev_size'."
    ldd install/g1_loco_server/lib/g1_loco_server/g1_loco_server | grep ddsc
    exit 1
fi

ldd install/g1_loco_server/lib/g1_loco_server/g1_loco_server | grep ddsc | sed 's/^/  /'

###########################################################
# Done
###########################################################

echo ""
echo "========================================="
echo "Workspace ready. Next:"
echo "  source install/setup.bash"
echo "  ros2 run g1_walk check_path square_2m.yaml"
echo "========================================="
