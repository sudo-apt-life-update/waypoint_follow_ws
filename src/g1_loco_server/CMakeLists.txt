cmake_minimum_required(VERSION 3.10)
project(g1_loco_server CXX)

# ament_cmake is used here as BUILD TOOLING ONLY. This binary must NOT link
# ROS: it is the only process that talks Unitree DDS, and unitree_sdk2 + ROS 2
# in one address space corrupt the heap (both bundle CycloneDDS). Nothing below
# calls find_package on a ROS library, and `ldd` on the result must never show
# one -- setup.sh checks.
#
# Why ament_cmake rather than plain cmake, which is what this started as: a
# plain CMake package gets no ament_prefix_path environment hook, so after
# sourcing install/setup.bash `ros2 run g1_loco_server ...` fails with
# "Package 'g1_loco_server' not found". The binary was on PATH and worked when
# invoked directly, but every doc and habit says `ros2 run`. ament_package()
# registers the package properly; it does not link anything.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# [BUILD TRAP] Default Python3 to the system interpreter.
#
# ament_cmake runs package_xml_2_cmake.py, which needs catkin_pkg. CMake's
# FindPython3 prefers whatever is highest on PATH, and on this machine that is
# ~/.local/bin/python3.10, which has no catkin_pkg -- so the build dies with
# "ModuleNotFoundError: No module named 'catkin_pkg'" long before it compiles
# anything. slam_ws works around this by passing -DPython3_EXECUTABLE on every
# colcon invocation; setting the default here means a bare
# `colcon build --packages-select g1_loco_server` also works. An explicit
# -DPython3_EXECUTABLE on the command line still wins.
if(NOT Python3_EXECUTABLE AND EXISTS /usr/bin/python3)
  set(Python3_EXECUTABLE /usr/bin/python3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)

# Installed by the SDK's own `make install` to /opt/unitree_robotics. Pointing
# at the install tree rather than ~/workspaces/unitree/unitree_sdk2 keeps this
# workspace independent of that checkout.
set(UNITREE_SDK2_ROOT "/opt/unitree_robotics" CACHE PATH
    "unitree_sdk2 install prefix")
find_package(unitree_sdk2 REQUIRED PATHS "${UNITREE_SDK2_ROOT}/lib/cmake"
             NO_DEFAULT_PATH)

#
# [CAUTION] Link zmq by bare name and do NOT add pkg-config's include dirs.
# pkg_check_modules(libzmq) reports ZMQ_INCLUDE_DIRS containing
# /usr/include/x86_64-linux-gnu (plus pgm-5.3, libxml2). Putting that on the
# -I path shadows arch-specific system headers, which changes struct layouts
# underneath the bundled CycloneDDS headers -- the binary then aborts inside
# ChannelFactory::Init with "corrupted size vs. prev_size". zmq.hpp lives in
# /usr/include, which is already on the default path, so nothing is needed.
# (Learned the hard way in slam_ws; see example/g1/CMakeLists.txt there.)
#
find_path(ZMQ_HPP zmq.hpp)
if(NOT ZMQ_HPP)
  message(FATAL_ERROR
          "cppzmq not found. Install it:  sudo apt install libzmq3-dev cppzmq-dev")
endif()

add_executable(g1_loco_server src/g1_loco_server.cpp)
target_link_libraries(g1_loco_server unitree_sdk2 zmq)

#
# [CRITICAL] Pin libddsc/libddscxx to the SDK's own copies.
#
# colcon builds this in a ROS-sourced shell, which puts
# /opt/ros/jazzy/lib/x86_64-linux-gnu at the head of LD_LIBRARY_PATH. The SDK's
# libddscxx was built against the libddsc sitting next to it; if the loader
# pairs it with ROS's libddsc instead, the ABI mismatch corrupts the heap
# inside dds_create_topic_impl. --disable-new-dtags emits DT_RPATH rather than
# DT_RUNPATH, which is the only form that outranks LD_LIBRARY_PATH.
#
# Verify after building:  ldd install/g1_loco_server/bin/g1_loco_server | grep ddsc
# Both lines must point into ${UNITREE_SDK2_ROOT}/lib.
#
set_target_properties(g1_loco_server PROPERTIES
    BUILD_RPATH "${UNITREE_SDK2_ROOT}/lib"
    INSTALL_RPATH "${UNITREE_SDK2_ROOT}/lib")
target_link_options(g1_loco_server PRIVATE "-Wl,--disable-new-dtags")

# Read-only diagnostic: does the G1 publish a pose we can use? A separate
# binary so that probing undocumented api ids cannot destabilise the server
# that is the only thing able to move the robot. Same DDS pinning applies.
add_executable(g1_odom_probe src/g1_odom_probe.cpp)
target_link_libraries(g1_odom_probe unitree_sdk2)
set_target_properties(g1_odom_probe PROPERTIES
    BUILD_RPATH "${UNITREE_SDK2_ROOT}/lib"
    INSTALL_RPATH "${UNITREE_SDK2_ROOT}/lib")
target_link_options(g1_odom_probe PRIVATE "-Wl,--disable-new-dtags")

# lib/${PROJECT_NAME} is where `ros2 run` looks. Installing to bin/ instead
# puts them on PATH but leaves `ros2 run` reporting the executable missing.
install(TARGETS g1_loco_server g1_odom_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
