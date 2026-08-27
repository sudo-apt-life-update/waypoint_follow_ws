cmake_minimum_required(VERSION 3.8)
project(g1_description)

# Data only -- URDF and meshes. No nodes, no launch files. The launch side
# lives in g1_walk, which is where the decision "which joint source, which
# clock" actually belongs.
#
# Copied verbatim from slam_ws (src/g1_nav/g1_description) so this workspace
# builds and runs on its own. The URDF and meshes are identical; if either
# workspace changes the model, copy it across by hand rather than pointing one
# at the other.

# setup.sh passes these for other packages; touch them so CMake does not warn
# about unused cache variables in every build.
set(_ignored "${Python3_EXECUTABLE}${UNITREE_SDK2_ROOT}")

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
