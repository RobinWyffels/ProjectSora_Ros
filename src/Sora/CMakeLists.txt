cmake_minimum_required(VERSION 3.8)
project(Sora)

find_package(ament_cmake REQUIRED)

# The URDF in robot_description references meshes via package://Sora/assets/...
# Reuse the existing meshes from robot_description without duplicating them in git.
install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../robot_description/assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
