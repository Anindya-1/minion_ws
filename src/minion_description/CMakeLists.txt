cmake_minimum_required(VERSION 3.8)
project(minion_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf gazebo meshes ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
