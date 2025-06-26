cmake_minimum_required(VERSION 3.8)
project(minion_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
