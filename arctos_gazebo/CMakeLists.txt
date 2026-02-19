cmake_minimum_required(VERSION 3.5)
project(arctos_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_arm.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_arm
)

ament_package()
