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

install(PROGRAMS
  scripts/move_arm_moveit.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_arm_moveit
)

install(PROGRAMS
  scripts/vision_processor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vision_processor
)

ament_package()
