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

ament_package()
