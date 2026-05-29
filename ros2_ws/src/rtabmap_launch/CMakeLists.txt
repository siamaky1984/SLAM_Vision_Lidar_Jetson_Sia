# MIT License – see LICENSE in repo root
cmake_minimum_required(VERSION 3.8)
project(rtabmap_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
