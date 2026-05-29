# MIT License – see LICENSE in repo root
cmake_minimum_required(VERSION 3.8)
project(tf_broadcasters)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
