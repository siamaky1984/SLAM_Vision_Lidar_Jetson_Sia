# MIT License – see LICENSE in repo root
cmake_minimum_required(VERSION 3.8)
project(orb_slam3_wrapper)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/odom_scale_aligner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
