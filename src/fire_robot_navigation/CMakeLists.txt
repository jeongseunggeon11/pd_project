cmake_minimum_required(VERSION 3.8)
project(fire_robot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/world_to_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
