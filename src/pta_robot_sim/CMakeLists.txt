cmake_minimum_required(VERSION 3.8)
project(pta_robot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    behavior_trees
    config
    launch
    maps
    meshes
    rviz
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(
  PROGRAMS
    src/mecanum_drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)
