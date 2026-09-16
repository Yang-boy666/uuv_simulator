cmake_minimum_required(VERSION 3.8)
project(project3_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/generate_pipeline_world.py
    scripts/thruster_command.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
