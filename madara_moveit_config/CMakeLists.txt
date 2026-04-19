cmake_minimum_required(VERSION 3.8)
project(madara_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf urdf rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

ament_package()