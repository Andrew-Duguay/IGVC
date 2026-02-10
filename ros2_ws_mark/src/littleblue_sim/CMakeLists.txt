cmake_minimum_required(VERSION 3.8)
project(littleblue_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  models
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/littleblue_sim.dsv.in"
)

ament_package()
