cmake_minimum_required(VERSION 3.8)
project(littleblue_sim)

# Deploy-only slim: only the URDF and launch files are installed.
# Sim-only assets (worlds, gazebo models, pass_or_fail) are not
# included in the deploy branch — they're only used by Gazebo, which
# isn't present on the real robot.

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
