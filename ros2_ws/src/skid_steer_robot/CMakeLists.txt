cmake_minimum_required(VERSION 3.8)
project(skid_steer_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install all resource directories
install(
  DIRECTORY
    config
    launch
    urdf
    worlds
    #models   # optional — create the folder only if you have meshes/textures
  DESTINATION
    share/${PROJECT_NAME}
)

# Install the Python executable
install(PROGRAMS
  finish_line/finish_line.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing (unchanged)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
