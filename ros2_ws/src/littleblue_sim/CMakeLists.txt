cmake_minimum_required(VERSION 3.8)
project(littleblue_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install all resource directories
install(
  DIRECTORY
    config
    launch
    urdf
    worlds
    models   
  DESTINATION
    share/${PROJECT_NAME}
)

# Install the Python executables
install(PROGRAMS
  pass_or_fail/pass_or_fail.py
  scripts/world_pose_relay.py
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