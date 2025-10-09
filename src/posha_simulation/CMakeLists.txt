cmake_minimum_required(VERSION 3.8)
project(posha_simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/simple_test.py
  scripts/path_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # comment the following line to skip linting
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Install additional Python scripts
install(PROGRAMS
  scripts/advanced_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

# Install motion control scripts
install(PROGRAMS
  scripts/motion_controller.py
  scripts/interactive_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Gazebo world
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}
)
