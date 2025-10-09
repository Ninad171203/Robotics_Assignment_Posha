cmake_minimum_required(VERSION 3.8)
project(posha_ros2_workspace)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#
# In the main workspace we don't build anything directly
# The packages handle their own builds
#

ament_package()
