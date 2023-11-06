#  Copyright (c) 2022 Jonas Mahler

#  This file is part of realsense2lidar.

#  realsense2lidar is free software: you can redistribute it and/or modify it under the terms 
#  of the GNU General Public License as published by the Free Software Foundation, 
#  either version 3 of the License, or (at your option) any later version.

#  realsense2lidar is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along 
#  with realsense2lidar. If not, see <https://www.gnu.org/licenses/>. 

cmake_minimum_required(VERSION 3.5)
project(realsense2lidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(realsense2lidar_node src/realsense2lidar_node.cpp src/realsense2lidar.cpp)

target_link_libraries( realsense2lidar_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  realsense2lidar_node
  rclcpp std_msgs 
  sensor_msgs
)

install(TARGETS
  realsense2lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Files for testing
install(FILES
  test/cube1_record_1.yaml test/cube1_record_2.yaml test/realsense2lidar_launch.testing.py 
  DESTINATION lib/${PROJECT_NAME}
)

# Rviz2 config for visualization
install(FILES config/realsense2lidar.rviz 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake)    
  add_launch_test(test/realsense2lidar_launch.testing.py)
endif()

ament_package()
