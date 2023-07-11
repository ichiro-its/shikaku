cmake_minimum_required(VERSION 3.5)
project(shikaku)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create C++ executables
add_executable(data_processor
  src/shikaku/data_processor.cpp
  src/data_processor_main.cpp)
ament_target_dependencies(data_processor 
  rclcpp
  sensor_msgs
  tachimawari_interfaces)
target_link_libraries(data_processor ${DEPENDENCIES})

# Install C++ executables
install(TARGETS
  data_processor
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  tachimawari_interfaces)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/crpybullet_visualizer_main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
