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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Create C++ executables
add_executable(pybullet_publisher
  src/shikaku/pybullet_publisher.cpp
  src/pybullet_publisher_main.cpp)
ament_target_dependencies(pybullet_publisher 
  rclcpp
  sensor_msgs)
target_link_libraries(pybullet_publisher ${DEPENDENCIES})

# Install C++ executables
install(TARGETS
  pybullet_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
