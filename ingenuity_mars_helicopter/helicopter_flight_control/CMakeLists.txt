cmake_minimum_required(VERSION 3.22)
project(helicopter_flight_control)

# Default to C11 for C and C++17 for C++
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(pluginlib REQUIRED)

#  find_package(ignition-gazebo6 REQUIRED)
#  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
#  message(STATUS "Compiling against Ignition Fortress")

#find_package(ignition-plugin1 REQUIRED)
#set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Include directories
include_directories(include)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/helicopter_flight_control.cc
)

target_link_libraries(${PROJECT_NAME}
   gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
#  ignition-gazebo${IGN_GAZEBO_VER}::core
#  ignition-plugin${IGN_PLUGIN_VER}::register
)

# Ament target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Install launch files and other directories if necessary
# e.g., install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Setup the project
ament_package()
