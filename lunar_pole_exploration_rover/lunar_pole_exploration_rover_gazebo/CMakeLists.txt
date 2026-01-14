cmake_minimum_required(VERSION 3.8)
project(lunar_pole_exploration_rover_gazebo)

# Find required packages
find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.

if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
  # Harmonic (default)
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  
  find_package(gz-sensors8 REQUIRED)
  set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})  

  message(STATUS "Compiling against Gazebo Harmonic")
elseif("$ENV{GZ_VERSION}" STREQUAL "ionic")
  # Ionic
  find_package(gz-cmake4 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake4_VERSION_MAJOR})

  gz_find_package(gz-common6 REQUIRED)
  set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

  gz_find_package(gz-rendering9 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering9_VERSION_MAJOR})
  
  gz_find_package(gz-sim9 REQUIRED)
  set(GZ_SIM_VER ${gz-sim9_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Ionic")
elseif("$ENV{GZ_VERSION}" STREQUAL "jetty")
  # Jetty
  find_package(gz-cmake REQUIRED)
  set(GZ_CMAKE_VER "")

  gz_find_package(gz-common REQUIRED)
  set(GZ_COMMON_VER "")

  gz_find_package(gz-rendering REQUIRED)
  set(GZ_RENDERING_VER "")
  
  gz_find_package(gz-sim REQUIRED)
  set(GZ_SIM_VER "")

  message(STATUS "Compiling against Gazebo Jetty")
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  # Garden
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering7 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
  
  gz_find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()



# Add the plugins directory to the include path
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Plugins)

# Add the SolarPanelPlugin library
#add_library(SolarPanelPlugin SHARED
#  plugins/SolarPanelPlugin.cc
#)

# Link the SolarPanelPlugin library with required dependencies
#target_link_libraries(SolarPanelPlugin
#  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
#  PRIVATE ignition-common${IGNITION_COMMON_VER}::ignition-common${IGNITION_COMMON_VER}
#  PRIVATE ignition-rendering${IGNITION_RENDERING_VER}::ignition-rendering${IGNITION_RENDERING_VER}
#  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
#)

# Add the RadioisotopeThermalGeneratorPlugin library
#add_library(RadioisotopeThermalGeneratorPlugin SHARED
#  plugins/RadioisotopeThermalGeneratorPlugin.cc
#)

# Link the RadioisotopeThermalGeneratorPlugin library with required dependencies
#target_link_libraries(RadioisotopeThermalGeneratorPlugin
#  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
#  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
#)

# Add the RechargeableBatteryPlugin library
#add_library(RechargeableBatteryPlugin SHARED
#  plugins/RechargeableBatteryPlugin.cc
#)

# Link the RechargeableBatteryPlugin library with required dependencies
#target_link_libraries(RechargeableBatteryPlugin
#  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
#  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
#  # PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGNITION_COMMON_VER}
#)


# add the SensorPowerSystemPlugin library
add_library(SensorPowerSystemPlugin SHARED
  plugins/SensorPowerSystemPlugin.cc
)

# Link the SensorPowerSystemPlugin library with required dependencies
target_link_libraries(SensorPowerSystemPlugin
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
)  
#  ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
#  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
#  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::ignition-gazebo${IGNITION_SIM_VER}
#  PRIVATE ignition-sensors${IGNITION_SENSORS_VER}::ignition-sensors${IGNITION_SENSORS_VER}
#)

# Install directories
install(DIRECTORY
  models launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Install the plugin library
install(TARGETS 
#  SolarPanelPlugin 
#  RadioisotopeThermalGeneratorPlugin 
#  RechargeableBatteryPlugin 
  SensorPowerSystemPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
