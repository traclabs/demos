cmake_minimum_required(VERSION 3.8)
project(lunar_sun_gz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(lunar_sun_gz_plugin SHARED
src/lunar_sun.cpp)

if( "$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  target_link_libraries(lunar_sun_gz_plugin PRIVATE
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

set_property(TARGET lunar_sun_gz_plugin PROPERTY CXX_STANDARD 17)
ament_target_dependencies(lunar_sun_gz_plugin PUBLIC ament_index_cpp)

install(
  TARGETS
  lunar_sun_gz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv")

ament_package()
