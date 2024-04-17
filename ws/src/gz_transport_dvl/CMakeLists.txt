cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-transport-dvl)

# Find the Gazebo Transport library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

# Generate examples for publisher and subscriber
if (NOT MSVC AND EXISTS "${CMAKE_SOURCE_DIR}/time_bridge.cc" AND EXISTS "${CMAKE_SOURCE_DIR}/dvl_bridge.cc")
  add_executable(time_bridge time_bridge.cc)
  add_executable(dvl_bridge dvl_bridge.cc)
  target_link_libraries(time_bridge gz-transport${GZ_TRANSPORT_VER}::core)
  target_link_libraries(dvl_bridge gz-transport${GZ_TRANSPORT_VER}::core)
  # Install the publisher binary
  install(TARGETS time_bridge DESTINATION bin)
  install(TARGETS dvl_bridge DESTINATION bin)
#endif()


endif()

