cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(gz-transport-dvl)

# Find the Gazebo Transport library
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

# Generate examples for publisher and subscriber
if (NOT MSVC AND EXISTS "${CMAKE_SOURCE_DIR}/publisher.cc")
  add_executable(publisher publisher.cc)
  target_link_libraries(publisher gz-transport${GZ_TRANSPORT_VER}::core)

  # Install the publisher binary
  install(TARGETS publisher DESTINATION bin)
endif()

if (EXISTS "${CMAKE_SOURCE_DIR}/subscriber.cc")
  add_executable(subscriber subscriber.cc)
  target_link_libraries(subscriber gz-transport${GZ_TRANSPORT_VER}::core)

  # Install the subscriber binary
  install(TARGETS subscriber DESTINATION bin)
endif()

# Logging examples (if the log component is available)
if (TARGET gz-transport${GZ_TRANSPORT_VER}::log)
  if (EXISTS "${CMAKE_SOURCE_DIR}/record.cc")
    add_executable(log_record record.cc)
    target_link_libraries(log_record gz-transport${GZ_TRANSPORT_VER}::log)

    # Install the log_record binary
    install(TARGETS log_record DESTINATION bin)
  endif()

  if (EXISTS "${CMAKE_SOURCE_DIR}/playback.cc")
    add_executable(log_playback playback.cc)
    target_link_libraries(log_playback gz-transport${GZ_TRANSPORT_VER}::log)

    # Install the log_playback binary
    install(TARGETS log_playback DESTINATION bin)
  endif()

  if (EXISTS "${CMAKE_SOURCE_DIR}/custom_query.cc")
    add_executable(log_custom_query custom_query.cc)
    target_link_libraries(log_custom_query gz-transport${GZ_TRANSPORT_VER}::log)

    # Install the log_custom_query binary
    install(TARGETS log_custom_query DESTINATION bin)
  endif()
endif()
