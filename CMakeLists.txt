# CMake build script for the autotross project.
# This script configures the project, finds dependencies, sets compiler options,
# defines source files, and creates the executable target.

cmake_minimum_required(VERSION 3.15)
project(autotross VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard to c++17 and enforce it
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define an option to build for Gazebo simulation (default is True).
# This allows switching between simulation and hardware builds.
option(USE_SIM "Build for Gazebo simulation" True)

# Disable building tests and examples from external projects (like protocore)
# to reduce build time and complexity.
set(BUILD_TESTING OFF CACHE BOOL "Disable external tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable external examples" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Disable Python bindings" FORCE)

# Add the protocore library located in the external directory.
# Define USE_CUSTOM_MSG_VARIANT so protocore uses our custom message variant types
# located in include/msg/msg_variant_types.h instead of its default.
add_compile_definitions(USE_CUSTOM_MSG_VARIANT)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/protocore)

# Add the kfplusplus library located in the external directory.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/kfplusplus)

# Specify include directories for the protocore target *before* it's built.
# This ensures that when protocore is compiled, it finds our custom
# msg_variant_types.h and other necessary headers from this project first.
target_include_directories(protocore BEFORE PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/msg" # Location of custom msg_variant_types.h
    "${CMAKE_CURRENT_SOURCE_DIR}/include"     # General project includes
    "${CMAKE_CURRENT_SOURCE_DIR}/external/kfplusplus/include" # kfplusplus headers
    "${CMAKE_CURRENT_SOURCE_DIR}/external"    # Other external headers if needed
)

# Define the list of source files for the main autotross executable.
set(SOURCES
    src/main.cpp
    src/csc/sensors/imu/imu_task.cpp
    src/csc/sensors/gps/gps_task.cpp
)

# Conditionally include source files and dependencies based on the USE_SIM option.
if(USE_SIM)
  # If building for simulation:
  # Find the Gazebo Transport library (version 13 specifically).
  # The 'QUIET' argument suppresses messages if the package is found.
  # The 'REQUIRED' argument makes CMake stop with an error if not found.
  # 'OPTIONAL_COMPONENTS log' specifies optional parts of the library.
  find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
  # Store the major version number for later use in linking.
  set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

  message(STATUS "Building for Gazebo simulation")
  add_definitions(-DUSE_SIM)
  list(APPEND SOURCES
      src/hal/gazebo/gazebo_imu.cpp
      src/hal/gazebo/gazebo_gps.cpp
      )

  if(gz-transport13_FOUND)
    message(STATUS "Found Gazebo Transport ${GZ_TRANSPORT_VER}")
  else()
    message(FATAL_ERROR "Gazebo Transport not found. Please install Gazebo Transport 13.")
  endif()
else()
  message(STATUS "Building for hardware")
  add_definitions(-DUSE_HARDWARE)
endif()

# Create Executable
add_executable(autotross ${SOURCES})
target_link_libraries(autotross PRIVATE protocore)

# Include directories for headers
target_include_directories(autotross PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/protocore/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/kfplusplus/include
)

# Link Gazebo Transport only if simulation is enabled
if(USE_SIM AND gz-transport13_FOUND)
  # Include Gazebo Transport headers
  target_include_directories(autotross PRIVATE ${gz-transport13_INCLUDE_DIRS})
  
  # Link Gazebo Transport
  target_link_libraries(autotross PRIVATE gz-transport${GZ_TRANSPORT_VER})
endif()

enable_testing()
add_subdirectory(test)
