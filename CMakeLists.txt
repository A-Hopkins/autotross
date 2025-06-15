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

# Define the list of source files for the main autotross executable.
set(SOURCES
    src/main.cpp
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
  set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

  message(STATUS "Building for Gazebo simulation")
  add_definitions(-DUSE_SIM)
  list(APPEND SOURCES

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
)

# Link Gazebo Transport only if simulation is enabled
if(USE_SIM AND gz-transport14_FOUND)
  # Include Gazebo Transport headers
  target_include_directories(autotross PRIVATE ${gz-transport14_INCLUDE_DIRS})
  
  # Link Gazebo Transport
  target_link_libraries(autotross PRIVATE gz-transport${GZ_TRANSPORT_VER})
endif()
