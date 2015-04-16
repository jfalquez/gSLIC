project(gSLIC)
cmake_minimum_required(VERSION 2.8.10)

# Default flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")

# Include local cmake modules directory.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Library.
add_subdirectory(library)

# Example application.
option(BUILD_APPLICATION "Build Application" ON)
if(BUILD_APPLICATION)
    add_subdirectory(application)
endif()
