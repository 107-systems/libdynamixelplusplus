##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("dynamixel++")
##########################################################################
option(BUILD_EXAMPLES "Build all examples provided with this library" OFF)
##########################################################################
add_subdirectory(lib/dynamixel/current)
##########################################################################
add_library(${PROJECT_NAME} STATIC src/Dynamixel++.cpp include/dynamixel++/CommunicationError.h)
##########################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -Wextra -Wpedantic)
endif()
##########################################################################
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} dynamixel)
##########################################################################
if(BUILD_EXAMPLES)
    add_subdirectory(examples/example-01-ping)
    add_subdirectory(examples/example-02-sync-read)
    add_subdirectory(examples/example-03-sync-write)
    add_subdirectory(examples/example-04-read)
    add_subdirectory(examples/example-05-write)
    add_subdirectory(examples/example-xx-minimal)
endif()
##########################################################################
