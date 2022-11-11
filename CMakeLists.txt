##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("dynamixel++")
##########################################################################
add_subdirectory(lib/dynamixel/current)
##########################################################################
add_library(${PROJECT_NAME} STATIC src/Dynamixel++.cpp)
##########################################################################
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} dynamixel)
##########################################################################
