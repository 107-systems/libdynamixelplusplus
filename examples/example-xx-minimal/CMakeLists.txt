##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(EXAMPLE_XX_TARGET example-xx-minimal)
##########################################################################
add_executable(${EXAMPLE_XX_TARGET} example-xx-minimal.cpp)
target_link_libraries(${EXAMPLE_XX_TARGET} dynamixel++)
##########################################################################
