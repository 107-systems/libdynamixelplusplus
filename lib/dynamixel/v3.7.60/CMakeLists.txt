##########################################################################
cmake_minimum_required(VERSION 3.16)
##########################################################################
set(DYNAMIXEL_SDK_TARGET "dynamixel")
##########################################################################
add_library(${DYNAMIXEL_SDK_TARGET} STATIC "")
##########################################################################
target_include_directories(${DYNAMIXEL_SDK_TARGET} PUBLIC include)
##########################################################################
target_sources(${DYNAMIXEL_SDK_TARGET}
  PRIVATE
  src/packet_handler.cpp
  src/protocol1_packet_handler.cpp
  src/protocol2_packet_handler.cpp
  src/group_sync_read.cpp
  src/group_sync_write.cpp
  src/group_bulk_read.cpp
  src/group_bulk_write.cpp
  src/port_handler.cpp
  src/port_handler_linux.cpp
)
##########################################################################
target_compile_options(${DYNAMIXEL_SDK_TARGET} PRIVATE -std=c++11)
##########################################################################
