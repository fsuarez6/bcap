cmake_minimum_required(VERSION 2.8.3)
project(bcap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    bcap_client
    bcap_server
  CATKIN_DEPENDS 
    roscpp
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

# Definitions
add_definitions(-D_USE_LINUX_API)

# Client library
add_library(bcap_client 
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c
  src/bcap_client.c
)
target_link_libraries(bcap_client ${catkin_LIBRARIES})

# Server library
add_library(bcap_server 
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c
  src/bcap_server.c
)
target_link_libraries(bcap_server ${catkin_LIBRARIES})

# Tests
add_executable(slave_mode tests/slave_mode.cpp)
target_link_libraries(slave_mode bcap_client ${catkin_LIBRARIES})
add_executable(change_digital_output tests/change_digital_output.cpp)
target_link_libraries(change_digital_output bcap_client ${catkin_LIBRARIES})
