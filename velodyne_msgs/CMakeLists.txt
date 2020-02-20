cmake_minimum_required(VERSION 2.8.3)
project(velodyne_msgs)

find_program(XCACHE_FOUND xcache_client)
if (XCACHE_FOUND)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE xcache_client)
endif(XCACHE_FOUND)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  VelodynePacket.msg
  VelodyneScan.msg
  VelodyneSweepInfo.msg
  VelodyneDeskewInfo.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
