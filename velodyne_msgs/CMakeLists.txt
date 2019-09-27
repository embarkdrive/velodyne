cmake_minimum_required(VERSION 2.8.3)
project(velodyne_msgs)

find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

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
