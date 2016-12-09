cmake_minimum_required(VERSION 2.8.3)
project(dji_ronin)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   gimbalangle.msg
 )

 add_service_files(
   DIRECTORY srv
   FILES
   controlgimbal_srv.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES passivelocalization
  CATKIN_DEPENDS roscpp std_msgs rospy
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


 install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

