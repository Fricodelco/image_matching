cmake_minimum_required(VERSION 3.0.2)
project(copa_msgs)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs                
  std_msgs                      
  sensor_msgs
  message_generation            
)
  
add_action_files(
  DIRECTORY action
  FILES
  WindSpeed.action
)
add_message_files(
  DIRECTORY msg
  FILES
  ImageImu.msg
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)