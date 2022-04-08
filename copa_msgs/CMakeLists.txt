cmake_minimum_required(VERSION 3.0.2)
project(copa_msgs)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs                
  std_msgs                      
  message_generation            
)
  
add_action_files(
  FILES
  WindSpeed.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)