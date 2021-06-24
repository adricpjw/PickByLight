cmake_minimum_required(VERSION 2.8.3)
project(hmiCmd)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  actionlib
  actionlib_msgs
  tf2_ros
)

add_message_files(DIRECTORY msg FILES 
  ObjRecognised.msg
  ObjRecognisedArr.msg
)

generate_messages (DEPENDENCIES 
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
  lib
)


add_library(lineMarker lib/lineMarker.cpp)
target_link_libraries(lineMarker
  ${catkin_LIBRARIES}
)

add_executable(hmiCmd src/hmiCmd.cpp)
target_link_libraries(hmiCmd
  ${catkin_LIBRARIES}
  lineMarker
)

add_executable(light_broadcaster src/lightarray_generated.cpp)
target_link_libraries(light_broadcaster
  ${catkin_LIBRARIES}
)


