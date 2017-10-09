cmake_minimum_required(VERSION 2.8.3)
project(easyfly)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  crazyflie_cpp
  geometry_msgs
  message_generation
  genmsg
)
add_message_files(
  FILES
  commands.msg
  pos_ctrl_sp.msg
  raw_ctrl_sp.msg
  trj_ctrl_sp.msg
  state_est.msg
  output.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS 
  program_options 
#  bind
  REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES easyfly
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  message_runtime
  crazyflie_cpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fly
  src/fly.cpp
)
target_link_libraries(fly
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(commander
  src/commander.cpp
)
target_link_libraries(commander
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(linker
  src/linker.cpp
)
target_link_libraries(linker
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(joysticks2
  src/joysticks2.cpp
)
target_link_libraries(joysticks2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(controller
  src/controller.cpp
)
target_link_libraries(controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
