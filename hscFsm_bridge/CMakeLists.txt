cmake_minimum_required(VERSION 3.5)
project(hscFsm_bridge)

add_compile_options(-std=c++11 -D_LINUX_)

add_definitions(-D__LINUX__)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/fshs/catkin_ws/devel/include
  /home/fshs/catkin_ws/src/hirop_ros/hscFsm_bridge/src
  /home/fshs/catkin_ws/src/hirop_ros/hscFsm_bridge/framework
)

#link_directories(./3rd/lib/HsApi)

add_executable(${PROJECT_NAME} 
               src/main.cpp src/HsFsmBridge.cpp framework/HsTaskFramework.cpp framework/pickplacetask.cpp)

add_dependencies(${PROJECT_NAME} ${hirop_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
