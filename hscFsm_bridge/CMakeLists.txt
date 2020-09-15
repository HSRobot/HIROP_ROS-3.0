cmake_minimum_required(VERSION 3.5)

project(hscfsm_bridge)

add_compile_options(-std=c++11 -D_LINUX_)

add_definitions(-D__LINUX__)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
FIND_PACKAGE(HPlugin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hirop_msgs
#  message_generation
  tf
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
# message_runtime
 hirop_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
  framework
  framework/loader
  appTEST
)

#link_directories(./3rd/lib/HsApi)
#file(GLOB_RECURSE ALL_SOURCE "*.cpp" "*.c" "*.h")
#file(GLOB USER_LIBS_PATH ./src/*.cpp)
file(GLOB ALL_FRAMEWORK  framework/*.cpp framework/loader/*.cpp)
file(GLOB ALL_BRIDGE_SOURCE src/*.cpp)
list(REMOVE_ITEM ALL_BRIDGE_SOURCE /home/fshs/catkin_ws/src/HIROP_ROS-3.0/hscFsm_bridge/src/test.cpp)
file(GLOB ALL_APPTEST appTEST/*.cpp)
list(APPEND ALL_SOURCE ${ALL_FRAMEWORK} ${ALL_APPTEST})
#message(${ALL_SOURCE})

list(REMOVE_ITEM ALL_APPTEST /home/fshs/catkin_ws/src/HIROP_ROS-3.0/hscFsm_bridge/appTEST/VoiceCtlRobRosFunc.cpp
    /home/fshs/catkin_ws/src/HIROP_ROS-3.0/hscFsm_bridge/appTEST/VoiceCtlRobTask.cpp )


## HsTaskFramework 主要框架的设计
add_library(HsTASKFramework ${ALL_FRAMEWORK})

target_link_libraries(HsTASKFramework
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
)

## HsTaskFramework 主要的APP上位机
add_library(PickPlaceTask ${ALL_APPTEST})

target_link_libraries(PickPlaceTask
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
)


# 所有的代码进行集合
add_executable(${PROJECT_NAME}  ${ALL_APPTEST} ${ALL_BRIDGE_SOURCE})

add_dependencies(${PROJECT_NAME} ${hirop_msgs_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
  HsTASKFramework
)


#add_executable(${PROJECT_NAME}_test src/test.cpp)
#target_link_libraries(${PROJECT_NAME}_test dl gtest boost_system boost_filesystem)
