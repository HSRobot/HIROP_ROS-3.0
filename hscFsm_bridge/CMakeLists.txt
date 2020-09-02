cmake_minimum_required(VERSION 3.5)

project(hscfsm_bridge)

add_compile_options(-std=c++11 -D_LINUX_)

add_definitions(-D__LINUX__)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

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
  appTEST
)

#link_directories(./3rd/lib/HsApi)
#file(GLOB_RECURSE ALL_SOURCE "*.cpp" "*.c" "*.h")
#file(GLOB USER_LIBS_PATH ./src/*.cpp)
file(GLOB ALL_FRAMEWORK  framework/*.cpp)
file(GLOB ALL_SOURCE src/*.cpp)
file(GLOB ALL_APPTEST appTEST/*.cpp)
list(APPEND ALL_SOURCE ${ALL_FRAMEWORK} ${ALL_APPTEST})
#message(${ALL_SOURCE})

add_executable(${PROJECT_NAME}  ${ALL_SOURCE})

add_dependencies(${PROJECT_NAME} ${hirop_msgs_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
