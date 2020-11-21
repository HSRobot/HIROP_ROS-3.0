cmake_minimum_required(VERSION 2.8.3)
project(perception_bridge)

# Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
find_package(PCL REQUIRED)
find_package(hirop_perception REQUIRED)
find_package(tf REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    message_generation
    moveit_msgs
    hirop_msgs
    geometric_shapes
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2_ros 
    tf2_geometry_msgs
    )


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Look.srv
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
    CATKIN_DEPENDS rospy  std_msgs roscpp
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
     include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
	/usr/local/include
    # /home/st/catkin_ws/devel/include

#    ${hirop_perception_INCLUDE_DIRS}
    )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/perception_bridge.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
aux_source_directory(./src FRAMEWORK_SRCS)
add_executable(${PROJECT_NAME} ${FRAMEWORK_SRCS} )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
   ${hirop_perception_LIBRARIES}
   hirop_perception
   yaml-cpp
   ${tf_LIBRARIES}
 )



