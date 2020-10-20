cmake_minimum_required(VERSION 2.8.3)
project(hirop_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  trajectory_msgs
  shape_msgs
  moveit_msgs
)
 
file(GLOB MSG_LIST msg/*.msg)
set(subMsgList "")
foreach(children ${MSG_LIST})
      file(RELATIVE_PATH childrenRmTxtRel ${CMAKE_CURRENT_SOURCE_DIR}/srv ${children}) # 获取相对路径
      list(APPEND subMsgList ${childrenRmTxtRel})
endforeach()

add_message_files(
	FILES
	${subMsgList}
)

file(GLOB SRV_LIST srv/*.srv)
file(GLOB hsclouddata_LIST srv/hsclouddata/*.srv)
file(GLOB force_bridge_LIST srv/force_bridge/*.srv)
list(APPEND SRV_LIST ${hsclouddata_LIST} ${force_bridge_LIST})
set(subSrvList "")
# string(REPLACE "/CMakeLists.txt" "/" childrenRmTxt ${children})
foreach(children ${SRV_LIST})
      file(RELATIVE_PATH childrenRmTxtRel ${CMAKE_CURRENT_SOURCE_DIR}/srv ${children}) # 获取相对路径
      list(APPEND subSrvList ${childrenRmTxtRel})
endforeach()



add_service_files(
  FILES
  ${subSrvList}
 # ${hsclouddata_SRV_LIST}
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  shape_msgs
  moveit_msgs
)
catkin_package(
  CATKIN_DEPENDS  
  std_msgs 
  geometry_msgs 
  trajectory_msgs
)


