cmake_minimum_required(VERSION 2.8.3)
project(hc_camera_interface)
# if(COMMAND cmake_policy)
#   cmake_policy(SET CMP0003 NEW)
# endif(COMMAND cmake_policy)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_FLAGS "-std=c++14 -O2 -Wall -g ${CMAKE_CXX_FLAGS}")

set(HC_CAMERA_INTERFACE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)
link_directories(${HC_CAMERA_INTERFACE_PATH}/hc_sdk/lib)
link_directories(${HC_CAMERA_INTERFACE_PATH}/hc_sdk/lib/HCNetSDKCom)

catkin_package(
  INCLUDE_DIRS .   ${OpenCV_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
  DEPENDS     libhcnetsdk.so libHCCore.so libPlayCtrl.so libHCCoreDevCfg.so libHCPreview.so libSystemTransform.so
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${HC_CAMERA_INTERFACE_PATH}/hc_camera_interface
  ${HC_CAMERA_INTERFACE_PATH}/hc_camera_interface_node
  ${HC_CAMERA_INTERFACE_PATH}/hc_sdk/include 
)

FILE(GLOB CAMERA_INTERFACE_SRCS
  ${HC_CAMERA_INTERFACE_PATH}/hc_camera_interface/hc_camera_interface.cpp
  ${HC_CAMERA_INTERFACE_PATH}/hc_camera_interface_node/hc_camera_interface_node.cpp
)
FILE(GLOB MODLE_BIN_SRCS
  ${HC_CAMERA_INTERFACE_PATH}/hc_camera_interface_node//main.cpp
)

ADD_LIBRARY(hc_camera_interface_api SHARED
    ${CAMERA_INTERFACE_SRCS}
)
target_link_libraries(hc_camera_interface_api
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
    libhcnetsdk.so
    libHCCore.so   
    libPlayCtrl.so
    libHCCoreDevCfg.so
    libHCPreview.so
    libSystemTransform.so        
)

add_executable(${PROJECT_NAME} ${MODLE_BIN_SRCS})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  hc_camera_interface_api     
)