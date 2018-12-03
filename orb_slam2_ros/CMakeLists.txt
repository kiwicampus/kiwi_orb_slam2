cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

ADD_DEFINITIONS("-std=c++11")

find_package(OpenCV 3.1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  orb_slam2_lib
  roscpp
  tf
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  pcl_conversions
  octomap_ros
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ORBState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
 #  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs
   DEPENDS message_runtime
 #  DEPENDS system_lib
)

include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/include
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} 
  ./src
)

LINK_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/DBoW2/lib
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/Thirdparty/g2o/lib
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/build/devel/lib
)

set(LIBS
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  orb_slam2_lib
)

# Node for monocular camera
add_executable(Mono
  src/ros_mono.cc
  src/ROSPublisher.cpp
)

target_link_libraries(Mono
  ${catkin_LIBRARIES}
  ${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
#add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)
#
#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
add_executable(Stereo
  src/ros_stereo.cc
  src/ROSPublisher.cpp
)

target_link_libraries(Stereo
  ${catkin_LIBRARIES}
  ${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
  src/ros_rgbd.cc
  src/ROSPublisher.cpp
)

target_link_libraries(RGBD
  ${catkin_LIBRARIES}
  ${LIBS}
)
