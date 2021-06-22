cmake_minimum_required(VERSION 3.0.2)
project(bitcq_camera)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -g -Wunused-function")

#add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

find_package(OpenCV 3.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/3rdparty/GMSL_Camera/
  ${PROJECT_SOURCE_DIR}/3rdparty/Galaxy_Camera/
  ${PROJECT_SOURCE_DIR}/3rdparty/Rtsp_Camera/
)

add_subdirectory(3rdparty/GMSL_Camera)
add_subdirectory(3rdparty/Galaxy_Camera)
add_subdirectory(3rdparty/Rtsp_Camera)

add_library(${PROJECT_NAME} src/bitcq_camera.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  GMSL_Camera
  Galaxy_Camera
  Rtsp_Camera
)

add_executable(${PROJECT_NAME}_node nodes/bitcq_camera_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
