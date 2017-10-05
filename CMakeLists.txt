cmake_minimum_required(VERSION 3.0)

set(PROJ_NAME actKniect)
project(${PROJ_NAME})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

aux_source_directory(src SOURCE_DIR)

add_executable(${PROJ_NAME} ${SOURCE_DIR})

#OpenCV
find_package(OpenCV REQUIRED)

IF(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS})
endif()

#Kinect V2
find_package(KinectSDK2 REQUIRED)

if(KinectSDK2_FOUND)
  include_directories(${KinectSDK2_INCLUDE_DIRS})
  link_directories(${KinectSDK2_LIBRARY_DIRS})
  target_link_libraries(${PROJ_NAME} ${KinectSDK2_LIBRARIES})
  add_custom_command(TARGET ${PROJ_NAME} POST_BUILD ${KinectSDK2_COMMANDS})
endif()
