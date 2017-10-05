cmake_minimum_required(VERSION 3.0)

project(solution)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

aux_source_directory(src SOURCE_DIR)

#OpenCV
find_package(OpenCV REQUIRED)

IF(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_executable(project ${SOURCE_DIR})
  target_link_libraries(project ${OpenCV_LIBS})
endif()

#Kinect V2
find_package(KinectSDK2 REQUIRED)

if(KinectSDK2_FOUND)
  include_directories(${KinectSDK2_INCLUDE_DIRS})
  link_directories(${KinectSDK2_LIBRARY_DIRS})
  target_link_libraries(project ${KinectSDK2_LIBRARIES})
  add_custom_command(TARGET project POST_BUILD ${KinectSDK2_COMMANDS})
endif()
