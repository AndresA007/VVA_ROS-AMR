cmake_minimum_required(VERSION 2.8.3)
project(vva_cliff_detector)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -O3
  -Werror
)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  image_geometry
  image_transport
  nodelet
  dynamic_reconfigure
  sensor_msgs
  depth_nav_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

# Dynamic reconfigure config
generate_dynamic_reconfigure_options(cfg/VVACliffDetector.cfg)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES VVACliffDetector VVACliffDetectorNode
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(VVACliffDetector src/vva_cliff_detector.cpp)
add_dependencies(VVACliffDetector ${PROJECT_NAME}_gencfg)
target_link_libraries(VVACliffDetector ${catkin_LIBRARIES})

add_library(VVACliffDetectorNode src/vva_cliff_detector_node.cpp)
add_dependencies(VVACliffDetectorNode ${PROJECT_NAME}_gencfg)
target_link_libraries(VVACliffDetectorNode VVACliffDetector ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(vva_cliff_detector src/main.cpp)
target_link_libraries(vva_cliff_detector VVACliffDetectorNode ${catkin_LIBRARIES})
