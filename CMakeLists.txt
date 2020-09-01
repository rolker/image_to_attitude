cmake_minimum_required(VERSION 2.8.3)
project(image_to_attitude)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport    
    marine_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
   CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (SOURCES
    src/image_to_attitude.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

