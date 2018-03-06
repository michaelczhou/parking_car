cmake_minimum_required(VERSION 2.8.3)
project(parking_car)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Add support for C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})


include_directories(${catkin_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR}/include)

add_executable(ndt_mapping_2d ${PROJECT_SOURCE_DIR}/test/ndt_mapping_2d.cpp)
target_link_libraries(ndt_mapping_2d ${catkin_LIBRARIES})

add_executable(loop_mapping ${PROJECT_SOURCE_DIR}/test/loop_mapping.cpp )
target_link_libraries(loop_mapping ${catkin_LIBRARIES})

add_executable(loop_mapping2 ${PROJECT_SOURCE_DIR}/test/loop_mapping2.cpp)
target_link_libraries(loop_mapping2 ${catkin_LIBRARIES} ceres)
include_directories(include ${catkin_INCLUDE_DIRS})
