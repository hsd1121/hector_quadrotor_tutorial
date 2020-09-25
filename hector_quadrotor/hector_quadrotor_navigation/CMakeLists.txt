cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_navigation)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    std_msgs 
    hector_uav_msgs
    actionlib
    actionlib_msgs
    message_generation
    geometry_msgs
    pcl_conversions
    pcl_ros
)

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)

## Generate messages in the 'msg' folder


#add_action_files(
#  DIRECTORY action
#  FILES Navigation.action
#)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs hector_uav_msgs actionlib_msgs geometry_msgs message_runtime)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(quadrotor_navigation src/quadrotor_navigation.cpp)

target_link_libraries(quadrotor_navigation ${catkin_LIBRARIES})