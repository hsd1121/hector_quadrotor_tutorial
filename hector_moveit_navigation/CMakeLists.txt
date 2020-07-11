cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_navigation)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    hector_uav_msgs
    hector_moveit_actions
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

add_action_files(
  DIRECTORY action
  FILES Navigation.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs hector_uav_msgs hector_moveit_actions actionlib_msgs geometry_msgs message_runtime)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(hector_navigator src/Navigator.cpp src/navigator_node.cpp)
add_executable(navigator_client src/navigator_client.cpp)
add_executable(path_distance_test src/path_distance_test.cpp)
# add_executable(pipeline_test src/pipeline_test.cpp)

target_compile_options(hector_navigator PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})

target_link_libraries(hector_navigator ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})
target_link_libraries(navigator_client ${catkin_LIBRARIES})
target_link_libraries(path_distance_test ${catkin_LIBRARIES})
# target_link_libraries(pipeline_test ${catkin_LIBRARIES})

add_dependencies(hector_navigator hector_uav_msgs_generate_messages hector_moveit_actions_generate_messages)
add_dependencies(navigator_client ${hector_moveit_navigation_EXPORTED_TARGETS})
add_dependencies(path_distance_test ${hector_moveit_navigation_EXPORTED_TARGETS})
# add_dependencies(pipeline_test ${hector_moveit_navigation_EXPORTED_TARGETS})