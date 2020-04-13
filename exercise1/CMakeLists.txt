cmake_minimum_required(VERSION 2.8.3)
project(exercise1)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS xacro
        moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(frame_publisher src/ros_intro.cpp)
target_link_libraries(frame_publisher ${catkin_LIBRARIES})
add_dependencies(frame_publisher ${catkin_EXPORTED_TARGETS})

