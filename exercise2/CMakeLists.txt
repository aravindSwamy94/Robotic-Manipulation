cmake_minimum_required(VERSION 2.8.3)
project(exercise2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS xacro
        moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package()

add_executable(plan src/plan.cpp)
target_link_libraries(plan ${catkin_LIBRARIES})
add_dependencies(plan ${catkin_EXPORTED_TARGETS})

install(TARGETS plan 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

file(GLOB xacro_files model/robots/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

