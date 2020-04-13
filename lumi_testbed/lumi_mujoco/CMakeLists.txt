cmake_minimum_required(VERSION 2.8.3)
project(lumi_mujoco)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib control_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gripper_action_to_trajectory_action src/gripper_action_to_trajectory_action.cpp)
target_link_libraries(gripper_action_to_trajectory_action ${catkin_LIBRARIES})
add_dependencies(gripper_action_to_trajectory_action ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS gripper_action_to_trajectory_action
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
