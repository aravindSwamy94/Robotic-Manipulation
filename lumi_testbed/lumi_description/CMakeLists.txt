cmake_minimum_required(VERSION 2.8.3)
project(lumi_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED urdf xacro franka_description)

catkin_package()

file(GLOB xacro_files robots/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY textures DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
