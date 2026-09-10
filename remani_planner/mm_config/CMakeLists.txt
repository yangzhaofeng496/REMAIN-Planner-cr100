cmake_minimum_required(VERSION 2.8.3)
project(mm_config)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  visualization_msgs
  plan_env
  urdf
  kdl_parser
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mm_config 
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

#add_message_files{
#    FILES
#    Assignment.msg
#}

add_library( mm_config
    src/mm_config.cpp
    src/urdf_collision_model.cpp
    )

find_package(assimp REQUIRED)

target_link_libraries( mm_config
    ${catkin_LIBRARIES} 
    assimp
    ${catkin_LIBRARIES}
    ) 
