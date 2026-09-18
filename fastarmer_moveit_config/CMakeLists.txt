cmake_minimum_required(VERSION 3.0.2)
project(fastarmer_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
)

catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(kinematics_demo_node src/kinematics_demo.cpp)
target_link_libraries(kinematics_demo_node ${catkin_LIBRARIES})
add_dependencies(kinematics_demo_node ${catkin_EXPORTED_TARGETS})

install(TARGETS kinematics_demo_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(PythonInterp REQUIRED)
  enable_testing()
  add_test(
    NAME fastarmer_moveit_config_static
    COMMAND ${PYTHON_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_config.py
            ${CMAKE_CURRENT_SOURCE_DIR}/../remani_planner/mm_config/meshes/FastArmer/FastArmer.urdf
            ${CMAKE_CURRENT_SOURCE_DIR}/config/fastarmer.srdf)
endif()
