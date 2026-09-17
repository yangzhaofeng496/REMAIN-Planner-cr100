cmake_minimum_required(VERSION 3.0.2)
project(ir100_goal_adapter)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs geometry_msgs roscpp std_msgs std_srvs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ir100_goal_adapter_node src/ir100_goal_adapter_node.cpp)
target_compile_features(ir100_goal_adapter_node PRIVATE cxx_std_14)
target_link_libraries(ir100_goal_adapter_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(target_validator_test test/target_validator_test.cpp)
  if(TARGET target_validator_test)
    target_link_libraries(target_validator_test ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ir100_goal_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
