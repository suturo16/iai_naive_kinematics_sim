cmake_minimum_required(VERSION 2.8.3)
project(iai_naive_kinematics_sim)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  urdf
  sensor_msgs
  std_msgs
  resource_retriever
  )

find_path(yaml_cpp_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(yaml_cpp_LIBRARIES NAMES yaml-cpp)

add_message_files(DIRECTORY msg
  FILES
  ProjectionClock.msg)

add_service_files(DIRECTORY srv
  FILES
  SetJointState.srv)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_generation message_runtime urdf sensor_msgs std_msgs
  DEPENDS yaml_cpp
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS})

add_executable(simulator
  src/${PROJECT_NAME}/simulator_main.cpp src/${PROJECT_NAME}/expressions.cpp)
add_dependencies(simulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator
  ${catkin_LIBRARIES} yaml-cpp)

set(TEST_SRCS
  test/${PROJECT_NAME}/main.cpp
  test/${PROJECT_NAME}/simulator.cpp
  test/${PROJECT_NAME}/watchdog.cpp)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
