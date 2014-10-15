cmake_minimum_required(VERSION 2.8.3)
project(libbarrett_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Barrett REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  wam_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${BARRETT_LIB_DIRS})
include_directories(${BARRETT_INCLUDE_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

add_executable(barrett_arm src/barrett_arm.cpp)
add_dependencies(barrett_arm wam_msgs_gencpp)
target_link_libraries(barrett_arm ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hand src/hand.cpp)
add_dependencies(hand wam_msgs_gencpp)
target_link_libraries(hand ${BARRETT_LIBRARIES} ${catkin_LIBRARIES})

