cmake_minimum_required(VERSION 2.8.3)
project(libbarrett_ros)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(BULLET bullet)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  wam_control
  message_generation
)

find_package(Barrett REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  message_runtime
  wam_msgs
  tf
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${BARRETT_LIB_DIRS})
include_directories(${BARRETT_INCLUDE_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

add_executable(${PROJECT_NAME} src/libbarrett_ros.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BARRETT_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

