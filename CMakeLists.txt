cmake_minimum_required(VERSION 3.0.2)
project(joy_franka_control)


find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp sensor_msgs roslaunch)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/teleop_franka_joy)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(teleop_franka_node src/teleop_franka_node.cpp)
target_link_libraries(teleop_franka_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(teleop_franka_node
  PROPERTIES OUTPUT_NAME teleop_franka_node PREFIX "")

install(TARGETS ${PROJECT_NAME} teleop_franka_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
if (CATKIN_ENABLE_TESTING)

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/teleop_franka.launch)

  find_package(rostest REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS})

endif()
