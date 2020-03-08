cmake_minimum_required(VERSION 2.8.3)
project(advanced_navigation_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  image_transport
  cv_bridge
  eigen_conversions
  message_generation
  message_runtime
)

find_package(Eigen3 REQUIRED)

include_directories(
  ansdk
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  EIGEN3_INCLUDE_DIR
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   InfoPanelData.msg
   InfoPanelError.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ansdk
  CATKIN_DEPENDS diagnostic_msgs nav_msgs roscpp sensor_msgs std_msgs tf message_generation message_runtime
)

# sdk library
add_library(${PROJECT_NAME}_sdk
  ansdk/an_packet_protocol.c
  ansdk/spatial_packets.c
  ansdk/rs232/rs232.c
)

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# project library
add_library(${PROJECT_NAME}
  src/an_driver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_sdk
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# make executables
add_executable(${PROJECT_NAME}_node
 src/an_driver_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  # ${PROJECT_NAME}_sdk
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# change baud rate
add_executable(an_change_baudrate
  src/an_change_baudrate.cpp
)

target_link_libraries(an_change_baudrate
  ${PROJECT_NAME}_sdk
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
