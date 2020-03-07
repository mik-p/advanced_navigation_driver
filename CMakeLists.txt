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
)

include_directories(
  ansdk
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include ansdk
  CATKIN_DEPENDS diagnostic_msgs nav_msgs roscpp sensor_msgs std_msgs tf
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
# add_library(${PROJECT_NAME}
# )

# target_link_libraries(${PROJECT_NAME}
#   ${PROJECT_NAME}_sdk
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# make executables
add_executable(${PROJECT_NAME}_node
 src/an_driver.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_sdk
  # ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
