cmake_minimum_required(VERSION 2.8.3)
project(ros_node_transformation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pluginlib
    std_msgs
    sensor_msgs
    tf
    diagnostic_updater
    serial
    ros_node_configuration
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_devices
    ${PROJECT_NAME}_device_publisher_plugins
  CATKIN_DEPENDS
    roscpp
    pluginlib
    std_msgs
    sensor_msgs
    tf
    diagnostic_updater
    serial
    ros_node_configuration
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_devices
  src/device/serial_device.cpp
)
target_link_libraries(${PROJECT_NAME}_devices ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_device_publisher_plugins
  src/device/publisher/imu_publisher.cpp
  src/device/publisher/range_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}_device_publisher_plugins ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_devices_node src/devices_node.cpp)
add_dependencies(${PROJECT_NAME}_devices_node ${PROJECT_NAME}_devices)
target_link_libraries(${PROJECT_NAME}_devices_node ${PROJECT_NAME}_devices ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_devices_node PROPERTIES OUTPUT_NAME devices_node PREFIX "")

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_devices
    ${PROJECT_NAME}_device_publisher_plugins
    ${PROJECT_NAME}_devices_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config include/${PROJECT_NAME} launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
