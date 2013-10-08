cmake_minimum_required(VERSION 2.8.3)
project(wifi_mapping)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs genmsg)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES wifi_measurement.msg)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES measurement_likelihood.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs)

###########
## Build ##
###########

ADD_SUBDIRECTORY(gaussian_process)
ADD_SUBDIRECTORY(wifi_scanner)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  wifi_scanner/include
)

## Declare a cpp executable
add_executable(wifi_scanning_node src/wifi_scanning_node.cpp)
target_link_libraries(wifi_scanning_node wifi_scanner ${catkin_LIBRARIES})

