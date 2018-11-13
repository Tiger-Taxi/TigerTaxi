cmake_minimum_required(VERSION 2.8.3)
project(camera_pub)

set(CMAKE_CXX_STANDARD 11)

# Uncomment to debug safezone:
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-2")

# External Packages
find_package(OpenCV REQUIRED)

# ROS Packages 
find_package(catkin REQUIRED roscpp std_msgs sensor_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS image_transport cv_bridge)

# ROS Messages
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_pub main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(camera_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(camera_pub
  ${catkin_LIBRARIES}
)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_pub ${OpenCV_LIBS})
catkin_package()
