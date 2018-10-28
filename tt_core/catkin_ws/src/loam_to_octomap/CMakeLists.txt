cmake_minimum_required(VERSION 2.8.3)
project(loam_to_octomap)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  tf2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL #OCTOMAP
)


link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OCTOMAP_DEFINITIONS})
add_definitions( -march=native )


add_executable(loam_to_octomap src/loam_to_octomap.cpp)
target_link_libraries(loam_to_octomap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

