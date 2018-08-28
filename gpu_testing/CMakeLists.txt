cmake_minimum_required(VERSION 2.8)
project(gpu_testing)

# External Packages
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

add_executable(gpu_testing src/main.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(gpu_testing ${OpenCV_LIBS})