cmake_minimum_required(VERSION 2.8.3)
project(vision_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(vision_node src/vision_node.cpp)

target_link_libraries(vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})