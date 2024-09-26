# 指定CMake版本
cmake_minimum_required(VERSION  3.10)

# 项目名称
project(depth_ros_driver)

# 指定C++版本
set(CMAKE_CXX_STANDARD 17)

# 设置编译模式为Release模式
SET(CMAKE_BUILD_TYPE "Release")

# 寻找ROS相关的包
find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        tf
        tf_conversions
        rosbag
        message_filters
        std_srvs)

include_directories(include
        ${catkin_INCLUDE_DIRS})

# version info
string( TIMESTAMP  COMPILE_TIME "%Y-%m-%d %H:%M:%S " )
add_definitions(-DCOMPILE_TIME="${COMPILE_TIME}")
add_definitions(-DVER="1.0")
site_name(host)
add_definitions(-DHOST="${host}")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb ")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall ")

add_library(depth_sdk SHARED
        include/sdk/api.cpp
        include/sdk/BuffShared.cpp
        include/sdk/nettransmission.cpp
        include/sdk/Thread.cpp
        include/sdk/TransferCmd.cpp)
target_link_libraries(depth_sdk pthread)

# set(CAMERA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/include/sdk/lib/librk3588_sdk.so)

# 选择编译用的包
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
          message_filters std_srvs tf_conversions tf
          DEPENDS INCLUDE_DIRS include)

add_executable(test_sdk
        test/main.cpp)
target_link_libraries(test_sdk
        ${catkin_LIBRARIES}
        depth_sdk)

