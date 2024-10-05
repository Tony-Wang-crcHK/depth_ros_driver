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
        cv_bridge
        std_msgs
        tf
        tf_conversions
        rosbag
        message_filters
        std_srvs)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    MESSAGE(STATUS "OpenCV_VERSION: " ${OpenCV_VERSION})
    MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
    MESSAGE(STATUS "OpenCV_LIBS: " ${OpenCV_LIBS})
endif()

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

set(CAMERA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/include/sdk/lib/librk3588_sdk.so)

add_library(camera SHARED
        include/depth_camera.h
        src/depth_camera.cpp
        src/depth_camera_data.cpp
        src/depth_camera_hardware_param.cpp
        src/depth_camera_model_param.cpp
        src/depth_camera_param.cpp
        src/depth_camera_runtime_param.cpp
        src/depth_camera_status.cpp)
target_link_libraries(camera
        ${CAMERA_LIB}
        ${EIGEN3_LIBRARIES}
        ${OpenCV_LIBS})

# 选择编译用的包
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
          message_filters std_srvs tf_conversions tf cv_bridge
          DEPENDS INCLUDE_DIRS include)

add_executable(test_ros_sdk
        test/test_ros_sdk.cpp)
target_link_libraries(test_ros_sdk
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${CAMERA_LIB})

add_executable(test_depth_camera_param
        test/test_depth_camera_param.cpp)
target_link_libraries(test_depth_camera_param
        camera)

add_executable(test_depth_camera
        test/test_depth_camera.cpp)
target_link_libraries(test_depth_camera
        camera)

add_executable(depth_camera_ros_driver
        depth_camera_ros_driver.h
        depth_camera_ros_driver.cpp)
target_link_libraries(depth_camera_ros_driver
        ${catkin_LIBRARIES}
        camera)