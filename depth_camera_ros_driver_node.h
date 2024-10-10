//
// Created by tony on 10/5/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_ROS_DRIVER_H
#define WORKSPACE_DEPTH_CAMERA_ROS_DRIVER_H

#include <string>
#include <filesystem>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>

#include "depth_camera.h"

namespace DepthRosDriver
{
    class DepthCameraRosDriver {

    public:

        /**
         * @brief 构造函数
         */
        DepthCameraRosDriver();

        /**
         * @brief 析构函数
         */
        ~DepthCameraRosDriver();

        /**
         * @brief signal句柄函数，可打断ROS节点
         */
        static void SignalHandler(int signal);

    private:

        /**
         * @brief 相机系统的运行函数
         */
        bool Run();

        ros::NodeHandle nh_;                                                                                       ///< ROS节点句柄

        ros::Publisher depthPub_;                                                                                  ///< ROS相机消息发布器

        DepthCamera camera_;                                                                                       ///< 深度相机类实例化

        DepthCameraParam cameraParam_;                                                                             ///< 相机参数类的实例化

        std::string packagePath_;                                                                                  ///< ROS的全局路径
    };
}

#endif //WORKSPACE_DEPTH_CAMERA_ROS_DRIVER_H
