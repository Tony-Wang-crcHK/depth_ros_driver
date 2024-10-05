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

        DepthCameraRosDriver();

        ~DepthCameraRosDriver();

        static void SignalHandler(int signal);

    private:

        bool Run();

        ros::NodeHandle nh_;

        ros::Publisher depthPub_;

        DepthCamera camera_;

        DepthCameraParam cameraParam_;

        std::string packagePath_;
    };

}

#endif //WORKSPACE_DEPTH_CAMERA_ROS_DRIVER_H
