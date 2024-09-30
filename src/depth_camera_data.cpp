//
// Created by tony on 9/29/24.
//

#include "depth_camera_data.h"

// ******************************  DepthCameraData类的公有函数  ******************************

// 构造函数
DepthCameraData::DepthCameraData():
        Index(0),
        Timestamp(0),
        RGBImage(cv::Mat::zeros(480, 640, CV_8UC3)),
        DepthImage(cv::Mat::zeros(480, 640, CV_32F))
{
}