#include "sdk/api.h"
#include <iostream>
#include <unistd.h>
#include <sys/time.h>

#include <mutex>
#include <deque>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        ROS_INFO("Caught SIGINT, shutting down node.");
        ros::shutdown();
        exit(0);
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test_sensor_array");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到"/depth_map"话题，消息类型为sensor_msgs/Image
    ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("/depth_map", 1);

    std::signal(SIGINT, signalHandler);

    // 初始化设备ID连接时间等变量， 并初始化SDK
    int DevID = 0;
    STRC_IMG_ALL *pImg;
    char *pInfo = NULL;

    api_init();

    cout << "SDK VER:" << api_get_sdk_ver() << endl;

    //connect
    DevID = api_connect((char *)"192.168.10.100" );
    cout << "DevID:" << DevID<< endl;
    if(DevID < 0)
    {
        cout << "api_connect failed." << endl;
        return -1;
    }
    sleep(1);
    pInfo = api_get_dev_ver(DevID);

    if(pInfo == NULL)
    {
        cout << "get Sdk version failed." << endl;
    }
    else
    {
        cout << "SDK VER:" << pInfo << endl;
    }

    pInfo = api_get_intrinsic_parameters(DevID);

    if(pInfo == NULL)
    {
        cout << "get lens failed." << endl;
    }
    else
    {
        cout << "lens info:" << pInfo << endl;

    }

    ros::Rate loop_rate(10);

    // 定义图像的宽度和高度
    int width = 640;
    int height = 480;

    // 创建一个640x480的深度图，数据类型为CV_32F（32位浮点数）
    cv::Mat depthMap(height, width, CV_32F, cv::Scalar(0));

    // 创建一个窗口
    const char* windowName = "Depth Map";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    while(ros::ok())
    {
        pImg = api_get_img(DevID);

        if(pImg != NULL)
        {
            // 给每个像素赋值，这里以赋值为它的索引位置为例
            for (int i = 0; i < height; ++i)
            {
                for (int j = 0; j < width; ++j)
                {
                    float& pixel = depthMap.at<float>(i, j);
                    pixel = static_cast<float>(pImg -> img_depth.data[i * width + j]);
                }
            }

            // 转换OpenCV图像到ROS图像消息
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthMap).toImageMsg();

            // 发布深度图
            depth_pub.publish(depth_msg);
        }

        loop_rate.sleep();
    }

    api_disconnect(DevID);

    api_exit();

    return 0;
}
