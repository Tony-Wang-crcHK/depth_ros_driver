//
// Created by tony on 10/5/24.
//

#include "depth_camera_ros_driver_node.h"

namespace DepthRosDriver
{
    DepthCameraRosDriver::DepthCameraRosDriver():nh_(),
                                                 depthPub_(),
                                                 packagePath_() {

        depthPub_ = nh_.advertise<sensor_msgs::Image>("/depth_map", 1);

        if(!Run())
        {
            std::cout<<"run error"<<std::endl;
        }
    }

    DepthCameraRosDriver::~DepthCameraRosDriver()
    {
        // 关闭相机
        camera_.Close();

        // 清理相机资源
        camera_.Release();
    }

    void DepthCameraRosDriver::SignalHandler(int signal)
    {
        if (signal == SIGINT)
        {
            ROS_INFO("Caught SIGINT, shutting down node.");
            ros::shutdown();
            exit(0);
        }
    }

    bool DepthCameraRosDriver::Run()
    {
        packagePath_ = ros::package::getPath("depth_ros_driver");
        std::string cameraYaml = packagePath_ + "/config/camera.yaml";
        if (!DepthCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam_))
        {
            return false;
        }

        // 设置相机参数
        if (!camera_.SetParam(cameraParam_))
        {
            return false;
        }

        // 初始化相机
        if (!camera_.Init())
        {
            return false;
        }

        // 打开相机
        if (!camera_.Open())
        {
            return false;
        }

        // 初始化播放数据帧索引和上一帧图像的时间戳
        uint64_t frameIndex = 0;
        uint64_t previousFrameTimestamp = 0;

        // 播放在线视频
        while (ros::ok())
        {
            // 获取相机数据
            DepthRosDriver::DepthCameraData cameraData;
            camera_.GetData(&cameraData);
            if (cameraData.Timestamp > previousFrameTimestamp)
            {
                previousFrameTimestamp = cameraData.Timestamp;
                frameIndex++;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            DepthRosDriver::DepthCameraStatus status;
            camera_.GetStatus(&status);

            sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", cameraData.DepthImage).toImageMsg();

            depthPub_.publish(depthMsg);

        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_ros_driver");
    DepthRosDriver::DepthCameraRosDriver depthCameraRosDriver;
   std::signal(SIGINT, DepthRosDriver::DepthCameraRosDriver::SignalHandler);
    ros::spin();
    return 0;
}