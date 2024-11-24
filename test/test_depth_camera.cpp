//
// Created by hkcrc-tony on 9/30/24.
//

#include <string>
#include <filesystem>
#include "depth_camera.h"

// 按ESC键，退出系统
// 按Enter键，暂停系统
// 按任意键，继续处理
int main(int argc, char *argv[])
{
    // 创建相机
    DepthRosDriver::DepthCamera camera;

    // 读取相机参数
    DepthRosDriver::DepthCameraParam cameraParam;
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string cameraYaml = currentPath.string() + "/src/depth_ros_driver/config/camera.yaml";
    if (!DepthRosDriver::DepthCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
    {
        std::cout<<"load fail"<<std::endl;
        return -1;
    }

    // 设置相机参数
    if (!camera.SetParam(cameraParam))
    {
        std::cout<<"set fail"<<std::endl;
        return -1;
    }

    // 初始化相机
    if (!camera.Init())
    {
        std::cout<<"init fail"<<std::endl;
        return -1;
    }

    // 打开相机
    if (!camera.Open())
    {
        std::cout<<"open fail"<<std::endl;
        return -1;
    }

    // 初始化显示窗体
    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("camera", 640, 480);

    // 初始化播放数据帧索引和上一帧图像的时间戳
    uint64_t frameIndex = 0;
    uint64_t previousFrameTimestamp = 0;

    // 播放在线视频
    while (true)
    {
        // 获取相机数据
        DepthRosDriver::DepthCameraData cameraData;
        camera.GetData(&cameraData);
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
        camera.GetStatus(&status);

        // 线程延迟，模拟图像处理的消耗
        std::this_thread::sleep_for(std::chrono::milliseconds(6));

//        // 输出播放数据帧索引和相机数据索引
//        std::cout << "**************************************************" << std::endl;
//        std::cout << "frameIndex: " << frameIndex << std::endl;
//        std::cout << "cameraDataIndex: " << cameraData.Index << std::endl;
//        std::cout << "**************************************************" << std::endl;

        // 播放视频
        cv::imshow("camera", cameraData.DepthImage);

        // 读取按键的ASCII码；注意：cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(10);

        // 如果按下ESC键，退出系统
        if (keyValue == 27)
        {
            break;
        }

        // 如果按下Enter键，暂停系统
        if (keyValue == 13)
        {
            cv::waitKey(0);
        }
    }

    // 关闭相机
    camera.Close();

    // 清理相机资源
    camera.Release();

    return 0;
}