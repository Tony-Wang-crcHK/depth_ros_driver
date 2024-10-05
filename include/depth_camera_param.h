//
// Created by tony on 9/29/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_PARAM_H
#define WORKSPACE_DEPTH_CAMERA_PARAM_H

#include <vector>
#include <string>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "depth_camera_runtime_param.h"
#include "depth_camera_model_param.h"
#include "depth_camera_hardware_param.h"

namespace DepthRosDriver
{
/**
 * @brief 华睿相机参数
 */
    class DepthCameraParam {
    public:
        std::string IP;                                           ///< 相机的IP
        DepthCameraRuntimeParam RuntimeParam;                     ///< 相机的运行时参数
        DepthCameraModelParam ModelParam;                         ///< 相机的模型参数
        DepthCameraHardwareParam HardwareParams;                  ///< 相机的硬件参数集合

        /**
         * @brief 构造函数
         */
        DepthCameraParam();

        /**
         * @brief 析构函数
         */
        ~DepthCameraParam() = default;

        /**
         * @brief 从yaml配置文件中加载相机参数
         * @param[in]  yamlFileName         相机参数配置文件名
         * @param[out] depthCameraParam     相机参数
         * @return 加载结果\n
         *         -<em>false</em> 加载失败\n
         *         -<em>true</em> 加载成功\n
         */
        static bool LoadFromYamlFile(const std::string &yamlFileName, DepthCameraParam *depthCameraParam);
    };
}

#endif //WORKSPACE_DEPTH_CAMERA_PARAM_H
