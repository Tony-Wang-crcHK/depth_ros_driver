//
// Created by tony on 9/29/24.
//

#include "depth_camera_param.h"


// ******************************  DepthCameraParam类的公有函数  ******************************

// 构造函数
DepthCameraParam::DepthCameraParam():
        IP("192.168.10.100"),
        RuntimeParam(),
        ModelParam(),
        HardwareParams()
{
}

// 从yaml配置文件中加载相机参数
bool DepthCameraParam::LoadFromYamlFile(const std::string &yamlFileName, DepthCameraParam *depthCameraParam)
{
    // 判断yaml配置文件是否存在
    if (::access(yamlFileName.c_str(), F_OK) == -1)
    {
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        return false;
    }

    // 读取IP参数
    if ((!fileStorage["IP"].isNone()) && (fileStorage["IP"].isString()))
    {
        depthCameraParam -> IP = static_cast<std::string>(fileStorage["IP"]);
    }

    // 读取相机的运行时参数
    cv::FileNode runtimeParamNode = fileStorage["RuntimeParam"];
    if (!runtimeParamNode.empty())
    {
        // 读取IsOnline参数
        if ((!runtimeParamNode["IsOnline"].isNone()) && (runtimeParamNode["IsOnline"].isInt()))
        {
            depthCameraParam->RuntimeParam.IsOnline = static_cast<int>(runtimeParamNode["IsOnline"]);
        }

        // 读取IsRecordVideo参数
        if ((!runtimeParamNode["IsRecordVideo"].isNone()) && (runtimeParamNode["IsRecordVideo"].isInt()))
        {
            depthCameraParam->RuntimeParam.IsRecordVideo = static_cast<int>(runtimeParamNode["IsRecordVideo"]);
        }

        // 读取UpdateDataCpuCore参数
        if ((!runtimeParamNode["UpdateDataCpuCore"].isNone()) && (runtimeParamNode["UpdateDataCpuCore"].isInt()))
        {
            depthCameraParam->RuntimeParam.UpdateDataCpuCore = static_cast<int>(runtimeParamNode["UpdateDataCpuCore"]);
        }

        // 读取RecordVideoCpuCore参数
        if ((!runtimeParamNode["RecordVideoCpuCore"].isNone()) && (runtimeParamNode["RecordVideoCpuCore"].isInt()))
        {
            depthCameraParam->RuntimeParam.RecordVideoCpuCore = static_cast<int>(runtimeParamNode["RecordVideoCpuCore"]);
        }

        // 读取RecordVideoFps参数
        if ((!runtimeParamNode["RecordVideoFps"].isNone()) && (runtimeParamNode["RecordVideoFps"].isReal()))
        {
            depthCameraParam->RuntimeParam.RecordVideoFps = static_cast<double>(runtimeParamNode["RecordVideoFps"]);
        }

        // 读取OfflineVideoName参数
        if ((!runtimeParamNode["OfflineVideoName"].isNone()) && (runtimeParamNode["OfflineVideoName"].isString()))
        {
            depthCameraParam->RuntimeParam.OfflineVideoName = static_cast<std::string>(runtimeParamNode["OfflineVideoName"]);
        }

        // 读取RecordVideoPath参数
        if ((!runtimeParamNode["RecordVideoPath"].isNone()) && (runtimeParamNode["RecordVideoPath"].isString()))
        {
            depthCameraParam->RuntimeParam.RecordVideoPath = static_cast<std::string>(runtimeParamNode["RecordVideoPath"]);
        }
    }

    // 读取相机的模型参数
    cv::FileNode modelParamNode = fileStorage["ModelParam"];
    if (!modelParamNode.empty())
    {
        // 读取CvIntrinsics参数
        if ((!modelParamNode["CvIntrinsics"].isNone()) && (modelParamNode["CvIntrinsics"].isMap()))
        {
            modelParamNode["CvIntrinsics"] >> depthCameraParam->ModelParam.CvIntrinsics;
            depthCameraParam->ModelParam.EigenIntrinsics.resize(depthCameraParam->ModelParam.CvIntrinsics.rows,
                                                                depthCameraParam->ModelParam.CvIntrinsics.cols);
            cv::cv2eigen(depthCameraParam->ModelParam.CvIntrinsics, depthCameraParam->ModelParam.EigenIntrinsics);
        }

        // 读取CvExtrinsics参数
        if ((!modelParamNode["CvExtrinsics"].isNone()) && (modelParamNode["CvExtrinsics"].isMap()))
        {
            modelParamNode["CvExtrinsics"] >> depthCameraParam->ModelParam.CvExtrinsics;
            cv::cv2eigen(depthCameraParam->ModelParam.CvExtrinsics,
                         depthCameraParam->ModelParam.EigenExtrinsics.matrix());
        }

        // 读取CvDistortions参数
        if ((!modelParamNode["CvDistortions"].isNone()) && (modelParamNode["CvDistortions"].isMap()))
        {
            modelParamNode["CvDistortions"] >> depthCameraParam->ModelParam.CvDistortions;
            cv::cv2eigen(depthCameraParam->ModelParam.CvDistortions,
                         depthCameraParam->ModelParam.EigenDistortions);
        }
    }

    // 读取相机的硬件参数
    cv::FileNode hardwareParamsNode = fileStorage["HardwareParams"];
    if (!hardwareParamsNode.empty())
    {
        // 创建硬件参数
        DepthCameraHardwareParam hardwareParam;

        // 读取IsBalanceWhiteAuto参数
        if ((!hardwareParamsNode["IsBalanceWhiteAuto"].isNone()) && (hardwareParamsNode["IsBalanceWhiteAuto"].isInt()))
        {
            hardwareParam.IsBalanceWhiteAuto = static_cast<int>(hardwareParamsNode["IsBalanceWhiteAuto"]);
        }

        // 读取IsExposureAuto参数
        if ((!hardwareParamsNode["IsExposureAuto"].isNone()) && (hardwareParamsNode["IsExposureAuto"].isInt()))
        {
            hardwareParam.IsExposureAuto = static_cast<int>(hardwareParamsNode["IsExposureAuto"]);
        }

        // 读取ExposureTime参数
        if ((!hardwareParamsNode["ExposureTime"].isNone()) && (hardwareParamsNode["ExposureTime"].isReal()))
        {
            hardwareParam.ExposureTime = static_cast<float>(hardwareParamsNode["ExposureTime"]);
        }

        // 读取GainRaw参数
        if ((!hardwareParamsNode["GainRaw"].isNone()) && (hardwareParamsNode["GainRaw"].isReal()))
        {
            hardwareParam.GainRaw = static_cast<float>(hardwareParamsNode["GainRaw"]);
        }

        // 读取Gamma参数
        if ((!hardwareParamsNode["Gamma"].isNone()) && (hardwareParamsNode["Gamma"].isReal()))
        {
            hardwareParam.Gamma = static_cast<float>(hardwareParamsNode["Gamma"]);
        }

        // 读取Brightness参数
        if ((!hardwareParamsNode["Brightness"].isNone()) && (hardwareParamsNode["Brightness"].isInt()))
        {
            hardwareParam.Brightness = static_cast<int>(hardwareParamsNode["Brightness"]);
        }

        // 保存硬件参数
        depthCameraParam -> HardwareParams = hardwareParam;
    }

    // 关闭文件存储器
    fileStorage.release();

    // 返回加载结果
    return true;
}