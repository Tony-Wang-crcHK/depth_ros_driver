//
// Created by tony on 9/29/24.
//

#include "depth_camera_param.h"

namespace DepthRosDriver {
// ******************************  DepthCameraParam类的公有函数  ******************************

// 构造函数
    DepthCameraParam::DepthCameraParam() :
            IP("192.168.10.100"),
            RuntimeParam(),
            ModelParam(),
            HardwareParams() {
    }

// 从yaml配置文件中加载相机参数
    bool DepthCameraParam::LoadFromYamlFile(const std::string &yamlFileName, DepthCameraParam *depthCameraParam) {
        // 判断yaml配置文件是否存在
        if (::access(yamlFileName.c_str(), F_OK) == -1) {
            return false;
        }

        // 判断yaml配置文件是否可读
        if (::access(yamlFileName.c_str(), R_OK) == -1) {
            return false;
        }

        // 创建并打开文件存储器
        cv::FileStorage fileStorage;
        if (!fileStorage.open(yamlFileName, cv::FileStorage::READ)) {
            return false;
        }

        // 读取IP参数
        if ((!fileStorage["IP"].isNone()) && (fileStorage["IP"].isString())) {
            depthCameraParam->IP = static_cast<std::string>(fileStorage["IP"]);
        }

        // 读取相机的运行时参数
        cv::FileNode runtimeParamNode = fileStorage["RuntimeParam"];
        if (!runtimeParamNode.empty()) {
            // 读取IsOnline参数
            if ((!runtimeParamNode["IsOnline"].isNone()) && (runtimeParamNode["IsOnline"].isInt())) {
                depthCameraParam->RuntimeParam.IsOnline = static_cast<int>(runtimeParamNode["IsOnline"]);
            }

            // 读取IsRecordVideo参数
            if ((!runtimeParamNode["IsRecordVideo"].isNone()) && (runtimeParamNode["IsRecordVideo"].isInt())) {
                depthCameraParam->RuntimeParam.IsRecordVideo = static_cast<int>(runtimeParamNode["IsRecordVideo"]);
            }

            // 读取UpdateDataCpuCore参数
            if ((!runtimeParamNode["UpdateDataCpuCore"].isNone()) && (runtimeParamNode["UpdateDataCpuCore"].isInt())) {
                depthCameraParam->RuntimeParam.UpdateDataCpuCore = static_cast<int>(runtimeParamNode["UpdateDataCpuCore"]);
            }

            // 读取RecordVideoCpuCore参数
            if ((!runtimeParamNode["RecordVideoCpuCore"].isNone()) &&
                (runtimeParamNode["RecordVideoCpuCore"].isInt())) {
                depthCameraParam->RuntimeParam.RecordVideoCpuCore = static_cast<int>(runtimeParamNode["RecordVideoCpuCore"]);
            }

            // 读取RecordVideoFps参数
            if ((!runtimeParamNode["RecordVideoFps"].isNone()) && (runtimeParamNode["RecordVideoFps"].isReal())) {
                depthCameraParam->RuntimeParam.RecordVideoFps = static_cast<double>(runtimeParamNode["RecordVideoFps"]);
            }

            // 读取OfflineVideoName参数
            if ((!runtimeParamNode["OfflineVideoName"].isNone()) && (runtimeParamNode["OfflineVideoName"].isString())) {
                depthCameraParam->RuntimeParam.OfflineVideoName = static_cast<std::string>(runtimeParamNode["OfflineVideoName"]);
            }

            // 读取RecordVideoPath参数
            if ((!runtimeParamNode["RecordVideoPath"].isNone()) && (runtimeParamNode["RecordVideoPath"].isString())) {
                depthCameraParam->RuntimeParam.RecordVideoPath = static_cast<std::string>(runtimeParamNode["RecordVideoPath"]);
            }
        }

        // 读取相机的模型参数
        cv::FileNode modelParamNode = fileStorage["ModelParam"];
        if (!modelParamNode.empty()) {
            // 读取CvIntrinsics参数
            if ((!modelParamNode["CvIntrinsics"].isNone()) && (modelParamNode["CvIntrinsics"].isMap())) {
                modelParamNode["CvIntrinsics"] >> depthCameraParam->ModelParam.CvIntrinsics;
                depthCameraParam->ModelParam.EigenIntrinsics.resize(depthCameraParam->ModelParam.CvIntrinsics.rows,
                                                                    depthCameraParam->ModelParam.CvIntrinsics.cols);
                cv::cv2eigen(depthCameraParam->ModelParam.CvIntrinsics, depthCameraParam->ModelParam.EigenIntrinsics);
            }

            // 读取CvExtrinsics参数
            if ((!modelParamNode["CvExtrinsics"].isNone()) && (modelParamNode["CvExtrinsics"].isMap())) {
                modelParamNode["CvExtrinsics"] >> depthCameraParam->ModelParam.CvExtrinsics;
                cv::cv2eigen(depthCameraParam->ModelParam.CvExtrinsics,
                             depthCameraParam->ModelParam.EigenExtrinsics.matrix());
            }

            // 读取CvDistortions参数
            if ((!modelParamNode["CvDistortions"].isNone()) && (modelParamNode["CvDistortions"].isMap())) {
                modelParamNode["CvDistortions"] >> depthCameraParam->ModelParam.CvDistortions;
                cv::cv2eigen(depthCameraParam->ModelParam.CvDistortions,
                             depthCameraParam->ModelParam.EigenDistortions);
            }
        }

        // 读取相机的硬件参数
        cv::FileNode hardwareParamsNode = fileStorage["HardwareParams"];
        if (!hardwareParamsNode.empty()) {
            // 读取IsBalanceWhiteAutoRGB参数
            if ((!hardwareParamsNode["IsBalanceWhiteAutoRGB"].isNone()) &&
                (hardwareParamsNode["IsBalanceWhiteAutoRGB"].isInt())) {
                depthCameraParam->HardwareParams.IsBalanceWhiteAutoRGB = static_cast<int>(hardwareParamsNode["IsBalanceWhiteAutoRGB"]);
            }

            // 读取IsExposureAutoRGB参数
            if ((!hardwareParamsNode["IsExposureAutoRGB"].isNone()) &&
                (hardwareParamsNode["IsExposureAutoRGB"].isInt())) {
                depthCameraParam->HardwareParams.IsExposureAutoRGB = static_cast<int>(hardwareParamsNode["IsExposureAutoRGB"]);
            }

            // 读取ExposureTimeRGB参数
            if ((!hardwareParamsNode["ExposureTimeRGB"].isNone()) && (hardwareParamsNode["ExposureTimeRGB"].isReal())) {
                depthCameraParam->HardwareParams.ExposureTimeRGB = static_cast<float>(hardwareParamsNode["ExposureTimeRGB"]);
            }

            // 读取GainRawRGB参数
            if ((!hardwareParamsNode["GainRawRGB"].isNone()) && (hardwareParamsNode["GainRawRGB"].isReal())) {
                depthCameraParam->HardwareParams.GainRawRGB = static_cast<float>(hardwareParamsNode["GainRawRGB"]);
            }

            // 读取GammaRGB参数
            if ((!hardwareParamsNode["GammaRGB"].isNone()) && (hardwareParamsNode["GammaRGB"].isReal())) {
                depthCameraParam->HardwareParams.GammaRGB = static_cast<float>(hardwareParamsNode["GammaRGB"]);
            }

            // 读取BrightnessRGB参数
            if ((!hardwareParamsNode["BrightnessRGB"].isNone()) && (hardwareParamsNode["BrightnessRGB"].isInt())) {
                depthCameraParam->HardwareParams.BrightnessRGB = static_cast<int>(hardwareParamsNode["BrightnessRGB"]);
            }

            // 读取MinBrightessDepth参数
            if ((!hardwareParamsNode["MinBrightessDepth"].isNone()) &&
                (hardwareParamsNode["MinBrightessDepth"].isInt())) {
                depthCameraParam->HardwareParams.MinBrightessDepth = static_cast<int>(hardwareParamsNode["MinBrightessDepth"]);
            }

            // 读取MaxBrightessDepth参数
            if ((!hardwareParamsNode["MaxBrightessDepth"].isNone()) &&
                (hardwareParamsNode["MaxBrightessDepth"].isInt())) {
                depthCameraParam->HardwareParams.MaxBrightessDepth = static_cast<int>(hardwareParamsNode["MaxBrightessDepth"]);
            }

            // 读取FlayingPixThresDepth参数
            if ((!hardwareParamsNode["FlayingPixThresDepth"].isNone()) &&
                (hardwareParamsNode["FlayingPixThresDepth"].isInt())) {
                depthCameraParam->HardwareParams.FlayingPixThresDepth = static_cast<int>(hardwareParamsNode["FlayingPixThresDepth"]);
            }

            // 读取IsExposureAutoDepth参数
            if ((!hardwareParamsNode["IsExposureAutoDepth"].isNone()) &&
                (hardwareParamsNode["IsExposureAutoDepth"].isInt())) {
                depthCameraParam->HardwareParams.IsExposureAutoDepth = static_cast<int>(hardwareParamsNode["IsExposureAutoDepth"]);
            }

            // 读取ExposureTimeDepth参数
            if ((!hardwareParamsNode["ExposureTimeDepth"].isNone()) &&
                (hardwareParamsNode["ExposureTimeDepth"].isReal())) {
                depthCameraParam->HardwareParams.ExposureTimeDepth = static_cast<float>(hardwareParamsNode["ExposureTimeDepth"]);
            }

            // 读取GainRawDepth参数
            if ((!hardwareParamsNode["GainRawDepth"].isNone()) && (hardwareParamsNode["GainRawDepth"].isReal())) {
                depthCameraParam->HardwareParams.GainRawDepth = static_cast<float>(hardwareParamsNode["GainRawDepth"]);
            }
        }

        // 关闭文件存储器
        fileStorage.release();

        // 返回加载结果
        return true;
    }
}