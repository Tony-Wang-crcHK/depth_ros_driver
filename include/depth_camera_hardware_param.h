//
// Created by tony on 9/29/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_HARDWARE_PARAM_H
#define WORKSPACE_DEPTH_CAMERA_HARDWARE_PARAM_H

#include <string>
#include <vector>

/**
 * @brief 相机的硬件参数
 */
class DepthCameraHardwareParam
{
public:

    bool IsBalanceWhiteAutoRGB;             ///< 彩色相机是否自动白平衡
    bool IsExposureAutoRGB;                 ///< 彩色相机是否自动曝光
    double ExposureTimeRGB;                 ///< 彩色相机的曝光时间
    double GainRawRGB;                      ///< 彩色相机的增益值
    double GammaRGB;                        ///< 彩色相机的伽马值
    int64_t  BrightnessRGB;                 ///< 彩色相机的亮度
    int64_t MinBrightessDepth;              ///< 深度相机最小有效亮度
    int64_t MaxBrightessDepth;              ///< 深度相机最大有效亮度
    int FlayingPixThresDepth;               ///< 深度相机飞行像相数阈值
    bool IsExposureAutoDepth;               ///< 深度相机是否自动曝光
    double ExposureTimeDepth;               ///< 深度相机的曝光时间
    double GainRawDepth;                    ///< 深度相机增益值

    /**
     * @brief 构造函数
     */
    DepthCameraHardwareParam();

    /**
     * @brief 析构函数
     */
    ~DepthCameraHardwareParam() = default;
};

#endif //WORKSPACE_DEPTH_CAMERA_HARDWARE_PARAM_H
