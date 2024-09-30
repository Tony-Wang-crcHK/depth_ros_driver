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

    bool IsBalanceWhiteAuto;            ///< 相机是否自动白平衡
    bool IsExposureAuto;                ///< 相机是否自动曝光
    double ExposureTime;                ///< 相机的曝光时间
    double GainRaw;                     ///< 相机的增益值
    double Gamma;                       ///< 相机的伽马值
    int64_t Brightness;                 ///< 相机的亮度

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
