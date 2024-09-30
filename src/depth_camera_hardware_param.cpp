//
// Created by tony on 9/29/24.
//

#include "depth_camera_hardware_param.h"

// ******************************  DepthCameraHardwareParam类的公有函数  ******************************

// 构造函数
DepthCameraHardwareParam::DepthCameraHardwareParam():
        IsBalanceWhiteAuto(true),
        IsExposureAuto(false),
        ExposureTime(1500.0),
        GainRaw(1.0),
        Gamma(0.8),
        Brightness(40)
{
}