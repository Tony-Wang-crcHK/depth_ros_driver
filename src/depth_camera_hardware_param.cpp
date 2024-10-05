//
// Created by tony on 9/29/24.
//

#include "depth_camera_hardware_param.h"

namespace DepthRosDriver {
// ******************************  DepthCameraHardwareParam类的公有函数  ******************************

// 构造函数
    DepthCameraHardwareParam::DepthCameraHardwareParam() :
            IsBalanceWhiteAutoRGB(true),
            IsExposureAutoRGB(false),
            ExposureTimeRGB(1500.0),
            GainRawRGB(1.0),
            GammaRGB(0.8),
            BrightnessRGB(40),
            MinBrightessDepth(40),
            MaxBrightessDepth(1000),
            FlayingPixThresDepth(),
            IsExposureAutoDepth(false),
            ExposureTimeDepth(),
            GainRawDepth() {
    }
}