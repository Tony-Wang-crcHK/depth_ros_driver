//
// Created by tony on 9/29/24.
//

#include "depth_camera_model_param.h"

// ******************************  HuarayCameraModelParam类的公有函数  ******************************

// 构造函数
DepthCameraModelParam::DepthCameraModelParam():
        CvIntrinsics(),
        CvExtrinsics(),
        CvDistortions(),
        EigenIntrinsics(),
        EigenExtrinsics(),
        EigenDistortions()
{
}
