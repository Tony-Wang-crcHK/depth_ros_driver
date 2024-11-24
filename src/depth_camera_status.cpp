//
// Created by tony on 9/29/24.
//

#include "depth_camera_status.h"

namespace DepthRosDriver {
// ******************************  DepthCameraStatus类的公有函数  ******************************

// 构造函数
    DepthCameraStatus::DepthCameraStatus() :
            IP(),
            IsConnected(false),
            IsWorking(false),
            ErrorFrame(0),
            LostPacketFrame(0),
            TotalFrame(0),
            BandWidth(0.0),
            FPS(0.0),
            EigenIntrinsics(),
            EigenDistortions(){
    }
}