//
// Created by tony on 9/29/24.
//

#include "depth_camera_runtime_param.h"

// ******************************  DepthCameraRuntimeParam类的公有函数  ******************************

// 构造函数
DepthCameraRuntimeParam::DepthCameraRuntimeParam():
        IsOnline(true),
        IsRecordVideo(false),
        UpdateDataCpuCore(-1),
        RecordVideoCpuCore(-1),
        RecordVideoFps(20.0),
        OfflineVideoName(),
        RecordVideoPath()
{
}