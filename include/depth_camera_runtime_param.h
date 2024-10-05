//
// Created by tony on 9/29/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_RUNTIME_PARAM_H
#define WORKSPACE_DEPTH_CAMERA_RUNTIME_PARAM_H

#include <string>

namespace DepthRosDriver
{

   /**
    * @brief 相机的运行时参数
    */
    class DepthCameraRuntimeParam {
    public:
        bool IsOnline;                      ///< 是否在线运行
        bool IsRecordVideo;                 ///< 是否录制视频
        int UpdateDataCpuCore;              ///< 更新数据帧任务的CPU内核编号；默认为-1
        int RecordVideoCpuCore;             ///< 录制视频流任务的CPU内核编号；默认为-1
        double RecordVideoFps;              ///< 录制视频的帧率；默认为20.0
        std::string OfflineVideoName;       ///< 离线视频文件名称
        std::string RecordVideoPath;        ///< 录像文件存储路径

        /**
         * @brief 构造函数
         */
        DepthCameraRuntimeParam();

        /**
         * @brief 析构函数
         */
        ~DepthCameraRuntimeParam() = default;
    };
}

#endif //WORKSPACE_DEPTH_CAMERA_RUNTIME_PARAM_H
