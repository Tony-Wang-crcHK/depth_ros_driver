//
// Created by tony on 9/29/24.
//

#include "depth_camera_param.h"

int main(int argc, char *argv[])
{
    // 从yaml文件中加载相机参数
    DepthCameraParam param;
    DepthCameraParam::LoadFromYamlFile("/home/tony/workspace/src/depth_ros_driver/config/camera.yaml",
                                        &param);

    std::cout<<"IP: "<<param.IP<<std::endl;

    std::cout<<"RuntimeParam: "<<param.RuntimeParam.IsOnline<<", " \
                               <<param.RuntimeParam.IsRecordVideo<<", " \
                               <<param.RuntimeParam.UpdateDataCpuCore<<", " \
                               <<param.RuntimeParam.RecordVideoCpuCore<<", " \
                               <<param.RuntimeParam.RecordVideoFps<<", " \
                               <<param.RuntimeParam.OfflineVideoName<<", " \
                               <<param.RuntimeParam.RecordVideoPath<<", " \
                               <<std::endl;

    std::cout<<"HardwareParams: "<<param.HardwareParams.IsBalanceWhiteAuto<<", " \
                               <<param.HardwareParams.IsExposureAuto<<", " \
                               <<param.HardwareParams.ExposureTime<<", " \
                               <<param.HardwareParams.GainRaw<<", " \
                               <<param.HardwareParams.Gamma<<", " \
                               <<param.HardwareParams.Brightness<<", " \
                               <<std::endl;
    return 0;
}