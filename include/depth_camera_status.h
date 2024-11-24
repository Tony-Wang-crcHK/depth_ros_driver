//
// Created by tony on 9/29/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_STATUS_H
#define WORKSPACE_DEPTH_CAMERA_STATUS_H

#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace DepthRosDriver
{
/**
 * @brief 相机状态帧
 * @note 封装了相机的标识符、连接状态、工作状态和数据统计信息
 */
    class DepthCameraStatus {
    public:
        std::string IP;                 ///< 相机的标识符
        bool IsConnected;               ///< 相机的连接状态
        bool IsWorking;                 ///< 相机的工作状态
        unsigned int ErrorFrame;        ///< 相机的错误帧数统计
        unsigned int LostPacketFrame;   ///< 相机的丢包帧数统计
        unsigned int TotalFrame;        ///< 相机的总帧数统计
        double BandWidth;               ///< 相机的带宽统计
        double FPS;                     ///< 相机的帧率统计
        Eigen::Matrix3d EigenIntrinsics;                    ///< 相机Eigen格式的内参矩阵
        Eigen::Matrix<double, 5, 1> EigenDistortions;       ///< 相机Eigen格式的畸变系数向量

        /**
         * @brief 构造函数
         */
        DepthCameraStatus();

        /**
         * @brief 析构函数
         */
        ~DepthCameraStatus() = default;
    };
}

#endif //WORKSPACE_DEPTH_CAMERA_STATUS_H
