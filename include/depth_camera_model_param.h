//
// Created by tony on 9/29/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_MODEL_PARAM_H
#define WORKSPACE_DEPTH_CAMERA_MODEL_PARAM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace DepthRosDriver
{

/**
 * @brief 相机的模型参数；包括内参矩阵、外参矩阵、畸变系数向量
 */
    class DepthCameraModelParam {
    public:
        cv::Mat CvIntrinsics;                               ///< 相机OpenCV格式的内参矩阵
        cv::Mat CvExtrinsics;                               ///< 相机OpenCV格式的外参矩阵
        cv::Mat CvDistortions;                              ///< 相机OpenCV格式的畸变系数向量
        Eigen::Matrix3d EigenIntrinsics;                    ///< 相机Eigen格式的内参矩阵
        Eigen::Isometry3d EigenExtrinsics;                  ///< 相机Eigen格式的外参矩阵
        Eigen::Matrix<double, 5, 1> EigenDistortions;       ///< 相机Eigen格式的畸变系数向量

        /**
        * @brief 构造函数
        */
        DepthCameraModelParam();

        /**
         * @brief 析构函数
         */
        ~DepthCameraModelParam() = default;
    };
}

#endif //WORKSPACE_DEPTH_CAMERA_MODEL_PARAM_H
