//
// Created by tony on 9/29/24.
//

#ifndef WORKSPACE_DEPTH_CAMERA_H
#define WORKSPACE_DEPTH_CAMERA_H

#include <string>
#include <chrono>
#include <thread>
#include <sched.h>
#include <sys/prctl.h>
#include <sys/sysinfo.h>
#include <shared_mutex>
#include <opencv2/opencv.hpp>
#include "sdk/api.h"
#include "depth_camera_data.h"
#include "depth_camera_status.h"
#include "depth_camera_param.h"

/**
 * @brief 基于指数星空驱动程序进行封装\n
 * @note 使用步骤如下：\n
 *       Step1：实例化DepthCameraa\n
 *       Step2：自行构造DepthCameraParam或从yaml文件中读取DepthCameraParam\n
 *       Step3：调用SetParam()函数设置相机参数\n
 *       Step4：调用Init()函数初始化相机\n
 *       Step5：调用Open()函数打开相机\n
 *       Step6：调用GetData()函数读取相机数据\n
 *       Step7：调用GetStatus()函数读取相机状态\n
 *       Step8：调用Close()函数关闭相机\n
 *       Step9：调用Release()函数释放相机资源\n
 */
class DepthCamera
{
public:
    /**
     * @brief 构造函数
     */
    DepthCamera();

    /**
     * @brief 析构函数
     */
    ~DepthCamera();

    /**
     * @brief 拷贝构造函数
     * @param[in] depthCamera 拷贝对象
     * @note 禁用拷贝构造函数
     */
    DepthCamera(const DepthCamera &depthCamera) = delete;

    /**
     * @brief 拷贝赋值运算符
     * @param[in] depthCamera 拷贝对象
     * @return 拷贝赋值结果
     * @note 禁用拷贝赋值运算符
     */
    DepthCamera& operator=(const DepthCamera &depthCamera) = delete;

    /**
     * @brief 移动构造函数
     * @param[in] depthCamera 移动对象
     * @note 禁用移动构造函数
     */
    DepthCamera(DepthCamera &&depthCamera) = delete;

    /**
     * @brief 移动赋值运算符
     * @param[in] depthCamera 移动对象
     * @return 移动赋值结果
     * @note 禁用移动赋值运算符
     */
    DepthCamera& operator=(DepthCamera &&depthCamera) = delete;

    /**
     * @brief 获取相机参数
     * @return 相机参数
     */
    DepthCameraParam GetParam();

    /**
     * @brief 设置相机参数
     * @param[in] param 相机参数
     * @return 设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     * @note 如果相机已经初始化，相机参数将会设置失败
     */
    bool SetParam(const DepthCameraParam &param);

    /**
     * @brief 获取相机的初始化状态
     * @return 相机的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取相机的打开状态
     * @return 相机的打开状态\n
     *         -<em>false</em> 尚未打开\n
     *         -<em>true</em> 已经打开\n
     */
    bool IsOpened();

    /**
     * @brief 获取相机的工作状态
     * @return 相机的工作状态\n
     *         -<em>false</em> 异常状态\n
     *         -<em>true</em> 正常状态\n
     * @note 如果相机在线模式下打开之后，连续500帧数据错误，则工作状态变为异常
     */
    bool IsNormal();

    /**
     * @brief 获取相机的初始化时间戳
     * @return 相机的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 获取相机的打开时间戳
     * @return 相机的打开时间戳
     */
    uint64_t GetOpenTimestamp();

    /**
     * @brief 初始化相机
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     * @note 在Init()函数中通过调用SDK自行搜索序列号匹配的相机
     */
    bool Init();

    /**
     * @brief 释放相机系统资源
     * @return 资源释放结果\
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     * @note Release()和Init()配套使用；相机必须关闭之后才能执行释放操作。
     */
    bool Release();

    /**
     * @brief 打开相机
     * @return 打开结果\n
     *         -<em>false</em> 打开失败\n
     *         -<em>true</em> 打开成功\n
     */
    bool Open();

    /**
     * @brief 关闭相机
     * @return 关闭结果\n
     *         -<em>false</em> 关闭失败\n
     *         -<em>true</em> 关闭成功\n
     */
    bool Close();

    /**
     * @brief 重置缓存的相机数据帧
     */
    void ResetData();

    /**
     * @brief 获取缓存的相机数据帧
     * @param[out] data 相机数据帧
     */
    void GetData(DepthCameraData *data);

    /**
     * @brief 获取当前相机状态帧
     * @param[out] status 相机状态帧
     * @return 获取结果\n
     *         -<em>false</em> 获取失败\n
     *         -<em>true</em> 获取成功\n
     * @note 如果相机没有打开，当前状态帧将会获取失败
     */
    bool GetStatus(DepthCameraStatus *status);

private:
    DepthCameraParam param_;                 ///< 相机参数
    std::atomic<bool> isInitialized_;        ///< 相机初始化状态
    std::atomic<bool> isOpened_;             ///< 相机打开状态
    std::atomic<bool> isNormal_;             ///< 相机工作状态
    std::atomic<uint64_t> initTimestamp_;    ///< 相机初始化时间戳
    std::atomic<uint64_t> openTimestamp_;    ///< 相机打开时间戳
    cv::VideoCapture videoCapture_;          ///< 相机视频捕捉器
    cv::VideoWriter videoWriter_;            ///< 相机视频录制器
    std::atomic<int> frameWidth_;            ///< 相机数据帧宽度
    std::atomic<int> frameHeight_;           ///< 相机数据帧高度
    std::mutex operateMutex_;                ///< 相机操作互斥锁
    DepthCameraData data_;                   ///< 相机缓存数据帧
    mutable std::shared_mutex dataMutex_;    ///< 相机缓存数据帧读写锁
    std::atomic<bool> updateDataSwitch_;     ///< 相机缓存数据帧更新开关
    std::thread updateDataThread_;           ///< 相机缓存数据帧更新线程
    std::atomic<bool> recordVideoSwitch_;    ///< 相机缓存数据帧录制开关
    std::thread recordVideoThread_;          ///< 相机缓存数据帧录制线程

    /**
     * @brief 将相机的硬件参数写入设备
     * @param[in] hardwareParam 相机硬件参数
     * @return 相机硬件参数写入结果\n
     *         -<em>false</em> 写入失败\n
     *         -<em>true</em> 写入成功\n
     */
    void WriteHardwareParamToDevice(const DepthCameraHardwareParam &hardwareParam);

    /**
     * @brief 更新缓存的相机数据帧
     */
    void UpdateData();

    /**
     * @brief 通过离线视频更新缓存的相机数据帧
     */
    void UpdateDataFromOfflineVideo();

    /**
     * @brief 通过在线设备更新缓存的相机数据帧
     */
    void UpdateDataFromOnlineDevice();

    /**
     * @brief 录制缓存的相机数据帧
     */
    void RecordVideo();
};

// **************************************  相机事件的回调函数  **************************************

/**
 * @brief 相机连接状态改变事件回调函数
 * @param[in] pConnectArg 相机连接参数
 * @param[in] pUser       自定义用户数据
 */
static void HandleConnectChangedEvent(const IMV_SConnectArg* pConnectArg, void* pUser);

#endif //WORKSPACE_DEPTH_CAMERA_H
