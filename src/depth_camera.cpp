//
// Created by tony on 9/29/24.
//

#include "depth_camera.h"

namespace DepthRosDriver
{
    // ******************************  DepthCamera类的公有函数  ******************************

    // 构造函数
    DepthCamera::DepthCamera() :
            param_(),
            isInitialized_(false),
            isOpened_(false),
            isNormal_(false),
            initTimestamp_(0),
            openTimestamp_(0),
            cameraHandle_(-1),
            videoCapture_(),
            videoWriter_(),
            frameWidth_(640),
            frameHeight_(480),
            operateMutex_(),
            data_(),
            dataMutex_(),
            updateDataSwitch_(false),
            updateDataThread_(),
            recordVideoSwitch_(false),
            recordVideoThread_() {
        ResetData();
    }

   // 析构函数
    DepthCamera::~DepthCamera()
    {
        // 关闭相机
        if (IsOpened())
        {
            Close();
        }

        // 释放相机系统资源
        if (IsInitialized())
        {
            Release();
        }
    }

    // 获取相机参数
    DepthCameraParam DepthCamera::GetParam()
    {
        std::lock_guard <std::mutex> lockGuard(operateMutex_);
        return param_;
    }

    // 设置相机参数
    bool DepthCamera::SetParam(const DepthCameraParam &param)
    {
        // 锁定相机的操作
        std::lock_guard <std::mutex> lockGuard(operateMutex_);

        // 判断相机是否已经初始化
        if (isInitialized_)
        {
            return false;
        }

        // 记录相机参数
        param_ = param;

        // 返回设置结果
        return true;
    }

    // 获取相机的初始化状态
    bool DepthCamera::IsInitialized()
    {
        std::lock_guard <std::mutex> lockGuard(operateMutex_);
        return isInitialized_;
    }

    // 获取相机的打开状态
    bool DepthCamera::IsOpened()
    {
        std::lock_guard <std::mutex> lockGuard(operateMutex_);
        return isOpened_;
    }

    // 获取相机的工作状态
    bool DepthCamera::IsNormal()
    {
        std::lock_guard <std::mutex> lockGuard(operateMutex_);
        return isNormal_;
    }

    // 获取相机的初始化时间戳
    uint64_t DepthCamera::GetInitTimestamp()
    {
        std::lock_guard <std::mutex> lockGuard(operateMutex_);
        return initTimestamp_;
    }

    // 获取相机的打开时间戳
    uint64_t DepthCamera::GetOpenTimestamp()
    {
        std::lock_guard <std::mutex> lockGuard(operateMutex_);
        return openTimestamp_;
    }

    // 初始化相机
    bool DepthCamera::Init()
    {
        // 锁定相机的操作
        std::lock_guard <std::mutex> lockGuard(operateMutex_);

        // 判断相机是否已经初始化
        if (isInitialized_)
        {
            return false;
        }

        // 根据相机的运行状态进行初始化
        if (param_.RuntimeParam.IsOnline)
        {
            // 如果相机需要录像，判断录像文件存储路径是否合法
            if (param_.RuntimeParam.IsRecordVideo)
            {
                // 判断录像文件存储路径是否存在
                if (::access(param_.RuntimeParam.RecordVideoPath.c_str(), R_OK) == -1)
                {
                    if (::mkdir(param_.RuntimeParam.RecordVideoPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
                    {
                        return false;
                    }
                }
            }

            // 初始化API
            api_init();
        }
        else
        {
            // 判断离线视频文件是否存在
            if (::access(param_.RuntimeParam.OfflineVideoName.c_str(), F_OK) == -1)
            {
                return false;
            }

            // 判断离线视频文件是否可读
            if (::access(param_.RuntimeParam.OfflineVideoName.c_str(), R_OK) == -1)
            {
                return false;
            }
        }

        // 设置初始化时间戳
        std::chrono::time_point <std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        initTimestamp_ = now.time_since_epoch().count();

        // 设置初始化状态
        isInitialized_ = true;

        // 返回初始化结果
        return true;
    }

    // 释放相机系统资源
    bool DepthCamera::Release()
    {
        // 锁定相机的操作
        std::lock_guard <std::mutex> lockGuard(operateMutex_);

        // 判断相机是否已经打开
        if (isOpened_)
        {
            return false;
        }

        // 判断相机是否初始化完毕
        if (!isInitialized_)
        {
            return false;
        }

        // 释放相机系统资源
        if (param_.RuntimeParam.IsOnline)
        {
            // 销毁相机句柄
            api_exit();

            // 重置相机句柄
            cameraHandle_ = -1;
        }

        // 重置初始化时间戳
        initTimestamp_ = 0;

        // 重置初始化状态
        isInitialized_ = false;

        // 返回释放结果
        return true;
    }

    // 打开相机
    bool DepthCamera::Open()
    {
        // 锁定相机的操作
        std::lock_guard <std::mutex> lockGuard(operateMutex_);

        // 判断相机是否初始化完毕
        if (!isInitialized_)
        {
            return false;
        }

        // 判断相机是否已经打开
        if (isOpened_)
        {
            return false;
        }

        // 根据相机的运行状态打开相机
        if (param_.RuntimeParam.IsOnline)
        {
            // 连接相机句柄
            cameraHandle_ = api_connect((char *) param_.IP.c_str());

            if (cameraHandle_ < 0)
                return false;

            // 注册相机连接状态改变事件回调函数

            // 将相机当前硬件参数写入设备
            WriteHardwareParamToDevice(param_.HardwareParams);

            // 读取相机的图像宽度

            // 读取相机的图像高度

            // 更新相机数据帧的宽度和高度
            frameWidth_ = 640;
            frameHeight_ = 480;

            // 如果需要录像，则打开相机视频录制器
            if (param_.RuntimeParam.IsRecordVideo)
            {
                // 获取当前日期时间字符串
                std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
                std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                std::tm tm = *std::localtime(&now_time);
                std::stringstream stream;
                stream << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");
                std::string dateTimeString = stream.str();

                // 创建录像文件名称和录像文件的绝对路径
                std::string recordFileName = "[" + param_.IP + "] - RawVideo_" + dateTimeString + ".avi";
                std::string fullPath = param_.RuntimeParam.RecordVideoPath + recordFileName;

                // 打开数据帧录制器
                if (!videoWriter_.open(fullPath,
                                       cv::VideoWriter::fourcc('M', 'P', '4', '2'),
                                       param_.RuntimeParam.RecordVideoFps,
                                       cv::Size(frameWidth_, frameHeight_),
                                       true)) {
                    return false;
                }
            }
        }
        else
        {
            // 打开视频捕捉器
            if (videoCapture_.open(param_.RuntimeParam.OfflineVideoName))
            {
                // 读取离线视频文件数据帧的高度和宽度
                double width = videoCapture_.get(cv::CAP_PROP_FRAME_WIDTH);
                double height = videoCapture_.get(cv::CAP_PROP_FRAME_HEIGHT);

                // 更新相机数据帧的宽度和高度
                frameWidth_ = 640;
                frameHeight_ = 480;
            } else
                return false;

        }

        // 启动相机缓存数据帧更新线程
        updateDataSwitch_ = true;
        updateDataThread_ = std::thread(&DepthCamera::UpdateData, this);

        // 启动相机缓存数据帧录制线程
        if (param_.RuntimeParam.IsOnline && param_.RuntimeParam.IsRecordVideo)
        {
            recordVideoSwitch_ = true;
            recordVideoThread_ = std::thread(&DepthCamera::RecordVideo, this);
        }

        // 设置打开时间戳
        std::chrono::time_point <std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        openTimestamp_ = now.time_since_epoch().count();

        // 设置打开状态
        isOpened_ = true;

        // 设置工作状态
        isNormal_ = true;

        // 返回打开结果
        return true;
    }

    // 关闭相机
    bool DepthCamera::Close()
    {
        // 锁定相机的操作
        std::lock_guard <std::mutex> lockGuard(operateMutex_);

        // 判断相机是否已经打开
        if (!isOpened_)
        {
            return false;
        }

        // 停止相机缓存数据帧更新线程
        updateDataSwitch_ = false;
        if (updateDataThread_.joinable())
        {
            updateDataThread_.join();
        }

        // 判断相机是否为在线模式
        if (param_.RuntimeParam.IsOnline)
        {
            // 停止相机缓存数据帧录制线程
            if (param_.RuntimeParam.IsRecordVideo)
            {
                recordVideoSwitch_ = false;
                if (recordVideoThread_.joinable())
                {
                    recordVideoThread_.join();
                }
            }

            // 释放数据帧录制器
            if (videoWriter_.isOpened())
            {
                videoWriter_.release();
            }

            // 停止拉流
            api_disconnect(cameraHandle_);
        }
        else
        {
            // 释放视频捕捉器
            if (videoCapture_.isOpened())
            {
                videoCapture_.release();
            }
        }

        // 重置打开时间戳
        openTimestamp_ = 0;

        // 重置打开状态
        isOpened_ = false;

        // 重置工作状态
        isNormal_ = false;

        // 返回关闭结果
        return true;
    }

    // 重置缓存的相机数据帧
    void DepthCamera::ResetData()
    {
        // 锁定相机数据缓冲区
        std::unique_lock <std::shared_mutex> uniqueLock(dataMutex_);

        // 重置相机数据帧的索引
        data_.Index = 0;

        // 重置相机数据帧的时间戳
        std::chrono::time_point <std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        data_.Timestamp = now.time_since_epoch().count();

        // 重置相机数据帧的图像
//    data_.RGBImage = cv::Mat(frameHeight_,
//                          frameWidth_,
//                          CV_8UC3,
//                          cv::Scalar(0, 0, 255));

        data_.DepthImage = (480, 640, CV_32F, cv::Scalar(0));

//    cv::putText(data_.RGBImage,
//                "The cached data has been reset",
//                cv::Point(frameWidth_ / 2 - 200, frameHeight_ / 2),
//                cv::FONT_HERSHEY_SIMPLEX,
//                1.0,
//                cv::Scalar(0, 0, 0),
//                2);

    cv::putText(data_.DepthImage,
                "The cached data has been reset",
                cv::Point(frameWidth_ / 2 - 200, frameHeight_ / 2),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(255),
                2);
    }

   // 获取缓存的相机数据帧
    void DepthCamera::GetData(DepthCameraData *data)
    {
        std::shared_lock <std::shared_mutex> sharedLock(dataMutex_);
        data->Index = data_.Index;
        data->Timestamp = data_.Timestamp;
        data_.DepthImage.copyTo(data->DepthImage);
    }

// 获取当前相机状态帧
    bool DepthCamera::GetStatus(DepthCameraStatus *status)
    {
        // 锁定相机的操作
        std::lock_guard <std::mutex> lockGuard(operateMutex_);

        // 判断相机是否处于在线状态
        if (!param_.RuntimeParam.IsOnline)
        {
            return false;
        }

        // 判断相机是否打开
        if (!isOpened_)
        {
            return false;
        }

        char *pInfo = NULL;

        pInfo = api_get_intrinsic_parameters(cameraHandle_);

        if(pInfo != NULL)
            std::cout<< "Intrinsics/Distortions: "<<std::endl<<pInfo <<std::endl;

//        nlohmann::json jsonCameraInfo = nlohmann::json::parse(cameraInfo);
//
//        status -> EigenIntrinsics << j["f_x"], 0, j["c_x"], 0, j["f_y"], j["c_y"], 0, 0, 1;
//
//        status -> EigenDistortions << j["p_1"], j["p_2"], j["k_1"], j["k_2"], j["k_3"];

        // std::cout<< "status: "<< cameraInfo <<std::endl;

        // 返回获取结果
        return true;
    }

// ******************************  DepthCamera类的私有函数  ******************************

// 将相机的硬件参数写入设备
    void DepthCamera::WriteHardwareParamToDevice(const DepthCameraHardwareParam &hardwareParam)
    {
        if (hardwareParam.IsExposureAutoDepth)
        {
            ///TODO::
        }
        else
        {
            // 写入ExposureTimeDepth
            // std::cout<<" hardwareParam.ExposureTimeDepth: "<< hardwareParam.ExposureTimeDepth<<std::endl;

            api_set_exposure(cameraHandle_, hardwareParam.ExposureTimeDepth);
        }
    }

// 更新缓存的相机数据帧
    void DepthCamera::UpdateData()
    {
        // 修改线程名称
        std::string threadName = "update_camera_data";
        prctl(PR_SET_NAME, threadName.c_str());

        // 设置线程绑定的CPU内核
        // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
        //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
        //         https://blog.csdn.net/liaoxiangui/article/details/7905612
        //         https://blog.csdn.net/zxc024000/article/details/79438061
        //         https://www.bbsmax.com/A/q4zVKp9XJK/
        int coreNumber = get_nprocs();
        int coreIndex = param_.RuntimeParam.UpdateDataCpuCore;
        if ((coreIndex >= 0) && (coreIndex < coreNumber))
        {
            cpu_set_t mask;
            CPU_ZERO(&mask);
            CPU_SET(coreIndex, &mask);
            sched_setaffinity(0, sizeof(mask), &mask);
        }

        // 更新缓存的相机数据帧
        if (param_.RuntimeParam.IsOnline)
        {
            UpdateDataFromOnlineDevice();
        }
        else
        {
            UpdateDataFromOfflineVideo();
        }
    }

// 通过离线视频更新缓存的相机数据帧
    void DepthCamera::UpdateDataFromOfflineVideo()
    {
        // 读取离线视频文件的帧数
        int frameCount = static_cast<int>(videoCapture_.get(cv::CAP_PROP_FRAME_COUNT));

        // 读取离线视频文件的帧率，计算相邻数据帧之间的时间延迟(微秒)
        double fps = videoCapture_.get(cv::CAP_PROP_FPS);
        auto frameDelay = static_cast<uint64_t>(1000000000.0 / fps);

        // 循环读取离线视频图像
        cv::Mat image;
        while (updateDataSwitch_)
        {
            // 获取起始时间戳
            std::chrono::time_point <std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
            uint64_t beginTimestamp = beginTime.time_since_epoch().count();

            // 读取离线视频数据帧
            if (!videoCapture_.read(image))
            {
                break;
            }

            // 获取当前时间戳
            std::chrono::time_point <std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            uint64_t timestamp = now.time_since_epoch().count();

            // 更新缓存的相机数据帧
            dataMutex_.lock();
            data_.Index++;
            data_.Timestamp = timestamp;
            image.copyTo(data_.DepthImage);
            dataMutex_.unlock();

            // 获取当前帧图像的位置，判断当前帧图像是否为最后一帧图像
            // TODO 对于某些离线视频文件，使用capture.get(cv::CAP_PROP_FRAME_COUNT)函数读取到的frameCount为0；\n
            //      无法通过framePosition判断是否播放到最后一帧图像，必须重新打开离线视频文件。
            int framePosition = static_cast<int>(videoCapture_.get(cv::CAP_PROP_POS_FRAMES));
            if ((frameCount > 0) && (framePosition == frameCount))
            {
                // 重置缓存的相机数据帧
                ResetData();

                // 重置当前帧图像的位置
                videoCapture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            }

            // 获取截止时间戳
            std::chrono::time_point <std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
            uint64_t endTimestamp = endTime.time_since_epoch().count();

            // 线程延时，保证离线视频图像读取速度正常
            uint64_t timeSpan = endTimestamp - beginTimestamp;
            if (frameDelay > timeSpan)
            {
                std::this_thread::sleep_for(std::chrono::nanoseconds(frameDelay - timeSpan));
            }
        }

        // 重置缓存的相机数据帧
        ResetData();
    }

// 通过在线设备更新缓存的相机数据帧
    void DepthCamera::UpdateDataFromOnlineDevice()
    {
        // 初始化原始数据帧
        STRC_IMG_ALL *frame = new STRC_IMG_ALL();

        // 初始化异常状态计数器
        unsigned int abnormalCounter = 0;

        // 循环读取并转换原始数据帧
        while (updateDataSwitch_)
        {
            // 获取原始数据帧
            frame = api_get_img(cameraHandle_);
            if (frame == NULL)
            {
                // 判断并修改相机的工作状态
                // 注意：在实际使用时，可以根据实际需要修改判断条件
                abnormalCounter++;
                if (abnormalCounter >= 500)
                {
                    isNormal_ = false;
                }

                continue;
            }

            // 重置异常状态计数器
            abnormalCounter = 0;

            // 给每个像素赋值，这里以赋值为它的索引位置为例
            cv::Mat depthMap(480, 640, CV_32F, cv::Scalar(0));
            for (int i = 0; i < frameHeight_; ++i)
            {
                for (int j = 0; j < frameWidth_; ++j)
                {
                    float &pixel = depthMap.at<float>(i, j);
                    pixel = static_cast<float>(frame->img_depth.data[i * frameWidth_ + j]);
                }
            }

            std::chrono::time_point <std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            uint64_t timestamp = now.time_since_epoch().count();

            // 更新缓存的相机数据帧
            dataMutex_.lock();
            data_.Index++;
            data_.Timestamp = timestamp;
            depthMap.copyTo(data_.DepthImage);
            dataMutex_.unlock();
        }

        // 释放原始数据帧
        delete frame;
        frame = nullptr;

        // 重置缓存的相机数据帧
        ResetData();
    }

// 录制缓存的相机数据帧
    void DepthCamera::RecordVideo()
    {
        // 修改线程名称
        std::string threadName = "record_camera_data";
        prctl(PR_SET_NAME, threadName.c_str());

        // 设置线程绑定的CPU内核
        // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
        //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
        //         https://blog.csdn.net/liaoxiangui/article/details/7905612
        //         https://blog.csdn.net/zxc024000/article/details/79438061
        //         https://www.bbsmax.com/A/q4zVKp9XJK/
        int coreNumber = get_nprocs();
        int coreIndex = param_.RuntimeParam.RecordVideoCpuCore;
        if ((coreIndex >= 0) && (coreIndex < coreNumber))
        {
            cpu_set_t mask;
            CPU_ZERO(&mask);
            CPU_SET(coreIndex, &mask);
            sched_setaffinity(0, sizeof(mask), &mask);
        }

        // 初始化临时图像
        cv::Mat depthImage;

        // 计算采样周期；单位：纳秒
        auto samplePeriod = static_cast<uint64_t>(1000000000.0 / param_.RuntimeParam.RecordVideoFps);

        // 录制相机数据帧
        while (recordVideoSwitch_)
        {
            // 获取起始时间戳
            std::chrono::time_point <std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
            uint64_t beginTimestamp = beginTime.time_since_epoch().count();

            // 复制相机数据帧中的图像
            dataMutex_.lock();
            data_.DepthImage.copyTo(depthImage);
            dataMutex_.unlock();

            // 判断图像参数是否与视频录制器匹配
            if ((depthImage.cols != frameWidth_) || (depthImage.rows != frameHeight_) || (depthImage.type() != CV_32F))
            {
                continue;
            }

            // 保存相机数据帧中的图像
            cv::Mat converted;
            depthImage.convertTo(converted, CV_8U);
            videoWriter_.write(converted);

            // 获取截止时间戳
            std::chrono::time_point <std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
            uint64_t endTimestamp = endTime.time_since_epoch().count();

            // 计算相机数据帧的保存时间
            uint64_t writeSpan = endTimestamp - beginTimestamp;

            // 线程延时，确保录制视频的帧率正常
            if (samplePeriod > writeSpan)
            {
                std::this_thread::sleep_for(std::chrono::nanoseconds(samplePeriod - writeSpan));
            }
        }
    }
}
// ******************************  DepthCamera类的事件回调函数  ******************************