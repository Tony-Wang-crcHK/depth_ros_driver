# TOF相机的ROS驱动 depth_ros_driver
  本程序基于指数星空驱动程序进行封装，ROS用户运行launch即可得到发布的ROS格式深度图。
  但由于ROS本身格式相对臃肿，本驱动还包含了视频录制功能，存储时间戳，读取以录制视频的功能。

## 一. 安装 & 运行
  本项目依赖ROS，以及openCV等开源系统

### 1.1 依赖安装
  推荐一键安装ROS： \
   wget http://fishros.com/install -O fishros && . fishros \
  之后在workspace的src下: \
   git clone https://github.com/Tony-Wang-crcHK/depth_ros_driver.git \
   cd .. \
   catkin_make 

### 1.2 运行
   source devel/setup.bash
   roslaunch depth_ros_driver depth_camera.launch

## 三. 补充
  有任何问题请联系: wyq18134091031@gmail.com
