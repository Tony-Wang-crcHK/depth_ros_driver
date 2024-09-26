#pragma once
#include <stdint.h>

#pragma pack(push)
#pragma pack(1)

#define DATA_STREAM_HEAD 0xFE
#define DATA_STREAM_END 0xFF

enum CMD
{
    CMD_LOGIN,          //登录,告知此连接是图像数据还是控制
    CMD_IMG,                //传输图像

    CMD_EXPOSURE,
    CMD_WINDOW,
    CMD_GAIN,
    CMD_FLIP_H,         //设置镜像 0 enable   1 disable
    CMD_FLIP_V,
    CMD_BLACK_LEVEL,
    CMD_FREQUENCY,      //设置工作模式 0 SMF 1 DMF
    CMD_READ_REG,   //读寄存器
    CMD_WRITE_REG,  //写寄存器
    CMD_REG_VAL,    //返回的寄存器值

    CMD_LOGIN_RESULT, //登录结果
    CMD_RET,        //命令返回
    CMD_TICK,       //心跳

    CMD_BURST,      //设置曝光burst次数    
    CMD_IMG_PARSE,  //返回解析后的图像
    CMD_IMG_DEPTH,  //深度图
    CMD_IMG_AMPLITUDE,  //幅值图
    CMD_IMG_RGB,    //RGB图
    CMD_GET_LENS_INFO,   //获取SDK信息（镜头信息等）
    CMD_GET_LENS_INFO_RESULT,
    CMD_GET_VER,    //获取SDK版本号
    CMD_GET_VER_RESULT,
    CMD_GET_BURST,
    CMD_GET_EXPOSURE,   //获取当前的曝光参数
    CMD_END,
};


struct StrcHead
{
    uint32_t len;
    uint16_t cmd;
};

/**
 * @brief
 *  用于客户端登录，表明用户为图像传输连接，还是命令连接
 *  type : 0      命令连接
 *         1      RAW图像连接
 *         2      解算后图像(深度图、幅值图等)
 */
struct StrcLogin
{
    StrcHead head;
    uint8_t type;
};
/**
 * @brief
 * 图像传输命令
 *
 */
struct StrcImg
{
    StrcHead head;
};


/**
 * @brief 
 *      获取曝光相关参数
 * 
 */
struct StrcExposureGet{
    StrcHead head;
    uint16_t cur;
    uint16_t min;
    uint16_t max;
};
/**
 * @brief The StrcGain struct
 * 设置增益，0~31.
 */
struct StrcGain{
    StrcHead head;
    uint8_t gain;
};

/**
 * @brief The StrcFlip struct
 *  传感器镜像设置
 */
struct StrcFlip
{
    StrcHead head;
    uint8_t val;
};

struct StrcBlackLevel
{
    StrcHead head;
    uint16_t val;
};

/**
 * @brief The StrcReadReg struct
 *  读寄存器指令
 */
struct StrcReadReg
{
    StrcHead head;
    uint16_t reg;
};

/**
 * @brief The StrcRegVal struct
 *  寄存器值对，可用于写寄存器、读寄存器返回
 */
struct StrcRegVal
{
    StrcHead head;
    uint16_t reg;
    uint8_t val;
};

/**
 * @brief 
 *      命令的返回值
 */
struct StrcRet
{
    StrcHead head;
    uint16_t cmd;
    uint16_t result;
};

/**
 * @brief The StrcWin struct
 *  定义传感器开窗功能中窗口的参数。
 *  win_s   窗口在传感器中左上角的起始行
 *  win_l   窗口有几行数据
 */
struct StrcWin{
    uint16_t win_s;
    uint16_t win_l;
};

#define MAX_NUM_WIN 4
/**
 * @brief The StrcMultiWin struct
 *  定义开窗功能的相关参数
 *  win 窗口信息
 *  cwin0_s 所有窗口左上角位置的列起始 （必需为8的倍数）
 *  cwin0_l 所有窗口的宽度 （必需为8的倍数）
 */
struct StrcMultiWin{
    StrcHead head;
    StrcWin win[MAX_NUM_WIN];
    uint16_t cwin0_s;
    uint16_t cwin0_l;
    uint8_t mode;
};

struct StrcSetVal
{
    StrcHead head;
    uint16_t val;
};

/**
 * @brief The StrcCmd union
 *  所有命令结构的联合体
 */
union StrcCmd
{
   StrcHead head;
   StrcLogin login;
   StrcImg img;
   StrcSetVal exposure;
   StrcGain gain;
   StrcFlip flip;
   StrcFlip frequency;
   StrcBlackLevel black;
   StrcReadReg readReg;
   StrcRegVal regVal;
   StrcRet ret;
   StrcMultiWin multiWin;
   StrcSetVal burst;   
   StrcHead info;
   StrcExposureGet exposureGet;
};




#pragma pack(pop)


