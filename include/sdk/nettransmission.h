#ifndef NETTRANSMISSION_H
#define NETTRANSMISSION_H

#include <string>
#include "sdk/TransferCmd.h"
#include "sdk/Thread.h"
#include "sdk/const.h"
#include "sdk/BuffShared.h"
#include "sdk/api.h"
using namespace std;

struct StrcParam{
    int ExposureMin;    //曝光等级的最小值
    int ExposureMax;    //曝光等级的最大值
    int ExposureCur = -1;    //当前曝光值
};

class NetTransmission 
{
public:
    NetTransmission();
    ~NetTransmission();

public:
    enum TYPEPIPE{
        PIPE_CONTROL,   //用于传输指令
        PIPE_IMG_RAW,       //用于传输原始图像
        PIPE_IMG_SEPARATE,  //用于传输解析后的数据
    };
    string GetServerIP();
    int GetServerPort();
    int Start(string ip , int port, TYPEPIPE type );
    int Stop();
    STRC_IMG_ALL *GetImg();
    char *GetSdkVer();
    char *GetLensInfo();
    void SendCmdGetSdkVer();
    void SendCmdGetLensInfo();
    int SendCmdGetExposure();
    int GetExposure(int *min, int*max);
    int SetExposure(int level);

private:
    TYPEPIPE mPipeType;
    bool mLoop;
    bool mStoped;
    bool mConnected;
    string mIPServer;   //服务器的IP地址
    int mPortServer;    //服务器的端口
    int mSocket;
    CThread m_LoopThread;
    STRC_IMG_ALL *m_Img;
    CBuffShared *m_ManageImg;
    int m_SizeBuffImg;
    char *m_BuffSdkVer;
    char *m_BuffLensInfo;
    StrcParam mParam; 

    int Connect();
    int ProcessCtlCmd(StrcCmd *cmd);
    int RecLoginResult();   //用于接收登录返回的结果
    int InitImgBuff(int size=NUM_IMG_RECV);
    void FreeImgBuff();
    static void *LoopCtrl(void *param);
    static void *LoopImgSeparate(void *param);    
};

#endif // NETTRANSMISSION_H
