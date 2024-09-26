#include "sdk/nettransmission.h"
#include "sdk/BuffShared.h"
#include "sdk/const.h"
#include <sys/socket.h>
#include <sys/types.h>
#include "sdk/api.h"
#include "sdk/TransferCmd.h"
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#ifdef DEBUG
#include <iostream>   
#endif

extern void api_set_errorcode(ERROR_CODE code);

/**
 * @brief ReadData
 *      从指定SOCKET中读取指定长度的数据
 * @return
 */
inline int ReadData(int fd , char *buff , int len )
{
    int ret = 0;
    int readLen = 0;
    int tmpLen;
    while(readLen < len)
    {
        tmpLen = recv(fd , (void *)&buff[readLen] , len-readLen , MSG_WAITALL);
        if(tmpLen <= 0)
        {
            ret = -1;
            break;
        }
        readLen += tmpLen;
    }
    ret = ret == 0?readLen:ret;
    return ret;
}

NetTransmission::NetTransmission()
{
    mPipeType = PIPE_CONTROL;
    mLoop = false;
    mStoped = true;
    mIPServer = "";
    mPortServer = 0;
    m_BuffSdkVer = (char *)malloc(1024);
    memset((void *)m_BuffSdkVer , 0 , 1024);
    m_BuffLensInfo = (char *)malloc(1024*5);
    memset((void *)m_BuffLensInfo , 0 , 1024*5);

}

NetTransmission::~NetTransmission()
{
    if(m_BuffLensInfo != NULL)
    {
        free(m_BuffLensInfo);
    }
    if(m_BuffSdkVer != NULL)
    {
        free(m_BuffSdkVer);
    }
}

void *NetTransmission::LoopCtrl(void *param)
{
    NetTransmission *pInstance = (NetTransmission *)param;
    CTransferCmd mTransfer;
    int mRet = 0;
    void *mBuff = malloc(1024*10);
    StrcCmd *mCmd = (StrcCmd *)mBuff;
    int mSocket = pInstance->mSocket;

    pInstance->SendCmdGetLensInfo();
    pInstance->SendCmdGetSdkVer();
    pInstance->SendCmdGetExposure();

    pInstance->mLoop = true;
    pInstance->mStoped = false;
    #ifdef DEBUG
    std::cout << "LoopCtrl start." <<std::endl;
    #endif    
    while(pInstance->mLoop)
    {
        //接收数据流起始标志
        mRet = mTransfer.ReceiveTagOfStreamHead(mSocket);
         
        if(mRet == -1)
        {
            pInstance->mLoop = false;
            continue;
        }else if(mRet != 0)
        {
            continue;
        }
        //接收命令数据
            //接收命令长度数据
        mRet = ReadData(mSocket , (char *)mCmd , sizeof(mCmd->head.len));
 
        if (mRet <= 0) {
            pInstance->mLoop = false;
            continue;
        }
        if (mCmd->head.len > (1024*10) || mCmd->head.len < sizeof(StrcHead)) {
            continue;
        }
            //接收命令的数据
        mRet = ReadData(mSocket ,(char *)mCmd+sizeof(mCmd->head.len) , mCmd->head.len - sizeof(mCmd->head.len));

        if (mRet <= 0) {
            pInstance->mLoop = false;
            continue;
        }
        //接收数据流结束标志
        mRet = mTransfer.ReceiveTagOfStreamEnd(mSocket);
         
        if (mRet == -1) {
            pInstance->mLoop = false;
            continue;
        }else if (mRet == -2){
            continue;
        }

        pInstance->ProcessCtlCmd(mCmd);
    }
    free(mBuff);
    pInstance->mStoped = true;
    close(mSocket);
    #ifdef DEBUG
    std::cout << "LoopCtrl end." <<std::endl;
    #endif    
    return NULL;
}

void *NetTransmission::LoopImgSeparate(void *param)
{
    NetTransmission *pInstance = (NetTransmission *)param;
    CTransferCmd mTransfer;
    StrcHead mCmdHead ;
    int mRecvLen = 0;
    uint32_t mDataLenRecv = 0;
    uint32_t mLenPackage = 0;
    int mSocket = pInstance->mSocket;
    CBuffShared *pImgBuff = NULL;
    STRC_IMG_ALL *pImgAll = NULL;   //用于存储当前图像
    char *tmpBuff = (char *)malloc(1024);
    if(0 != pInstance->InitImgBuff())
    {
        goto END;
    }

    pImgBuff = pInstance->m_ManageImg;
    pInstance->mLoop = true;
    pInstance->mStoped = false;
    while(pInstance->mLoop)
    {
        if(mTransfer.ReceiveTagOfStreamHead(mSocket) != 0)
        {
            continue;
        }
        mRecvLen = ReadData(mSocket,(char *)&mCmdHead , sizeof(mCmdHead));
        if(mRecvLen != sizeof (mCmdHead) || mCmdHead.cmd != CMD_IMG_PARSE)
        {
            continue;
        }

        mLenPackage = mCmdHead.len;
        mDataLenRecv = sizeof(StrcHead);

        if(pImgBuff->IsFull())
        {//丢掉此帧
            //pImgBuff->GetBuff(CBuffShared::READING);
            int tmpLen;
            while(mDataLenRecv < mLenPackage)
            {
                tmpLen = mLenPackage-mDataLenRecv>1024?1024:mLenPackage-mDataLenRecv;
                if( ReadData(mSocket,tmpBuff , tmpLen) < 0)
                {
                     pInstance->mLoop = false;
                    break;
                }
                mDataLenRecv += tmpLen;
            }
            continue;
        }
        pImgAll = (STRC_IMG_ALL*)pImgBuff->GetBuff(CBuffShared::WRITING);        
        if( pImgAll == NULL)
        {
            perror("buff error.");
            pInstance->mLoop = false;
            break;
        }
        pImgAll->img_amplitude.len = 0;
        pImgAll->img_depth.len = 0;
        pImgAll->img_rgb.len = 0;

        while (mDataLenRecv < mLenPackage) {
            if( ReadData(mSocket,(char *)&mCmdHead , sizeof(StrcHead)) <=0 )
            {
                 pInstance->mLoop = false;
                break;
            }

            if(mCmdHead.cmd == CMD_IMG_AMPLITUDE)
            {
                if( ReadData(mSocket,(char *)pImgAll->img_amplitude.data , mCmdHead.len-sizeof(StrcHead)) < 0)
                {
                     pInstance->mLoop = false;
                    break;
                }
                pImgAll->img_amplitude.len = mCmdHead.len-sizeof(StrcHead);
            }else if(mCmdHead.cmd == CMD_IMG_DEPTH)
            {
                if(ReadData(mSocket,(char *)pImgAll->img_depth.data , mCmdHead.len-sizeof(StrcHead)) < 0)
                {
                    pInstance->mLoop = false;
                    break;
                }
                pImgAll->img_depth.len = mCmdHead.len-sizeof(StrcHead);
            }else if(mCmdHead.cmd == CMD_IMG_RGB)
            {
                if(ReadData(mSocket,(char *)pImgAll->img_rgb.data , mCmdHead.len-sizeof(StrcHead)) < 0)
                {
                    pInstance->mLoop = false;
                    break;
                }
                pImgAll->img_rgb.len = mCmdHead.len-sizeof(StrcHead);
            }
            mDataLenRecv += mCmdHead.len;
        }

        if(mTransfer.ReceiveTagOfStreamEnd(mSocket) != 0)
        {
            continue;
        }

        pImgAll = NULL;
    }
END:
    pInstance->FreeImgBuff();
    pInstance->mStoped = true;
    close(mSocket);
    free(tmpBuff);
    return NULL;
}

/**
 * @brief NetTransmission::Start
 *      启动本线程
 * @param ip
 *      TCP要连接的服务器地址
 * @param port
 *      TCP要连接的服务器端口
 * @param type
 *      本连接的类型（控制、图像接收）
 */
int NetTransmission::Start(string ip , int port,TYPEPIPE type)
{
    int ret = 0;
    mPipeType = type;
    mIPServer = ip;
    mPortServer = port;
    m_ManageImg = NULL;

    //连接到服务器
    ret = Connect();

    if(ret == 0)
    {
        CTransferCmd mTcpTx;
        //登录
        ret = mTcpTx.Login((int)mPipeType, mSocket);
        if(0 > ret)
        {
            api_set_errorcode(STATUS_SOCKET_SEND_FAILED);
            return ret ;
        }

        //开始进程
        if(mPipeType == PIPE_CONTROL)
        {
            ret = RecLoginResult();
            if(ret != 0)
            {
                return ret;
            }            
            m_LoopThread.Start(NetTransmission::LoopCtrl, this);
        }else if(mPipeType == PIPE_IMG_SEPARATE)
        {
            m_LoopThread.Start(NetTransmission::LoopImgSeparate, this);
            
        }    
    }
    return ret;
}

int NetTransmission::Connect()
{
    int ret = 0;
    mSocket = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int keepAlive = 1; 
    setsockopt(mSocket , SOL_SOCKET , SO_KEEPALIVE , (void *)&keepAlive, sizeof(keepAlive));

    int syncnt=1;
    setsockopt(mSocket , IPPROTO_TCP , TCP_SYNCNT , &syncnt , sizeof(syncnt) );
    if(mSocket < 0)
    {
        api_set_errorcode(STATUS_SOCKET_FAILED);
        ret = -1;
    }
    else
    {
        struct sockaddr_in servaddr;
        bzero(&servaddr, sizeof(servaddr)); 
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(mIPServer.c_str());
        servaddr.sin_port = htons(mPortServer);
        ret = connect(mSocket , (struct sockaddr*)&servaddr, sizeof(servaddr));
        if( ret < 0)
        {
            api_set_errorcode(STATUS_SOCKET_CONNECT_FAILED);

        }

    }
    return ret;
}

/**
 * @brief NetTransmission::GetServerIP
 *      获取当前连接的服务器地址
 * @return
 *
 */
string NetTransmission::GetServerIP()
{
    return mIPServer;
}

/**
 * @brief NetTransmission::GetServerPort
 *      获取当前连接的服务器的端口
 * @return
 */
int NetTransmission::GetServerPort()
{
    return mPortServer;
}

int NetTransmission::ProcessCtlCmd(StrcCmd *cmd)
{
    int ret = 0;
    CTransferCmd mTransfer;
    char *p = (char *)cmd;
    #ifdef DEBUG
    std::cout << "Process cmd:" << cmd->head.cmd <<std::endl;
    #endif

    switch (cmd->head.cmd) {
    case CMD_LOGIN_RESULT:

        break;
    case CMD_TICK:
        mTransfer.SendTick(mSocket);
        break;
    case CMD_GET_VER_RESULT:
        memcpy((void *)m_BuffSdkVer , (void *)(p+sizeof(StrcHead)) , cmd->head.len - sizeof(StrcHead));
        break;
    case CMD_GET_LENS_INFO_RESULT:
        memcpy((void *)m_BuffLensInfo , (void *)(p+sizeof(StrcHead)) , cmd->head.len - sizeof(StrcHead));
        break;
    case CMD_GET_EXPOSURE:
        mParam.ExposureCur = cmd->exposureGet.cur;
        mParam.ExposureMax = cmd->exposureGet.max;
        mParam.ExposureMin = cmd->exposureGet.min;
        break;
    default:
        ret = -1;
        break;
    }


    return ret;
}

int NetTransmission::Stop()
{
    close(mSocket);
    mLoop = false;
    while(!mStoped)
    {
        usleep(10000);
    }
    return 0;

}

/**
 * @brief 
 *      接收登录指令返回的结果
 * @param cmd 
 *      返回的数据
 * @return int 
 *      >=0     登录成功
 *      <0      登录失败
 */
int NetTransmission::RecLoginResult()
{
    int mRet = 0;
    CTransferCmd mTransfer;
    StrcCmd cmd;
    
    mRet = mTransfer.ReceiveTagOfStreamHead(mSocket);
    if(mRet != 0)
    {
        return mRet;
    }
    mRet = ReadData(mSocket , (char *)&cmd , sizeof(StrcLogin));
    if(mRet != sizeof(StrcLogin))
    {
        mRet = -1;
        return mRet;
    }
    mRet = mTransfer.ReceiveTagOfStreamEnd(mSocket);
    if(mRet == 0)
    {
        if(cmd.login.head.cmd != CMD_LOGIN_RESULT || cmd.login.type != 0)
        {
            mRet = -1;
        }
    }
    return mRet;
}

int NetTransmission::InitImgBuff(int size)
{
    int ret = 0;
    void **array;
    m_SizeBuffImg = size;
    m_Img = (STRC_IMG_ALL *)malloc(sizeof(STRC_IMG_ALL)*size);

    if(m_Img == NULL)
    {
        ret = -1;
    }
    else
    {
        array = (void **)malloc(sizeof(void *)*size);
        memset((void *)m_Img , 0  , sizeof(STRC_IMG_ALL)*size);
        for(int i=0; i<size ;i++)
        {
            m_Img[i].img_amplitude.data = (uint16_t*)malloc(MAX_LEN_IMG_DATA);
            m_Img[i].img_depth.data = (uint16_t*)malloc(MAX_LEN_IMG_DATA);
            m_Img[i].img_rgb.data = (uint16_t*)malloc(MAX_LEN_IMG_RGB_DATA);

            if(m_Img[i].img_amplitude.data==NULL || m_Img[i].img_depth.data == NULL || m_Img[i].img_rgb.data == NULL)
            {
                ret = -1;
                break;
            }
            array[i] = (void *)&m_Img[i];
        }

    }
    if(ret == -1)
    {
        FreeImgBuff();
    }else{
        m_ManageImg = new CBuffShared();
        m_ManageImg->Init(array , size);
        free(array);
    }
    return ret;
}

void NetTransmission::FreeImgBuff()
{
    if(m_Img != NULL)
    {
        for(int i=0; i<m_SizeBuffImg; i++)
        {
            if(m_Img[i].img_amplitude.data != NULL)
            {
                free(m_Img[i].img_amplitude.data);
            }
            if(m_Img[i].img_depth.data != NULL)
            {
                free(m_Img[i].img_depth.data);
            }
            if(m_Img[i].img_rgb.data != NULL)
            {
                free(m_Img[i].img_rgb.data);
            }
        }
        free(m_Img);
        m_Img = NULL;
    }

    m_SizeBuffImg = 0 ;
    if(m_ManageImg != NULL)
    {
        free(m_ManageImg);
        m_ManageImg = NULL;
    }
}

STRC_IMG_ALL *NetTransmission::GetImg()
{
    if(m_ManageImg == NULL)
    {
        return NULL;
    }else{
        return (STRC_IMG_ALL *)m_ManageImg->GetBuff(CBuffShared::READING);
    }
     
}

char *NetTransmission::GetSdkVer()
{
    if(strlen(m_BuffSdkVer) > 0)
    {
        return m_BuffSdkVer;
    }
    return NULL;
}

char *NetTransmission::GetLensInfo()
{
    if(strlen(m_BuffLensInfo) > 0)
    {
        return m_BuffLensInfo;
    }
    return NULL;

}

void NetTransmission::SendCmdGetSdkVer()
{
    CTransferCmd mTransfer;
    StrcCmd cmd={0};
    cmd.head.len = sizeof(StrcHead);
    cmd.head.cmd = CMD_GET_VER;
    mTransfer.CmdTcp(cmd , NULL , 0 , mSocket);

}

void NetTransmission::SendCmdGetLensInfo()
{
    CTransferCmd mTransfer;
    StrcCmd cmd={0};
    cmd.head.len = sizeof(StrcHead);
    cmd.head.cmd = CMD_GET_LENS_INFO;
    mTransfer.CmdTcp(cmd , NULL , 0 , mSocket);

}

/**
 * @brief 
 *      向下位机发送查寻当前曝光参数
 * @param min 
 *      下位机返回的最小等级曝光参数
 * @param max 
 *      下位机返回的最大等级曝光参数
 * @return int 
 *      当前配置的曝光等级
 *      >=0     曝光等级
 *      <0      设置出错
 */
int NetTransmission::GetExposure(int *min, int*max)
{
    int ret = 0;
    if(mParam.ExposureCur < 0)
    {
        ret = -1;
    }else{
        ret = mParam.ExposureCur;
        *min = mParam.ExposureMin;
        *max = mParam.ExposureMax; 
    }

    return ret;
}

/**
 * @brief 
 *      向下位机设置曝光等级
 * @param level 
 *      设置的曝光等级
 * @return int 
 *      
 */
int NetTransmission::SetExposure(int level)
{
    int ret = 0;
    if(mParam.ExposureCur < 0)
    {
        ret = -1;
    }else{
        CTransferCmd mTransfer;
        StrcCmd cmd={0};
        level = level < mParam.ExposureMin ? mParam.ExposureMin:level;
        level = level > mParam.ExposureMax ? mParam.ExposureMax:level;
        cmd.head.cmd = CMD_EXPOSURE;
        cmd.head.len = sizeof(StrcSetVal);
        cmd.exposure.val = level;
        ret = mTransfer.CmdTcp(cmd , NULL , 0 , mSocket);
        if(ret >= 0)
        {
            mParam.ExposureCur = level;
        }
    }
    
    return ret;
}

int NetTransmission::SendCmdGetExposure()
{
    CTransferCmd mTransfer;
    StrcCmd cmd={0};
    cmd.head.len = sizeof(StrcExposureGet);
    cmd.head.cmd = CMD_GET_EXPOSURE;
    return mTransfer.CmdTcp(cmd , NULL , 0 , mSocket);
}