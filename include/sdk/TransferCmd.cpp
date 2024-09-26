#include "sdk/TransferCmd.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include "sdk/prot_tcp/prot_tcp.h"
#ifdef DEBUG
#include <iostream>   
#endif
// Most of the funcs are not used for server

CTransferCmd::CTransferCmd()
{
}

CTransferCmd::~CTransferCmd()
{
}

/**
 * @brief CTransferCmd::CmdTcp
 *      发送指令的接口（TCP连接）#include <sys/socket.h>
#include <sys/types.h>
 * @param cmd
 *      需要发送的指令的相关信息
 * @param data
 *      指令需要的附加信息
 * @param len
 *      length of data
 * @param fd
 *      socket id
 * @return
 *      ==0     success
 *      !=0     failed
 */
int CTransferCmd::CmdTcp(StrcCmd cmd, void *data , int len , int fd)
{
    int ret = 0;
    if(len == 0)
    {
        ret = SendCmd(cmd, fd);
    }else{
        switch (cmd.head.cmd) {
            case CMD_LOGIN:
                ret = Login(cmd.login.type, fd);
                break;
            case CMD_GAIN:
                ret = Gain(cmd, fd);
                break;
            case CMD_GET_VER:
            case CMD_GET_LENS_INFO:
                ret = SendGetInfo(cmd , fd);
                break;
            case CMD_EXPOSURE:
                ret = SendSetExporsure(cmd , fd);
                break;
            case CMD_GET_EXPOSURE:
                ret = SendGetExporsure(cmd , fd);
                break;
            default:
                ret = -1;
                break;
        }
    }
    return ret ;
}

int CTransferCmd::Login(int type , int fd)
{
    int ret = 0;
    StrcCmd cmd;
    cmd.head.len = sizeof(StrcLogin);
    cmd.head.cmd = CMD_LOGIN;
    cmd.login.type = type;
    ret = SendCmd(cmd , fd);
    return ret;
}

/**
 * @brief CTransferCmd::Gain
 * @param cmd
 * @param fd
 * @return
 */
int CTransferCmd::Gain(StrcCmd cmd , int fd)
{
    int ret = 0;
    ret |= SendTagOfStreamHead(fd);
    ret |= send(fd, (void *)&cmd , sizeof(StrcGain), 0);
    ret |= SendTagOfStreamEnd(fd);
    ret = ret > 0?0:-1;
    return ret ;
}

/**
 * @brief CTransferCmd::SendCmd
 * @param cmd
 * @param fd
 * @return
 */
int CTransferCmd::SendCmd(StrcCmd cmd , int fd)
{
    int ret = 0;

    ret |= SendTagOfStreamHead(fd);
    ret |= send(fd, (void *)&cmd , cmd.head.len, 0);
    ret |= SendTagOfStreamEnd(fd);
    ret = ret > 0?0:-1;
    return ret ;
}

/**
 * @brief CTransferCmd::EndStream
 *      send end of stram.
 * @param fd
 * @return
 */
int CTransferCmd::SendTagOfStreamEnd(int fd)
{
    char end = DATA_STREAM_END;
    return send(fd , (void *)&end , sizeof(end) , 0 );
}
/**
 * @brief CTransferCmd::HeadStream
 *      send head of stram.
 * @param fd
 * @return
 */
int CTransferCmd::SendTagOfStreamHead(int fd)
{
    char end = DATA_STREAM_HEAD;
    return send(fd , (void *)&end , sizeof(end) , 0 );
}

/**
 * @brief ReceiveTagOfStreamHead
 *      接收数据流头标志
 * @param fdSocket
 *      SOCKET FD
 * @return
 *      ==0     接收正确
 *      !=0     接收出错
 */
int CTransferCmd::ReceiveTagOfStreamHead(int fdSocket)
{
    int ret = 0 , len;
    uint8_t buff[10];
    len = recv(fdSocket , (void *)buff , 1, MSG_WAITALL);
    if(len <= 0)
    {
        ret = -1;
    }else{
        if(buff[0] != DATA_STREAM_HEAD)
        {
            ret = -2;
        }else{
            ret = 0;
        }
    }
    return ret ;
}

/**
 * @brief ReceiveTagOfStreamEnd
 *      接收数据流尾标志
 * @param fdSocket
 *      SOCKET FD
 * @return
 *      ==0     接收正确
 *      !=0     接收出错
 */
int CTransferCmd::ReceiveTagOfStreamEnd(int fdSocket)
{
    int ret = 0 , len;
    uint8_t buff[10];
    len = recv(fdSocket , (void *)buff , 1, MSG_WAITALL);
    if(len <= 0)
    {
        ret = -1;
    }else{
        if(buff[0] != DATA_STREAM_END)
        {
            ret = -2;
        }else{
            ret = 0;
        }
    }
    return ret ;
}

/**
 * @brief 
 *      向客户端发送心跳
 * @param fdSocket 
 *      handle of socket
 * @return int 
 *      ==0     success
 *      !=0     failed
 */
int CTransferCmd::SendTick(int fdSocket)
{
    StrcCmd cmd;
    cmd.head.len = sizeof(StrcHead);
    cmd.head.cmd = CMD_TICK ;
    return SendCmd(cmd , fdSocket);
}

int CTransferCmd::SendGetInfo(StrcCmd cmd , int fdSocket)
{
    #ifdef DEBUG
    std::cout << "cmd:" << cmd.head.cmd << std::endl;
    #endif
    cmd.head.len = sizeof(StrcHead);
    return SendCmd(cmd , fdSocket);
}


int CTransferCmd::SendSetExporsure(StrcCmd cmd , int fdSocket)
{
    cmd.head.len = sizeof(StrcSetVal);
    return SendCmd(cmd , fdSocket);
}

int CTransferCmd::SendGetExporsure(StrcCmd cmd , int fd)
{
    cmd.head.len = sizeof(StrcExposureGet);
    return SendCmd(cmd , fd);
}