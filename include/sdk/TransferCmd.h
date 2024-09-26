#ifndef TRANSFERCMD_H
#define TRANSFERCMD_H
#include "sdk/prot_tcp/prot_tcp.h"


class CTransferCmd
{
public:
    CTransferCmd();
    ~CTransferCmd();

public:
    int CmdTcp(StrcCmd cmd, void *data , int len , int fd);
    int ReceiveTagOfStreamHead(int fdSocket);
    int ReceiveTagOfStreamEnd(int fdSocket);
    int Login(int type , int fd);
    int SendTick(int fdSocket);


private:
    
    int Gain(StrcCmd cmd , int fd);
    int SendCmd(StrcCmd cmd , int fd);
    int SendGetInfo(StrcCmd cmd , int fd);
    int SendSetExporsure(StrcCmd cmd , int fd);
    int SendGetExporsure(StrcCmd cmd , int fd);
    int SendTagOfStreamHead(int fd);
    int SendTagOfStreamEnd(int fd);

};


#endif // TRANSFERCMDTCP_H
