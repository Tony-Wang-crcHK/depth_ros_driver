#include "sdk/Thread.h"
#include <signal.h>
#include <errno.h>

CThread::CThread()
{
    m_HThread = 0;
}

CThread::~CThread()
{

}



int CThread::Start(FuncLoop func, void *param)
{
    int ret = 0;
    /*
    if( m_HThread != 0 && pthread_kill(m_HThread , 0) == 0  )
    {
        pthread_cancel(m_HThread);
    }
    */
    ret = pthread_create(&m_HThread , nullptr , func, param);
    return ret ;
}

int CThread::Stop()
{
    int ret = 0;
    
    return ret ;
}

