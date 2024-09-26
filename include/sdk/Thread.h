#pragma once
#include <pthread.h>

typedef void* (*FuncLoop)(void*); 

class CThread
{
    public:
        CThread();
        ~CThread();

    private:
        pthread_t m_HThread;
    public:
        int Start(FuncLoop func, void *param);
        int Stop();

};
