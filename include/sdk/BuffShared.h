#pragma once

#include <stdlib.h>

class CBuffShared
{

public:
    CBuffShared();
    ~CBuffShared();
public:
    enum STATUS{
        FREE,       //数据无效
        READING,    //当前正在读取,UI线程正在使用
        WRITING,    //当前正在写入,正在解码MIPI数据，使用当前缓存
        DONE        //数据已经准备完成
    };
    int Init(void **data , int size);
    void *GetBuff(STATUS status);     
    void CleanData();
    bool IsFull();

private:
    struct StrcData
    {
        int status;
        void *data;
    };
    StrcData *m_pData;
    int m_NumItem ; //管理几个数据空间
    int m_IndexR, m_IndexW;
};

