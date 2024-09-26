#include "sdk/BuffShared.h"

CBuffShared::CBuffShared()
{
    m_NumItem = 0;
    m_IndexR = -1;
    m_IndexW = -1;
}

CBuffShared::~CBuffShared()
{
    if(m_pData != nullptr)
    {
        free(m_pData);
    }
}

void CBuffShared::CleanData()
{

    m_IndexR = -1;
    m_IndexW = -1;    
    for(int i = 0; i< m_NumItem ;i++)
    {
        m_pData[i].status = FREE;
    }
}

/**
 * @brief 
 *  初始化
 * @param data
 *  需要管理的数据 
 * @param size 
 *  data的数量
 * @return int
 *      >=0     success
 *      <0      failed 
 */
int CBuffShared::Init(void **data , int size)
{
    int ret = 0;
    if(size < 3)
    {
        return -1;
    }
    m_pData = (StrcData *)malloc(size*sizeof(StrcData));
    if(m_pData == nullptr)
    {
        ret = -1;
    }else{
        for(int i = 0; i< size ;i++)
        {
            m_pData[i].status = FREE;
            m_pData[i].data = data[i];
        }
    }
    m_NumItem = size;
    return ret;
}

/**
 * @brief 
 *      获取供指定状态使用的BUFFER
 * @param status 
 *      指定的状态
 * @return void* 
 *      ==null  获取失败
 *      !=null  返回数据指针
 */
void *CBuffShared::GetBuff(STATUS status)
{
    void *ret = nullptr;
    int mTmpIndex = 0;
    if(m_NumItem <=0)
    {
        return nullptr;
    }
    if(status == READING)
    {
        mTmpIndex = (m_IndexR +1)%m_NumItem;
        if(m_pData[mTmpIndex].status == DONE)
        {
            if(m_IndexR >= 0)
            {
                m_pData[m_IndexR].status = FREE;
            }
            m_IndexR = mTmpIndex;
            m_pData[m_IndexR].status = READING;
            ret = m_pData[m_IndexR].data;
        }
    }else if( status == WRITING)
    {
        mTmpIndex = (m_IndexW +1)%m_NumItem;
        if(m_pData[mTmpIndex].status == FREE)
        {
            if(m_IndexW >= 0)
            {
                m_pData[m_IndexW].status = DONE;
            }
            m_IndexW = mTmpIndex;
            m_pData[m_IndexW].status = WRITING;
            ret = m_pData[m_IndexW].data;
        }
    }
    return ret;
}

bool CBuffShared::IsFull()
{
    bool ret = true;
    int mTmpIndex = 0;
    mTmpIndex = (m_IndexW +1)%m_NumItem;
    if(m_pData[mTmpIndex].status == FREE)
    {
        ret = false;
    }

    return ret;
}