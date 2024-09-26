#include "sdk/api.h"
#include <iostream>
#include <unistd.h>
#include "sdk/BuffShared.h"
#include "sdk/BuffShared.cpp"
#include <sys/time.h>

using namespace std;

void test_buff_shared()
{
    // 创建buffshare变量
    CBuffShared mBuff;
    void *buff[4];
    char mCharBuff[4][20];
    void *p = NULL;
    int numAdd = 0, numGet = 0;

    for(int i = 0; i< 4 ; i++)
    {
        buff[i] = (void *)mCharBuff[i];
    }

    // 初始化缓冲区
    mBuff.Init(buff , 4);

    for( int i = 0; i<20; i++)
    {
        for(int j=0; j<2;j++)
        {
            if(NULL == mBuff.GetBuff(CBuffShared::WRITING))
            {
                printf("getbuff failed. \n\r");
                break;
            }
            numAdd++;
        }

        p = mBuff.GetBuff(CBuffShared::READING);
        if(p == NULL)
        {
            printf("get reading buff failed!\n\r");
            if(mBuff.IsFull())
            {
                printf("is full \n\r");
            }
        }else{
            numGet++ ;
        }
        // if(NULL == mBuff.GetBuff(CBuffShared::WRITING))
        // {
        //     cout << "get writing buff failed." << endl;

        // }
        
        // cout << "add i:" << i ;
        
        // if( mBuff.IsFull())
        // {
        //     int j = 0;
        //     cout << " FULL" << endl;
        //     while(mBuff.IsFull())
        //     {
                
        //         mBuff.GetBuff(CBuffShared::READING);
        //         cout << "GetBuff times: " << j++ <<endl;
        //     }
        // }else {
        //     cout << " no full " << endl;
        // }
    }
    printf("addNum:%d    getNum:%d\n\r", numAdd , numGet);
}

void test()
{
    int DevID = 0;
    struct timeval startTime , endTime;
    STRC_IMG_ALL *pImg;
    int mNumFrame = 0;
    char *pInfo = NULL;
    api_init();
    
    cout << "SDK VER:" << api_get_sdk_ver() << endl;

    //connect
    DevID = api_connect((char *)"192.168.10.100" );
    cout << "DevID:" << DevID<< endl;
    if(DevID < 0)
    {
        cout << "api_connect failed." << endl;
        goto END;
    }
    sleep(1);
    pInfo = api_get_dev_ver(DevID);
    
    if(pInfo == NULL)
    {
        cout << "get Sdk version failed." << endl;
    }else{
        cout << "SDK VER:" << pInfo << endl; 

    }

    pInfo = api_get_intrinsic_parameters(DevID);
    
    if(pInfo == NULL)
    {
        cout << "get lens failed." << endl;
    }else{
        cout << "lens info:" << pInfo << endl; 

    }
    
    while((pImg = api_get_img(DevID) ) == NULL);
    gettimeofday(&startTime , NULL);
    while(mNumFrame < 60)
    {
        if( (pImg = api_get_img(DevID)) != NULL)
        {
            mNumFrame++;
        }
    }
    gettimeofday(&endTime , NULL);
    cout << "mNumFrame:" << mNumFrame << endl;
    cout << "sec:" << endTime.tv_sec - startTime.tv_sec << endl;
    cout << "usec:" << endTime.tv_usec - startTime.tv_usec << endl;

    cout << "fps: " << (int)(60/(endTime.tv_sec - startTime.tv_sec)) << endl;


    sleep(3);

END:
    api_exit();    
}

int main()
{
    int ret = 0;
    // test_buff_shared();
    test();
    return ret;
}
