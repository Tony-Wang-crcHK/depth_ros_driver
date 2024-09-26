#include "sdk/api.h"
#include <map>
#include "sdk/nettransmission.h"
#include <unistd.h>
#include <iostream>

static ERROR_CODE inside_error_code = STATUS_SUCCESS;

struct STRC_DEV {
    NetTransmission *ctl;   //控件通道
    NetTransmission *img;   //图像传输通道
};

static map<int , STRC_DEV*> mMapDev;
static int mIndexDev = 0;

EXPORT void api_init()
{
    mIndexDev = 0;
}

EXPORT void api_exit()
{
    if(mMapDev.size() != 0)
    {
        map<int , STRC_DEV*>::iterator item;
        for(item = mMapDev.begin() ; item != mMapDev.end(); item++)
        {
            STRC_DEV *pDev = (STRC_DEV *)item->second;
            pDev->ctl->Stop();
            delete pDev->ctl;
            pDev->img->Stop();
            delete pDev->img;

            free(pDev);
        }
        mMapDev.clear();
        mIndexDev = 0;
    }
}

EXPORT ERROR_CODE api_errorcode()
{
    return inside_error_code;
}

EXPORT void api_set_errorcode(ERROR_CODE code)
{
    inside_error_code = code;
}

EXPORT int api_connect(char *ip , int port)
{
    int mDev = 0;
    STRC_DEV *pDev = (STRC_DEV *)malloc(sizeof(STRC_DEV));
    pDev->ctl = new NetTransmission();
    if(pDev->ctl->Start(string(ip) , port, NetTransmission::PIPE_CONTROL) >=0)
    {
        
        pDev->img = new NetTransmission();
        if(pDev->img->Start(string(ip) , port, NetTransmission::PIPE_IMG_SEPARATE) >=0)
        {
            mIndexDev++;
            mDev = mIndexDev;
            mMapDev.insert({mDev , pDev});
        }else{
            mDev = -1;
            pDev->ctl->Stop();
            delete pDev->img;
            delete pDev->ctl;
            free(pDev);           
        }
        
    }else{
        mDev = -1;
        delete pDev->ctl;
        free(pDev);
    }
    

    return mDev;
}

EXPORT int api_disconnect(int handle)
{
    int ret = 0;
    auto item = mMapDev.find(handle);
    if(item == mMapDev.end())
    {
        ret = -1;
    }else{
        STRC_DEV *pDev = item->second;
        pDev->ctl->Stop();
        delete pDev->ctl;
        pDev->img->Stop();
        delete pDev->img;        
        free(pDev);
        mMapDev.erase(handle);
    }
    return ret;
}

EXPORT char *api_get_sdk_ver()
{
    return VER;
}

EXPORT char *api_get_dev_ver(int handle)
{
    auto item = mMapDev.find(handle);
    if(item != mMapDev.end())
    {
        STRC_DEV *pDev = item->second;
        return pDev->ctl->GetSdkVer();
    }

    return NULL;
}

EXPORT char *api_get_intrinsic_parameters(int handle)
{
    auto item = mMapDev.find(handle);
    if(item != mMapDev.end())
    {
        STRC_DEV *pDev = item->second;
        return pDev->ctl->GetLensInfo();
    }

    return NULL;    
}

EXPORT STRC_IMG_ALL *api_get_img(int handle )
{
    STRC_IMG_ALL *pImg = NULL;
    auto item = mMapDev.find(handle);
    if(item != mMapDev.end())
    {
        STRC_DEV *pDev = item->second;
        pImg = pDev->img->GetImg();
    }
    return pImg;
}

EXPORT int api_get_exposure(int handle , int *min , int *max)
{
    auto item = mMapDev.find(handle);
    if(item != mMapDev.end())
    {
        STRC_DEV *pDev = item->second;
        return pDev->ctl->GetExposure(min , max);
    }
    return -1;
}

EXPORT int api_set_exposure(int handle , int exposure)
{
    auto item = mMapDev.find(handle);
    if(item != mMapDev.end())
    {
        STRC_DEV *pDev = item->second;
        return pDev->ctl->SetExposure(exposure);
    }
    return -1;

}