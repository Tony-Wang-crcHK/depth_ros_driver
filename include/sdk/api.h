#pragma once
#include <stdint.h>

#define EXPORT __attribute__ ((visibility("default")))

typedef enum {
    STATUS_SUCCESS,
    STATUS_ERROR,
    STATUS_SOCKET_FAILED,           
    STATUS_SOCKET_CONNECT_FAILED,   
    STATUS_SOCKET_SEND_FAILED,      
}ERROR_CODE;

/**
 * @brief Image type.
 */
typedef enum {
    IMG_DEPTH,
    IMG_AMPLITUDE,
    IMG_RGB,
}IMG_TYPE;

/**
 * @brief Image data.
 */
typedef struct {
    IMG_TYPE type;
    int len;
    uint16_t *data;
}STRC_IMG;

typedef struct {
    STRC_IMG img_rgb;
    STRC_IMG img_amplitude;
    STRC_IMG img_depth;
}STRC_IMG_ALL;

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief SDK initialization
 */
void api_init();

/**
 * @brief 
 *      exit SDK
 */
void api_exit();

/**
 * @brief 
 *      Connect to device.
 * @param ip 
 *      IP of the device.
 * @param port
 *      Port of the device. 
 * @return int 
 *      <0  failed
 *      >=0 success,The handle of the device.
 */
int api_connect(char *ip , int port=65300);

/**
 * @brief 
 *      Disconnect from the device.      
 * @param handle 
 *      The handle of the device.
 * @return int 
 *      >=0     success
 *      <0      failed
 */
int api_disconnect(int handle);

/**
 * @brief 
 *      Get the version of the SDK
 * @return char* 
 *      The version of the SDK
 */
char *api_get_sdk_ver();

/**
 * @brief 
 *      Get the version of the device.
 * @param handle 
 *      The handle of the device.
 * @return char* 
 *      ==NULL  failed
 *      !=NULL  the version of the device
 */
char *api_get_dev_ver(int handle);

/**
 * @brief 
 *      Get the camera intrinsic parameters.
 * @param handle 
 *      The handle of the device.
 * @return char* 
 *      ==NULL  failed
 *      !=NULL  The camera intrinsic parameters.
 */
char *api_get_intrinsic_parameters(int handle);

/**
 * @brief 
 *      Get image.
 * @param handle 
 *      The handle of the device.
 * @return STRC_IMG_ALL* 
 *      ==NULL     No data.
 *      !=NULL     Data of the image.
 * 
 */
STRC_IMG_ALL *api_get_img(int handle );

/**
 * @brief 
 *      Get the last error code.
 * @return ERROR_CODE 
 */
ERROR_CODE api_errorcode();

/**
 * @brief 
 *      Gets the current camera exposure level
 * @param handle 
 *      The handle of the device.
 * @param min
 *      Minimum exposure level returned
 * @param max
 *      Maximum exposure level returned
 * @return int 
 *      >=0 The current exposure level
 *      <0  failed
 *      ==-1    Initialization incomplete
 *      
 */
int api_get_exposure(int handle , int *min , int *max);

/**
 * @brief 
 *      Set exposure level.
 * @param handle 
 *      The handle of the device.
 * @param exposure 
 *      exposure level
 * @return int 
 *      >=0 success
 *      <0  failed
 */
int api_set_exposure(int handle , int exposure);

#ifdef __cplusplus
}
#endif
