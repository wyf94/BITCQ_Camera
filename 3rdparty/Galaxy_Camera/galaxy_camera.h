#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <signal.h>
#include <unistd.h>

#include "GxIAPI.h"
#include "DxImageProc.h"

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block

//Show error message
#define GX_VERIFY(emStatus)            \
    if (emStatus != GX_STATUS_SUCCESS) \
    {                                  \
        GetErrorString(emStatus);      \
        return;                         \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus)       \
    if (emStatus != GX_STATUS_SUCCESS) \
    {                                  \
        GetErrorString(emStatus);      \
        GXCloseDevice(hDevice);      \
        hDevice = NULL;              \
        GXCloseLib();                  \
        printf("<App Exit!>\n");       \
        return;                         \
    }

class galaxy_camera
{
private:
    std::string address;
    int width;
    int height;
    int fps;
    std::string format;
    int collectionMode = 4; // 1:DQBuf(Linux) 2:DQAllBuf(Linux) 3.Callback 4.GetImage
    cv::Mat image;
    bool bColorFilter = false;

    GX_STATUS status;
    GX_DEV_HANDLE hDevice = NULL; ///< Device handle
    GX_OPEN_PARAM openParam; //Struct about device open param. pszContent: Camera Num; openMode: How to open device; accessMode: Way to access device.
    GX_FRAME_DATA gFrameData; 

public:
    galaxy_camera(std::string addr, int w, int h, int f, std::string fm) : address(addr), width(w), height(h), fps(f), format(fm) 
    {
        image = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
        gFrameData = { 0 }; 
        status = GX_STATUS_SUCCESS;
    }

    void OpenDevice();
    cv::Mat GetImage();
    void GetErrorString(GX_STATUS);
    void CloseDevice();
    void FunctionSetting();
    void GetDeviceInfo();
    
    void DQBufInit();
    void DQAllBufInit();
    void CallbackInit();
    void GetImageInit();
};