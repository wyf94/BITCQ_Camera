#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <signal.h>
#include <unistd.h>

#include "GxIAPI.h"
#include "DxImageProc.h"

#define ACQ_BUFFER_NUM 5              ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE (64 * 1024) ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64    ///< Qty. of data transfer block

//Show error message
#define GX_VERIFY(emStatus)            \
    if (emStatus != GX_STATUS_SUCCESS) \
    {                                  \
        GetErrorString(emStatus);      \
        return;                        \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus)       \
    if (emStatus != GX_STATUS_SUCCESS) \
    {                                  \
        GetErrorString(emStatus);      \
        GXCloseDevice(hDevice);        \
        hDevice = NULL;                \
        GXCloseLib();                  \
        printf("<App Exit!>\n");       \
        return;                        \
    }

class galaxy_camera
{
private:
    typedef struct
    {
        GX_ACQUISITION_MODE_ENTRY acquisitionMode; //0: GX_ACQ_MODE_SINGLE_FRAME 1: GX_ACQ_MODE_MULITI_FRAME 2: GX_ACQ_MODE_CONTINUOUS

        //------------Trigger Setting------------
        GX_TRIGGER_SWITCH_ENTRY triggerSwitch;       //0: GX_TRIGGER_SWITCH_OFF 1: GX_TRIGGER_SWITCH_ON
        GX_TRIGGER_ACTIVATION_ENTRY activationEntry; //0: GX_TRIGGER_ACTIVATION_FALLINGEDGE 1: GX_TRIGGER_ACTIVATION_RISINGEDGE
        GX_TRIGGER_SOURCE_ENTRY sourceEntry;         //0: GX_TRIGGER_SOURCE_SOFTWARE 1: GX_TRIGGER_SOURCE_LINE0 2: GX_TRIGGER_SOURCE_LINE1 3: GX_TRIGGER_SOURCE_LINE2 4: GX_TRIGGER_SOURCE_LINE3 5: GX_TRIGGER_SOURCE_COUNTER2END
        GX_TRIGGER_SELECTOR_ENTRY selectorEntry;     //1: GX_ENUM_TRIGGER_SELECTOR_FRAME_START 2:GX_ENUM_TRIGGER_SELECTOR_FRAME_BURST_START
        int64_t triggerDelay;                        //TriggerDelay Min:0.0000 Max:3000000.0000 us
        //------------End Trigger Setting------------

        //------------Exposure Setting------------
        GX_EXPOSURE_MODE_ENTRY exposureModeEntry;      //1: GX_EXPOSURE_MODE_TIMED 2: GX_EXPOSURE_MODE_TRIGGERWIDTH
        GX_EXPOSURE_TIME_MODE_ENTRY exposureTimeEntry; //0: GX_EXPOSURE_TIME_MODE_ULTRASHORT 1: GX_EXPOSURE_TIME_MODE_STANDARD
        double minShutterRange; //Min:20.0000 Max:27000.0000
        double maxShutterRange; //Min:20.0000 Max:1000000.0000
        GX_EXPOSURE_AUTO_ENTRY exposureAutoEntry; //0: GX_EXPOSURE_AUTO_OFF 1: GX_EXPOSURE_AUTO_CONTINUOUS 2: GX_EXPOSURE_AUTO_ONCE
        double exposureDelayValue; //2us
        //------------End Exposure Setting------------

        //------------Desired Gray Value Setting------------
        int64_t desiredGrayValue;
        //------------End Desired Gray Value Setting------------
    } CameraSetting;

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
    GX_OPEN_PARAM openParam;      //Struct about device open param. pszContent: Camera Num; openMode: How to open device; accessMode: Way to access device.
    GX_FRAME_DATA gFrameData;
    CameraSetting cameraSetting;

public:
    galaxy_camera(std::string addr, int w, int h, int f, std::string fm) : address(addr), width(w), height(h), fps(f), format(fm)
    {
        image = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
        gFrameData = {0};
        status = GX_STATUS_SUCCESS;

        cameraSetting.acquisitionMode = GX_ACQ_MODE_CONTINUOUS;

        //------------Trigger Setting------------
        cameraSetting.triggerSwitch = GX_TRIGGER_SWITCH_OFF;
        cameraSetting.activationEntry = GX_TRIGGER_ACTIVATION_FALLINGEDGE;
        cameraSetting.sourceEntry = GX_TRIGGER_SOURCE_LINE2;
        cameraSetting.selectorEntry = GX_ENUM_TRIGGER_SELECTOR_FRAME_START;
        cameraSetting.triggerDelay = 0;
        //------------End Trigger Setting------------

        //------------Exposure Setting------------
        cameraSetting.exposureModeEntry = GX_EXPOSURE_MODE_TIMED;
        cameraSetting.exposureTimeEntry = GX_EXPOSURE_TIME_MODE_STANDARD;
        cameraSetting.minShutterRange = 20;
        cameraSetting.maxShutterRange = 27000;
        cameraSetting.exposureAutoEntry = GX_EXPOSURE_AUTO_CONTINUOUS;
        cameraSetting.exposureDelayValue = 2.0;
        //------------End Exposure Setting------------

        //------------Desired Gray Value Setting------------
        cameraSetting.desiredGrayValue = 120;
        //------------End Desired Gray Value Setting------------
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