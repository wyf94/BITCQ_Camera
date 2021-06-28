#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <signal.h>
#include <unistd.h>

#include "GxIAPI.h"
#include "DxImageProc.h"

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
        GX_TRIGGER_ACTIVATION_ENTRY triggerActivation; //0: GX_TRIGGER_ACTIVATION_FALLINGEDGE 1: GX_TRIGGER_ACTIVATION_RISINGEDGE
        GX_TRIGGER_SOURCE_ENTRY triggerSource;         //0: GX_TRIGGER_SOURCE_SOFTWARE 1: GX_TRIGGER_SOURCE_LINE0 2: GX_TRIGGER_SOURCE_LINE1 3: GX_TRIGGER_SOURCE_LINE2 4: GX_TRIGGER_SOURCE_LINE3 5: GX_TRIGGER_SOURCE_COUNTER2END
        GX_TRIGGER_SELECTOR_ENTRY triggerSelector;     //1: GX_ENUM_TRIGGER_SELECTOR_FRAME_START 2:GX_ENUM_TRIGGER_SELECTOR_FRAME_BURST_START
        int64_t triggerDelay;                        //TriggerDelay Min:0.0000 Max:3000000.0000 us
        //------------End Trigger Setting------------

        //------------Exposure Setting------------
        GX_EXPOSURE_MODE_ENTRY exposureMode;      //1: GX_EXPOSURE_MODE_TIMED 2: GX_EXPOSURE_MODE_TRIGGERWIDTH
        GX_EXPOSURE_TIME_MODE_ENTRY exposureTimeMode; //0: GX_EXPOSURE_TIME_MODE_ULTRASHORT 1: GX_EXPOSURE_TIME_MODE_STANDARD
        double minAutoExposureValue; //Min:20.0000 Max:27000.0000
        double maxAutoExposureValue; //Min:20.0000 Max:1000000.0000
        GX_EXPOSURE_AUTO_ENTRY exposureAuto; //0: GX_EXPOSURE_AUTO_OFF 1: GX_EXPOSURE_AUTO_CONTINUOUS 2: GX_EXPOSURE_AUTO_ONCE
        double exposureDelayValue; //2us
        //------------End Exposure Setting------------

        //------------Desired Gray Value Setting------------
        int64_t desiredGrayValue; //Min:0 Max:255
        GX_AA_LIGHT_ENVIRMENT_ENTRY aaLightEnvirment; //0: GX_AA_LIGHT_ENVIRMENT_NATURELIGHT 1: GX_AA_LIGHT_ENVIRMENT_AC50HZ 2: GX_AA_LIGHT_ENVIRMENT_AC60HZ
        //------------End Desired Gray Value Setting------------

        //------------Frame Rate Mode Setting------------
        GX_ACQUISITION_FRAME_RATE_MODE_ENTRY acquisitionFrameRateMode; //0: GX_ACQUISITION_FRAME_RATE_MODE_OFF 1: GX_ACQUISITION_FRAME_RATE_MODE_ON
        double acquisitionFrameRate; //Min:0.1000 Max:10000.0000
        //------------End Frame Rate Mode Setting------------

        //------------Digital IO Control Setting------------
        GX_LINE_SELECTOR_ENTRY lineSelector; //0-10: GX_ENUM_LINE_SELECTOR_LINE0-10 11: GX_ENUM_LINE_SELECTOR_LINE_STROBE
        GX_LINE_MODE_ENTRY lineMode; //0: GX_ENUM_LINE_MODE_INPUT 1: GX_ENUM_LINE_MODE_OUTPUT
        bool boolLineInverter; //0: false 1: true
        GX_LINE_SOURCE_ENTRY lineSource; //0: GX_ENUM_LINE_SOURCE_OFF 1: GX_ENUM_LINE_SOURCE_STROBE 2-4: GX_ENUM_LINE_SOURCE_USEROUTPUT0-2 5: GX_ENUM_LINE_SOURCE_EXPOSURE_ACTIVE 6: GX_ENUM_LINE_SOURCE_FRAME_TRIGGER_WAIT 7: GX_ENUM_LINE_SOURCE_ACQUISITION_TRIGGER_WAIT 8(13 14): GX_ENUM_LINE_SOURCE_TIMER1(2 3)_ACTIVE 9-12: GX_ENUM_LINE_SOURCE_USEROUTPUT3-6
        GX_USER_OUTPUT_SELECTOR_ENTRY userOutputSelector; //1: GX_USER_OUTPUT_SELECTOR_OUTPUT0 2: GX_USER_OUTPUT_SELECTOR_OUTPUT1 4: GX_USER_OUTPUT_SELECTOR_OUTPUT2
        bool boolUserOutputValue; //0: false 1: true
        //------------End Digital IO Control Setting------------

        //------------Analog Control Setting------------
        GX_GAIN_SELECTOR_ENTRY gainSelector; //0: GX_GAIN_SELECTOR_ALL 1: GX_GAIN_SELECTOR_RED 2: GX_GAIN_SELECTOR_GREEN 3: GX_GAIN_SELECTOR_BLUE
        GX_GAIN_AUTO_ENTRY gainAuto; //0: GX_GAIN_AUTO_OFF 1: GX_GAIN_AUTO_CONTINUOUS 2: GX_GAIN_AUTO_ONCE
        double minAutoGainValue; //Min:0.0000 Max:23.9000
        double maxAutoGainValue; //Min:0.0000 Max:23.9000
        double gainValue; //Min:0.0000 Max:23.9000
        //------------End Analog Control Setting------------

        //------------Balance Ratio Setting------------
        GX_BALANCE_RATIO_SELECTOR_ENTRY balanceRatioSelector; //0: GX_BALANCE_RATIO_SELECTOR_RED 1: GX_BALANCE_RATIO_SELECTOR_GREEN 2: GX_BALANCE_RATIO_SELECTOR_BLUE
        GX_BALANCE_WHITE_AUTO_ENTRY balanceWhiteAuto; //0: GX_BALANCE_WHITE_AUTO_OFF 1: GX_BALANCE_WHITE_AUTO_CONTINUOUS 2: GX_BALANCE_WHITE_AUTO_ONCE
        GX_AWB_LAMP_HOUSE_ENTRY awbLampHouse; //0: GX_AWB_LAMP_HOUSE_ADAPTIVE 1: GX_AWB_LAMP_HOUSE_D65 2: GX_AWB_LAMP_HOUSE_FLUORESCENCE 3: GX_AWB_LAMP_HOUSE_INCANDESCENT 4: GX_AWB_LAMP_HOUSE_D75 5: GX_AWB_LAMP_HOUSE_D50 6: GX_AWB_LAMP_HOUSE_U30
        GX_LIGHT_SOURCE_PRESET_ENTRY lightSourcePreset; //0: GX_LIGHT_SOURCE_PRESET_OFF 1: GX_LIGHT_SOURCE_PRESET_CUSTOM 2: GX_LIGHT_SOURCE_PRESET_DAYLIGHT_6500K 3: GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K 4: GX_LIGHT_SOURCE_PRESET_COOL_WHITE_FLUORESCENCE 5: GX_LIGHT_SOURCE_PRESET_INCA
        double balanceRatio; //Min:1.0000 Max:7.9961
        //------------End Balance Ratio Setting------------

        //------------Mining Parameter Setting------------
        int64_t acquisitionBufferNum; //5  Acquisition Buffer Qty.
        int64_t acquisitionTransferSize; //(64 * 1024)   Size of data transfer block
        int64_t acquisitionTransferNumberURB; //64  Qty. of data transfer block
        //------------End Mining Parameter Setting------------
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
        cameraSetting.triggerSwitch = GX_TRIGGER_SWITCH_ON;
        cameraSetting.triggerActivation = GX_TRIGGER_ACTIVATION_FALLINGEDGE;
        cameraSetting.triggerSource = GX_TRIGGER_SOURCE_LINE2;
        cameraSetting.triggerSelector = GX_ENUM_TRIGGER_SELECTOR_FRAME_START;
        cameraSetting.triggerDelay = 0;
        //------------End Trigger Setting------------

        //------------Exposure Setting------------
        cameraSetting.exposureMode = GX_EXPOSURE_MODE_TIMED;
        cameraSetting.exposureTimeMode = GX_EXPOSURE_TIME_MODE_STANDARD;
        cameraSetting.minAutoExposureValue = 20;
        cameraSetting.maxAutoExposureValue = 27000;
        cameraSetting.exposureAuto = GX_EXPOSURE_AUTO_CONTINUOUS;
        cameraSetting.exposureDelayValue = 2.0;
        //------------End Exposure Setting------------

        //------------Desired Gray Value Setting------------
        cameraSetting.desiredGrayValue = 120;
        cameraSetting.aaLightEnvirment = GX_AA_LIGHT_ENVIRMENT_AC50HZ;
        //------------End Desired Gray Value Setting------------

        //------------Frame Rate Mode Setting------------
        cameraSetting.acquisitionFrameRateMode = GX_ACQUISITION_FRAME_RATE_MODE_ON;
        cameraSetting.acquisitionFrameRate = 36;
        //------------End Frame Rate Mode Setting------------

        //------------Digital IO Control Setting------------
        cameraSetting.lineSelector = GX_ENUM_LINE_SELECTOR_LINE2;
        cameraSetting.lineMode = GX_ENUM_LINE_MODE_INPUT;
        cameraSetting.boolLineInverter = true;
        cameraSetting.lineSource = GX_ENUM_LINE_SOURCE_USEROUTPUT0;
        cameraSetting.userOutputSelector = GX_USER_OUTPUT_SELECTOR_OUTPUT0;
        cameraSetting.boolUserOutputValue = true;
        //------------End Digital IO Control Setting------------

        //------------Analog Control Setting------------
        cameraSetting.gainSelector = GX_GAIN_SELECTOR_ALL;
        cameraSetting.gainAuto = GX_GAIN_AUTO_CONTINUOUS;
        cameraSetting.minAutoGainValue = 0; 
        cameraSetting.maxAutoGainValue = 23.9;
        cameraSetting.gainValue = 6;
        //------------End Analog Control Setting------------

        //------------Balance Ratio Setting------------
        cameraSetting.balanceRatioSelector = GX_BALANCE_RATIO_SELECTOR_RED;
        cameraSetting.balanceWhiteAuto = GX_BALANCE_WHITE_AUTO_CONTINUOUS;
        cameraSetting.awbLampHouse = GX_AWB_LAMP_HOUSE_FLUORESCENCE;
        cameraSetting.lightSourcePreset = GX_LIGHT_SOURCE_PRESET_DAYLIGHT_6500K;
        cameraSetting.balanceRatio = 1;
        //------------End Balance Ratio Setting------------

        //------------Mining Parameter Setting------------
        cameraSetting.acquisitionBufferNum = 5;
        cameraSetting.acquisitionTransferSize = 64 * 1024;
        cameraSetting.acquisitionTransferNumberURB = 64;
        //------------End Mining Parameter Setting------------
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