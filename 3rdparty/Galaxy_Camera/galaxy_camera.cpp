#include "galaxy_camera.h"

void galaxy_camera::OpenDevice()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t deviceNum = 0;

    //Init Library
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }

    //Get device enumerated number
    status = GXUpdateDeviceList(&deviceNum, 1000);
    printf("Device Num: %d\n", deviceNum);
    if (status != GX_STATUS_SUCCESS || deviceNum <= 0)
    {
        GetErrorString(status);
        GXCloseLib();
    }

    switch (collectionMode)
    {
    case 1:
        DQBufInit();
        break;
    case 2:
        DQAllBufInit();
        break;
    case 3:
        CallbackInit();
        break;
    case 4:
        GetImageInit();
        break;
    default:
        printf("Collection Mode set wrong!\n");
        break;
    }
}

void galaxy_camera::DQBufInit()
{
    //Open first device enumerated
    status = GXOpenDeviceByIndex(1, &hDevice);
    GX_VERIFY_EXIT(status);
}

void galaxy_camera::DQAllBufInit()
{
}

void galaxy_camera::CallbackInit()
{
}

void galaxy_camera::GetImageInit()
{
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;
    openParam.openMode = GX_OPEN_INDEX;
    openParam.pszContent = (char *)"1";

    //Open Device
    status = GXOpenDevice(&openParam, &hDevice);
    GX_VERIFY_EXIT(status);

    GetDeviceInfo();

    FunctionSetting();

    int64_t payload_size = 0;
    status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &payload_size);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        GX_VERIFY_EXIT(status);
    }

    printf("payload_size: %ld\n", payload_size);

    gFrameData.pImgBuf = malloc(payload_size);
    if (gFrameData.pImgBuf == NULL)
    {
        printf("<Failed to allot memory>\n");
        return;
    }

    //发送开采命令
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        GX_VERIFY_EXIT(status);
    }
}

cv::Mat galaxy_camera::GetImage()
{
    status = GXGetImage(hDevice, &gFrameData, 100);
    if (status == GX_STATUS_SUCCESS)
    {
        if (gFrameData.nStatus == 0)
        {
            char *rgb = new char[gFrameData.nWidth * gFrameData.nHeight * 3];
            memcpy(image.data, gFrameData.pImgBuf, gFrameData.nWidth * gFrameData.nHeight);
            DxRaw8toRGB24(gFrameData.pImgBuf, rgb, gFrameData.nWidth, gFrameData.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
            memcpy(image.data, rgb, gFrameData.nWidth * gFrameData.nHeight * 3);
        }
    }
    return image;
}

void galaxy_camera::FunctionSetting()
{
    //------------Trigger Setting------------
    //Set acquisition mode
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, cameraSetting.acquisitionMode);
    GX_VERIFY_EXIT(status);

    //Set trigger switch
    status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, cameraSetting.triggerSwitch);
    GX_VERIFY_EXIT(status);
    status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_ACTIVATION, cameraSetting.triggerActivation);
    GX_VERIFY_EXIT(status);
    status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, cameraSetting.triggerSource);
    GX_VERIFY_EXIT(status);
    status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SELECTOR, cameraSetting.triggerSelector);
    GX_VERIFY_EXIT(status);
    status = GXSetFloat(hDevice, GX_FLOAT_TRIGGER_DELAY, cameraSetting.triggerDelay);
    GX_VERIFY_EXIT(status);

    //Get rising edge filter setting range
    GX_FLOAT_RANGE raisingRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_TRIGGER_FILTER_RAISING, &raisingRange);
    //Set minimum value of the rising edge filter
    status = GXSetFloat(hDevice, GX_FLOAT_TRIGGER_FILTER_RAISING, raisingRange.dMin);
    //Set maximum value of the rising edge filter
    status = GXSetFloat(hDevice, GX_FLOAT_TRIGGER_FILTER_RAISING, raisingRange.dMax);
    //Get falling edge filter setting range
    GX_FLOAT_RANGE fallingRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_TRIGGER_FILTER_FALLING, &fallingRange);
    //Set minimum value of the falling edge filter
    status = GXSetFloat(hDevice, GX_FLOAT_TRIGGER_FILTER_FALLING, fallingRange.dMin);
    //Set maximum value of the falling edge filter
    status = GXSetFloat(hDevice, GX_FLOAT_TRIGGER_FILTER_FALLING, fallingRange.dMax);
    //------------End Trigger Setting------------

    //------------Exposure Setting------------
    //Set exposure mode
    status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_MODE, cameraSetting.exposureMode);
    GX_VERIFY_EXIT(status);
    //Set exposure time mode
    // status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_TIME_MODE, cameraSetting.exposureTimeMode);
    // GX_VERIFY_EXIT(status);

    //Set exposure delay
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_DELAY, cameraSetting.exposureDelayValue);

    if(cameraSetting.exposureMode == GX_EXPOSURE_MODE_TIMED)
    {
        //Set continuous auto exposure
        status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, cameraSetting.exposureAuto);
        GX_VERIFY_EXIT(status);

        if(cameraSetting.exposureAuto == GX_EXPOSURE_AUTO_OFF)
        {
            //Set exposure value
            status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, cameraSetting.maxAutoExposureValue);
        }
        else
        {
            //Set minimum auto exposure value
            status = GXSetFloat(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, cameraSetting.minAutoExposureValue); 
            GX_VERIFY_EXIT(status);
            //Set maximum auto exposure value
            status = GXSetFloat(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, cameraSetting.maxAutoExposureValue); 
            GX_VERIFY_EXIT(status);
        }
    }
    //------------End Exposure Setting------------

    //------------Desired Gray Value Setting------------
    status = GXSetInt(hDevice, GX_INT_GRAY_VALUE, cameraSetting.desiredGrayValue);
    GX_VERIFY_EXIT(status);
    //Set automatic function lighting environment
    // status = GXSetEnum(hDevice, GX_ENUM_AA_LIGHT_ENVIRMENT, aaLightEnvirment);
    // GX_VERIFY_EXIT(status);
    //------------End Desired Gray Value Setting------------

    //------------Frame Rate Mode Setting------------
    //Enable acquisition frame rate adjustment mode
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, cameraSetting.acquisitionFrameRateMode);
    GX_VERIFY_EXIT(status);
    //Set acquisition frame rate
    status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, cameraSetting.acquisitionFrameRate);
    GX_VERIFY_EXIT(status);
    //------------End Frame Rate Mode Setting------------

    //------------Digital IO Control Setting------------
    //Line Select
    status = GXSetEnum(hDevice, GX_ENUM_LINE_SELECTOR, cameraSetting.lineSelector);
    GX_VERIFY_EXIT(status);
    //Set line mode
    status = GXSetEnum(hDevice, GX_ENUM_LINE_MODE, cameraSetting.lineMode);
    GX_VERIFY_EXIT(status);
    //Set Line level inversion
    status = GXSetBool(hDevice, GX_BOOL_LINE_INVERTER, cameraSetting.boolLineInverter);
    GX_VERIFY_EXIT(status);
    //Set Line source
    status = GXSetEnum(hDevice, GX_ENUM_LINE_SOURCE, cameraSetting.lineSource);
    GX_VERIFY_EXIT(status);

    if(cameraSetting.lineSource == 2 || 
        cameraSetting.lineSource == 3 || 
        cameraSetting.lineSource == 4 || 
        cameraSetting.lineSource == 9 || 
        cameraSetting.lineSource == 10 || 
        cameraSetting.lineSource == 11 || 
        cameraSetting.lineSource == 12)
    {
        status = GXSetEnum(hDevice,GX_ENUM_USER_OUTPUT_SELECTOR, cameraSetting.userOutputSelector);
        status = GXSetBool(hDevice, GX_BOOL_USER_OUTPUT_VALUE, cameraSetting.boolUserOutputValue);
        GX_VERIFY_EXIT(status);
    }
    //------------End Digital IO Control Setting------------

    //------------Analog Control Setting------------
    //Set gain channel type
    status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, cameraSetting.gainSelector);
    GX_VERIFY_EXIT(status);
    //Set gain type
    status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, cameraSetting.gainAuto);
    GX_VERIFY_EXIT(status);

    if(cameraSetting.gainAuto == GX_GAIN_AUTO_OFF)
    {
        status = GXSetFloat(hDevice, GX_FLOAT_GAIN, cameraSetting.gainValue);
        GX_VERIFY_EXIT(status);
    }
    else
    {
        status = GXSetFloat(hDevice, GX_FLOAT_AUTO_GAIN_MIN, cameraSetting.minAutoGainValue);
        status = GXSetFloat(hDevice, GX_FLOAT_AUTO_GAIN_MAX, cameraSetting.maxAutoGainValue);
        GX_VERIFY_EXIT(status);
    }
    //------------End Analog Control Setting------------

    //------------Balance Ratio Setting------------
    if (bColorFilter)
    {
        //Set continuous automatic white balance
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, cameraSetting.balanceWhiteAuto);
        GX_VERIFY_EXIT(status);
        //Select white balance channel
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, cameraSetting.balanceRatioSelector);
        GX_VERIFY_EXIT(status);
    }

    //Set automatic white balance lighting environment
    status = GXSetEnum(hDevice, GX_ENUM_AWB_LAMP_HOUSE, cameraSetting.awbLampHouse);
    GX_VERIFY_EXIT(status);
    //Set ambient light preset
    // status = GXSetEnum(hDevice, GX_ENUM_LIGHT_SOURCE_PRESET, cameraSetting.lightSourcePreset);
    // GX_VERIFY_EXIT(status);

    if(cameraSetting.balanceWhiteAuto == GX_BALANCE_WHITE_AUTO_OFF)
    {
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, cameraSetting.balanceRatio);
        GX_VERIFY_EXIT(status);
    }
    //------------End Balance Ratio Setting------------

    //------------Mining Parameter Setting------------
    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = cameraSetting.acquisitionBufferNum;
    status = GXSetAcqusitionBufferNumber(hDevice, nBufferNum);
    GX_VERIFY_EXIT(status);

    bool bStreamTransferSize = false;
    status = GXIsImplemented(hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(status);

    if (bStreamTransferSize)
    {
        //Set size of data transfer block
        status = GXSetInt(hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, cameraSetting.acquisitionTransferSize);
        GX_VERIFY_EXIT(status);
    }

    bool bStreamTransferNumberUrb = false;
    status = GXIsImplemented(hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(status);

    if (bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        status = GXSetInt(hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, cameraSetting.acquisitionTransferNumberURB);
        GX_VERIFY_EXIT(status);
    }
    //------------End Mining Parameter Setting------------
}

void galaxy_camera::CloseDevice()
{
    //Release resources when anything is done.
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
}

void galaxy_camera::GetDeviceInfo()
{
    //Get Device Info
    printf("***********************************************\n");
    //Get libary version
    printf("<Libary Version : %s>\n", GXGetLibVersion());

    size_t nSize = 0;
    //Get string length of Vendor name
    status = GXGetStringLength(hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    GX_VERIFY_EXIT(status);

    //Alloc memory for Vendor name
    char *pszVendorName = new char[nSize];

    //Get Vendor name
    status = GXGetString(hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (status != GX_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = NULL;
        GX_VERIFY_EXIT(status);
    }

    printf("<Vendor Name : %s>\n", pszVendorName);
    //Release memory for Vendor name
    delete[] pszVendorName;
    pszVendorName = NULL;

    //Get string length of Model name
    status = GXGetStringLength(hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY_EXIT(status);
    //Alloc memory for Model name
    char *pszModelName = new char[nSize];
    //Get Model name
    status = GXGetString(hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (status != GX_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = NULL;
        GX_VERIFY_EXIT(status);
    }

    printf("<Model Name : %s>\n", pszModelName);
    //Release memory for Model name
    delete[] pszModelName;
    pszModelName = NULL;

    //Get string length of Serial number
    status = GXGetStringLength(hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_VERIFY_EXIT(status);
    //Alloc memory for Serial number
    char *pszSerialNumber = new char[nSize];
    //Get Serial Number
    status = GXGetString(hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (status != GX_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;
        GX_VERIFY_EXIT(status);
    }

    printf("<Serial Number : %s>\n", pszSerialNumber);
    //Release memory for Serial number
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    //Get string length of Device version
    status = GXGetStringLength(hDevice, GX_STRING_DEVICE_VERSION, &nSize);
    GX_VERIFY_EXIT(status);
    char *pszDeviceVersion = new char[nSize];
    //Get Device Version
    status = GXGetString(hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (status != GX_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        GX_VERIFY_EXIT(status);
    }

    printf("<Device Version : %s>\n", pszDeviceVersion);
    //Release memory for Device version
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
    printf("***********************************************\n");

    //Get the type of Bayer conversion. whether is a color camera.
    bColorFilter = false;
    status = GXIsImplemented(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &bColorFilter);
    GX_VERIFY_EXIT(status);
}

void galaxy_camera::GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS status = GX_STATUS_SUCCESS;

    // Get length of error description
    status = GXGetLastError(&emErrorStatus, NULL, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return;
    }

    // Get error description
    status = GXGetLastError(&emErrorStatus, error_info, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete[] error_info;
        error_info = NULL;
    }
}
