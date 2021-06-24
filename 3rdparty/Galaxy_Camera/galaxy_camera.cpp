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
    //Set the acquisition mode to continuous acquisition
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY_EXIT(status);

    //Set the trigger switch to OFF
    status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(status);
    // status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
    // GX_VERIFY_EXIT(status);
    // status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
    // GX_VERIFY_EXIT(status);
    // status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
    // GX_VERIFY_EXIT(status);

    //exposure mode and time setting
    status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GX_VERIFY_EXIT(status);
    status = GXSetFloat(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, 20.0000);//us,20-1000000
    GX_VERIFY_EXIT(status);
    status = GXSetFloat(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, 25000.0000);//us,20-1000000

    //set AutoGain mode and max_min
    status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    GX_VERIFY_EXIT(status);
    status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    GX_VERIFY_EXIT(status);
    status = GXSetFloat(hDevice, GX_FLOAT_AUTO_GAIN_MIN, 0.0000);//0-24
    GX_VERIFY_EXIT(status);
    status = GXSetFloat(hDevice, GX_FLOAT_AUTO_GAIN_MAX, 20.0000);//0-24
    GX_VERIFY_EXIT(status);

    if (bColorFilter)
    {
        printf("This is a color camera! \n");
        //white balance
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
        GX_VERIFY_EXIT(status);
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        GX_VERIFY_EXIT(status);
    }

    //acquisition frame rate 
    status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, fps);
    GX_VERIFY_EXIT(status);

    //set GPIO output signal
    status = GXSetEnum(hDevice, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE3);
    GX_VERIFY_EXIT(status);
    status = GXSetEnum(hDevice, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_USEROUTPUT0);
    GX_VERIFY_EXIT(status);
    status = GXSetEnum(hDevice, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_OUTPUT);
    GX_VERIFY_EXIT(status);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    status = GXSetAcqusitionBufferNumber(hDevice, nBufferNum);
    GX_VERIFY_EXIT(status);

    bool bStreamTransferSize = false;
    status = GXIsImplemented(hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(status);

    if(bStreamTransferSize)
    {
        //Set size of data transfer block
        status = GXSetInt(hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(status);
    }

    bool bStreamTransferNumberUrb = false;
    status = GXIsImplemented(hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(status);

    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        status = GXSetInt(hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(status);
    }

    // //Set automatic function lighting environment
    // status = GXSetEnum(hDevice, GX_ENUM_AA_LIGHT_ENVIRMENT, GX_AA_LIGHT_ENVIRMENT_NATURELIGHT);
    // GX_VERIFY_EXIT(status);

    //获取期望灰度值调节范围
    GX_INT_RANGE grayValueRange;
    status = GXGetIntRange(hDevice, GX_INT_GRAY_VALUE, &grayValueRange);
    //设置最小期望灰度值
    status = GXSetInt(hDevice, GX_INT_GRAY_VALUE, grayValueRange.nMin);
    //设置最大期望灰度值
    status = GXSetInt(hDevice, GX_INT_GRAY_VALUE, grayValueRange.nMax);

    // //获取调节范围
    // GX_FLOAT_RANGE autoGainMinRange;
    // GX_FLOAT_RANGE autoGainMaxRange;
    // GX_FLOAT_RANGE autoExposureMinRange;
    // GX_FLOAT_RANGE autoExposureMaxRange;
    // status = GXGetFloatRange(hDevice, GX_FLOAT_AUTO_GAIN_MIN, &autoGainMinRange);
    // status = GXGetFloatRange(hDevice, GX_FLOAT_AUTO_GAIN_MAX, &autoGainMaxRange);
    // status = GXGetFloatRange(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, &autoExposureMinRange);
    // status = GXGetFloatRange(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, &autoExposureMaxRange);
    // //设置调节边界值
    // status = GXSetInt(hDevice, GX_FLOAT_AUTO_GAIN_MIN, autoGainMinRange.dMin);
    // status = GXSetInt(hDevice, GX_FLOAT_AUTO_GAIN_MAX, autoGainMinRange.dMax);
    // status = GXSetInt(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, autoShutterMinRange.dMin);
    // status = GXSetInt(hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, autoShutterMaxRange.dMax);
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
