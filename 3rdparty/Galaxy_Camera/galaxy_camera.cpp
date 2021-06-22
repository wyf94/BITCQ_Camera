#include "galaxy_camera.h"

int galaxy_camera::OpenDevice()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ui32DeviceNum = 0;

    status = GXInitLib();

    if (status != GX_STATUS_SUCCESS)
    {
        printf("Load Galaxy Camera Failed!\n");
    }
    printf("Load Galaxy Camera Success!\n");

    //Get device enumerated number
    status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if (status != GX_STATUS_SUCCESS)
    {
        GXCloseLib();
        return status;
    }

    //If no device found, app exit
    if (ui32DeviceNum <= 0)
    {
        printf("No device found\n");
        GXCloseLib();
        return status;
    }

    //Release resources when anything is done.
    status = GXCloseLib();
    return status;
}