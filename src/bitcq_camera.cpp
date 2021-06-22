#include <bitcq_camera.h>

//Automatical start camera init and image publish.
void bitcq_camera::Open()
{
    if (cameraInfo.deviceType == "gmsl" || cameraInfo.deviceType == "usb")
    {
        gmsl->OpenDevice();
        gmsl->InitCamera();
        gmsl->StartCapture();

        Spin();

        gmsl->StopCapture();
        gmsl->FreeBuffers();
        gmsl->CloseDevice();
    }
    else if (cameraInfo.deviceType == "rtsp")
    {
        rtsp->GetImage();
    }
    else if (cameraInfo.deviceType == "dahua_sdk")
    {
        int status = galaxy->OpenDevice();
        ROS_INFO("Dahua Camera Return Status: %d\n", status);
    }
    else if (cameraInfo.deviceType == "hik_sdk")
    {
        ROS_INFO("DEVICE NOT FOUND: Program Do not access Hikvision Camera!\n");
    }
    ROS_INFO("NOT FOUND: Wrong device type!\n");
}

//Loop detection of image acquisition and image sending.
void bitcq_camera::Spin()
{
    ros::Rate loopRate(this->cameraInfo.fps);
    while (node.ok())
    {
        cv::Mat image = gmsl->GetImage();
        PublishImage(image);

        ros::spinOnce();
        loopRate.sleep();
    }
}

//Publish Image which grab from memory.
void bitcq_camera::PublishImage(cv::Mat image)
{
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub.publish(msg);
}