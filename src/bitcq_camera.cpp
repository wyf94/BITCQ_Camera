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
        Spin();
    }
    else if (cameraInfo.deviceType == "daheng_sdk")
    {
        galaxy->OpenDevice();
        Spin();
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
    cv::Mat image = cv::Mat::zeros(cv::Size(cameraInfo.width, cameraInfo.height), CV_8UC3);
    ros::Rate loopRate(this->cameraInfo.fps);
    while (node.ok())
    {
        if (cameraInfo.deviceType == "gmsl" || cameraInfo.deviceType == "usb")
        {
            image = gmsl->GetImage();
        }
        else if (cameraInfo.deviceType == "rtsp")
        {
            image = rtsp->GetImage();
        }
        else if (cameraInfo.deviceType == "daheng_sdk")
        {
            image = galaxy->GetImage();
        }
        else if (cameraInfo.deviceType == "hik_sdk")
        {
            ROS_INFO("Hik_Camera was not supported!\n");
        }

	    cv::resize(image,image,cv::Size(640,480));

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
