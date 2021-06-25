#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <stdio.h>
#include <string.h>

#include "galaxy_camera.h"
#include "gmsl_camera.h"
#include "rtsp_camera.h"
#include <JetsonGPIO.h>

class bitcq_camera
{
public:
    typedef struct
    {
        std::string deviceType;
        std::string address;
        int width;
        int height;
        int fps;
        std::string format;
    } CameraInfo;

    void Open();

    ros::Time time;
    CameraInfo cameraInfo;
    std::string deviceType;
    sensor_msgs::ImagePtr msg;

    gmsl_camera *gmsl;
    galaxy_camera *galaxy;
    rtsp_camera *rtsp;

    bitcq_camera() : node("~")
    {
        GPIO::setmode(GPIO::BOARD);
        time = ros::Time::now();
        std::cout << time << std::endl;

        //Initialize ROS node and set camera's parameters.
        image_transport::ImageTransport it(node);
        pub = it.advertise("image_source", 1);

        node.param("device_type", cameraInfo.deviceType, std::string("daheng_sdk"));
        node.param("device_address", cameraInfo.address, std::string("/dev/video0"));
        node.param("image_width", cameraInfo.width, 1920);
        node.param("image_height", cameraInfo.height, 1080);
        node.param("fps", cameraInfo.fps, 30);
        node.param("pixel_format", cameraInfo.format, std::string("uyvy"));

        gmsl = new gmsl_camera(cameraInfo.address, cameraInfo.width, cameraInfo.height, cameraInfo.fps, cameraInfo.format);
        rtsp = new rtsp_camera(cameraInfo.address, cameraInfo.width, cameraInfo.height, cameraInfo.fps, cameraInfo.format);
        galaxy = new galaxy_camera(cameraInfo.address, cameraInfo.width, cameraInfo.height, cameraInfo.fps, cameraInfo.format);
    }

private:
    void Spin();

    void PublishImage(cv::Mat);

    ros::NodeHandle node;
    image_transport::Publisher pub;
};