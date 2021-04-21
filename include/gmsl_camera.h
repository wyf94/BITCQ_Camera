#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include <algorithm>
#include <iostream>

class gmsl_camera
{
public:
    typedef enum
    {
        PIXEL_FORMAT_YUYV,
        PIXEL_FORMAT_UYVY,
        PIXEL_FORMAT_MJPEG,
        PIXEL_FORMAT_YUVMONO10,
        PIXEL_FORMAT_RGB24,
        PIXEL_FORMAT_GREY,
        PIXEL_FORMAT_UNKNOWN
    } pixel_format;

    typedef struct
    {
        std::string name;
        int width;
        int height;
        pixel_format format;
    } CameraInfo;

    gmsl_camera()
    {
        time = ros::Time::now();
        std::cout << time << std::endl;
    }

    void RosInit();
    void Spin();

    ros::Time time;
    CameraInfo cameraInfo;

private:
    pixel_format String2Enum(std::string);

    ros::NodeHandle node;
    image_transport::Publisher pub;
};