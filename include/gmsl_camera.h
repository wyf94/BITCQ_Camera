#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>

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
        std::string deviceType;
        std::string address;
        int width;
        int height;
        int fps;
        pixel_format format;
    } CameraInfo;

    gmsl_camera()
    {
        time = ros::Time::now();
        std::cout << time << std::endl;
    }

    void RosInit();
    void Spin();
    void CameraInit();
    void GetImage();

    ros::Time time;
    CameraInfo cameraInfo;
    std::string format;
    std::string deviceType;

private:
    pixel_format String2Enum(std::string);

    ros::NodeHandle node;
    image_transport::Publisher pub;
};