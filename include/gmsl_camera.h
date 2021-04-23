#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <error.h>
#include <errno.h>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

    struct buffer
    {
        void *start;
        size_t length;
    };

    int Open();

    ros::Time time;
    CameraInfo cameraInfo;
    std::string format;
    std::string deviceType;
    sensor_msgs::ImagePtr msg;
    int fd;

    gmsl_camera() : node("~")
    {
        time = ros::Time::now();
        std::cout << time << std::endl;

        //Initialize ROS node and set camera's parameters.
        image_transport::ImageTransport it(node);
        pub = it.advertise("image_source", 1);

        node.param("device_type", cameraInfo.deviceType, std::string("other"));
        node.param("device_address", cameraInfo.address, std::string("/dev/video0"));
        node.param("image_width", cameraInfo.width, 640);
        node.param("image_height", cameraInfo.height, 480);
        node.param("fps", cameraInfo.fps, 24);
        node.param("pixel_format", format, std::string("yuyv"));
        cameraInfo.format = String2Enum(format);

        fd = -1;
        buffers = NULL;
        bufferCount = 0;
    }

private:
    int InitCamera();
    int OpenDevice();
    int CloseDevice();
    int StartCapture();
    int StopCapture();
    int InitBuffers();
    int FreeBuffers();
    int RtspCamera();

    void Spin();
    void GetImage();
    void ProcessImage(cv::Mat);
    pixel_format String2Enum(std::string);

    ros::NodeHandle node;
    image_transport::Publisher pub;
    struct buffer *buffers;
    unsigned int bufferCount;
};