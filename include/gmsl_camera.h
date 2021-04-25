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
#include <stdint.h>
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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class gmsl_camera
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

    struct buffer
    {
        void *start;
        size_t length;
    };

    int Open();

    ros::Time time;
    CameraInfo cameraInfo;
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

        node.param("device_type", cameraInfo.deviceType, std::string("gmsl"));
        node.param("device_address", cameraInfo.address, std::string("/dev/video0"));
        node.param("image_width", cameraInfo.width, 1920);
        node.param("image_height", cameraInfo.height, 1080);
        node.param("fps", cameraInfo.fps, 30);
        node.param("pixel_format", cameraInfo.format, std::string("uyvy"));

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
    void PublishImage(cv::Mat);
    uint32_t FormatStringConvertor(std::string);
    cv::Mat ColorConvertor(const void *,uint32_t);
    cv::Mat ColorConvertor(cv::Mat);

    ros::NodeHandle node;
    image_transport::Publisher pub;
    struct buffer *buffers;
    unsigned int bufferCount;

    static int xioctl(int fd, int request, void *arg)
    {
        int r;
        do
            r = ioctl(fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;
    }

    static void errno_exit(const char *s)
    {
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
};