#include <gmsl_camera.h>

typedef unsigned long long hash_t;

constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

constexpr inline static hash_t hash_(char const *str)
{
    hash_t ret{basis};
    while (*str)
    {
        ret ^= *str;
        ret *= prime;
        str++;
    }
    return ret;
}

constexpr hash_t operator"" _hash(char const *p, size_t)
{
    return hash_(p);
}

//Initialize ROS node and set camera's parameters.
void gmsl_camera::RosInit()
{
    node = ros::NodeHandle("publisher");

    image_transport::ImageTransport it(node);
    pub = it.advertise("image_source", 1);

    node.param("device_type", cameraInfo.deviceType, std::string("other"));
    node.param("device_address", cameraInfo.address, std::string("/dev/video0"));
    node.param("image_width", cameraInfo.width, 640);
    node.param("image_height", cameraInfo.height, 480);
    node.param("fps", cameraInfo.fps, 24);
    node.param("pixel_format", format, std::string("yuyv"));
    cameraInfo.format = String2Enum(format);
}

//Initialize logical camera.
void gmsl_camera::CameraInit()
{
    //Open camera device by camera's type.
    int fd = open(cameraInfo.address.c_str(), O_RDWR | O_NONBLOCK, 0);

    //Find camera's capability.
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        if (EINVAL == errno)
        {
            printf("%s is no V4L2 device.\n", cameraInfo.address.c_str());
            return;
        }
        else 
        {
            printf("Successfully Initilize Camera Device!\n");
        }
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        printf("%s does not support video capture.\n", cameraInfo.address.c_str());
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        printf("%s does not support streaming i/o.\n", cameraInfo.address.c_str());
    }

    //Find multimedia device.
    struct v4l2_input inp;
    inp.index = 0;
    int ret = ioctl(fd, VIDIOC_S_INPUT, &inp);

    struct v4l2_streamparm params;
    memset(&params, 0, sizeof(params));
    params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    params.parm.capture.capturemode = V4L2_MODE_HIGHQUALITY;
    params.parm.capture.timeperframe.numerator = 1;
    params.parm.capture.timeperframe.denominator = 25;

    ret = ioctl(fd, VIDIOC_S_PARM, &params);

    struct v4l2_format fmt;
    ret = ioctl(fd, VIDIOC_G_FMT, &fmt);
}

//Get one frame from camera.
void gmsl_camera::GetImage()
{
    struct v4l2_buffer buffer;
    memset(&(buffer), 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
}

//Loop detection of image acquisition and image sending.
void gmsl_camera::Spin()
{
    ros::Rate loopRate(this->cameraInfo.fps);
    while (node.ok())
    {
        GetImage();
        ros::spinOnce();
        loopRate.sleep();
    }
}

//Case camera format and return enum type.
gmsl_camera::pixel_format gmsl_camera::String2Enum(std::string format)
{
    switch (hash_(format.c_str()))
    {
    case "yuyv"_hash:
        return PIXEL_FORMAT_YUYV;
        break;
    case "uyvy"_hash:
        return PIXEL_FORMAT_UYVY;
        break;
    case "mjpeg"_hash:
        return PIXEL_FORMAT_MJPEG;
        break;
    case "yuvmono10"_hash:
        return PIXEL_FORMAT_YUVMONO10;
        break;
    case "rgb24"_hash:
        return PIXEL_FORMAT_RGB24;
        break;
    case "grey"_hash:
        return PIXEL_FORMAT_GREY;
        break;
    default:
        return PIXEL_FORMAT_UNKNOWN;
    }
}