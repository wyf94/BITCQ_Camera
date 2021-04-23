#include <gmsl_camera.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

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

//Automatical start camera init and image publish.
int gmsl_camera::Open()
{
    if (cameraInfo.deviceType == "gmsl" || cameraInfo.deviceType == "usb")
    {
        if (!OpenDevice())
        {
            perror("Open Device Failed!\n");
            return -1;
        }
        if (!InitCamera())
        {
            perror("Init Camera Failed!\n");
            return -1;
        }
        if (!StartCapture())
        {
            perror("Start Capture Failed!\n");
            return -1;
        }

        Spin();

        StopCapture();
        FreeBuffers();
        CloseDevice();
        return 1;
    }
    else if (cameraInfo.deviceType == "rtsp")
    {
        if (!RtspCamera())
        {
            perror("RTSP Camera Load Failed!\n");
        }
        return 1;
    }
    else if(cameraInfo.deviceType == "hik_sdk")
    {
        
    }
}

//Open camera device by camera's type.
int gmsl_camera::RtspCamera()
{
    cv::VideoCapture cap;
    cv::Mat image;

    cap.open(cameraInfo.address);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, cameraInfo.height);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, cameraInfo.width);

    while (node.ok())
    {
        if (cap.isOpened())
        {
            cap >> image;
        }
        if (image.empty())
        {
            printf("%s get frame failed!\n", cameraInfo.deviceType.c_str());
            continue;
        }
        ProcessImage(image);
    }
    return 1;
}

//Open camera device by camera's type.
int gmsl_camera::OpenDevice()
{
    fd = open(cameraInfo.address.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd < 0)
    {
        perror("Can't open camera.\n");
        return -1;
    }
    return 1;
}

int gmsl_camera::CloseDevice()
{
    if (fd > 0)
    {
        if (close(fd) < 0)
        {
            perror("Can't close camera.\n");
            return -1;
        }
    }
    return 1;
}

int gmsl_camera::StartCapture()
{
    //Set streaming on.
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
        printf("%s start streaming failed.\n", cameraInfo.address.c_str());
        return -1;
    }
    return 1;
}

int gmsl_camera::StopCapture()
{
    //Set streaming on.
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        printf("%s stop streaming failed.\n", cameraInfo.address.c_str());
        return -1;
    }
    return 1;
}

//Init buffer cache and setting camera mapping.
int gmsl_camera::InitBuffers()
{
    struct v4l2_requestbuffers rb;
    struct v4l2_buffer buf;
    CLEAR(buf);
    CLEAR(rb);

    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    rb.count = 4; //Most usage less than 5.
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
    {
        printf("%s request frame buffers failed.\n", cameraInfo.address.c_str());
    }

    if (rb.count < 2)
    {
        perror("Request frame buffers while insufficient buffer memory.\n");
    }

    //Calloc new memory to buffers.
    buffers = (struct buffer *)calloc(rb.count, sizeof(*buffers));
    if (!buffers)
    {
        perror("Buffer calloc out of memory.\n");
        return -1;
    }

    //Get each buffer and mmap to userspace.
    for (bufferCount = 0; bufferCount < rb.count; ++bufferCount)
    {
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = bufferCount;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            printf("%s does not get buffer.\n", cameraInfo.address.c_str());
            return -1;
        }
        buffers[bufferCount].length = buf.length;
        buffers[bufferCount].start = mmap(NULL /* start anywhere */,
                                          buf.length,
                                          PROT_READ | PROT_WRITE /* required */,
                                          MAP_SHARED /* recommended */,
                                          fd, buf.m.offset);

        if (buffers[bufferCount].start == MAP_FAILED)
        {
            printf("%d buffer mmap failed.\n", bufferCount);
            return -1;
        }
    }

    return 1;
}

int gmsl_camera::FreeBuffers()
{
    unsigned int i;
    for (i = 0; i < bufferCount; ++i)
    {
        if (-1 == munmap(buffers[i].start, buffers[i].length))
        {
            printf("munmap buffer%d failed\n", i);
            return -1;
        }
    }
    free(buffers);
    return 0;
}

//Initialize logical camera.
int gmsl_camera::InitCamera()
{
    struct v4l2_capability cap;    //Find camera's capability.
    struct v4l2_cropcap cropcap;   //Setting camera's capture mode.
    struct v4l2_fmtdesc fmtdesc;   //Find all Camera's format：VIDIOC_ENUM_FMT
    struct v4l2_crop crop;         //Scaling camera.
    struct v4l2_format fmt;        //Setting camera's framerate and video parameters.
    struct v4l2_streamparm params; //Setting Capture parameters.

    CLEAR(cap);
    CLEAR(cropcap);
    CLEAR(fmtdesc);
    CLEAR(crop);
    CLEAR(fmt);
    CLEAR(params);

    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        if (EINVAL == errno)
        {
            printf("%s is no camera device.\n", cameraInfo.address.c_str());
            return -1;
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

    params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    params.parm.capture.capturemode = V4L2_MODE_HIGHQUALITY;
    params.parm.capture.timeperframe.numerator = 1;
    params.parm.capture.timeperframe.denominator = cameraInfo.fps;
    if (ioctl(fd, VIDIOC_S_PARM, &params) < 0)
    {
        printf("%s can't setting capture parameters.\n", cameraInfo.address.c_str());
    }

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format:\n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
    {
        printf("\t%d.%s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = cameraInfo.width;
    fmt.fmt.pix.height = cameraInfo.height;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY; //TO-----------------DO
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        printf("%s does not support those specified parameter.\n", cameraInfo.address.c_str());
        return -1;
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0)
    {
        perror("Can't get frame information");
    }

    if (InitBuffers() < 0)
    {
        perror("Buffers init error");
        return -1;
    }

    return 1;
}

//Get one frame from camera.
void gmsl_camera::GetImage()
{
    cv::Mat image;

    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    {
        printf("%s get frame failed.\n", cameraInfo.address.c_str());
        return;
    }

    image = cv::Mat(cv::Size(cameraInfo.width, cameraInfo.height), CV_8UC3, buffers[buf.index].start);

    ProcessImage(image);
}

//Process Image which grab from memory.
void gmsl_camera::ProcessImage(cv::Mat image)
{
    cv::cvtColor(image, image, cv::COLOR_YUV420p2BGR);

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub.publish(msg);
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