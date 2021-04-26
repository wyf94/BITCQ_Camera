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

const unsigned char uchar_clipping_table[] = {
    0, 0, 0, 0, 0, 0, 0, 0, // -128 - -121
    0, 0, 0, 0, 0, 0, 0, 0, // -120 - -113
    0, 0, 0, 0, 0, 0, 0, 0, // -112 - -105
    0, 0, 0, 0, 0, 0, 0, 0, // -104 -  -97
    0, 0, 0, 0, 0, 0, 0, 0, //  -96 -  -89
    0, 0, 0, 0, 0, 0, 0, 0, //  -88 -  -81
    0, 0, 0, 0, 0, 0, 0, 0, //  -80 -  -73
    0, 0, 0, 0, 0, 0, 0, 0, //  -72 -  -65
    0, 0, 0, 0, 0, 0, 0, 0, //  -64 -  -57
    0, 0, 0, 0, 0, 0, 0, 0, //  -56 -  -49
    0, 0, 0, 0, 0, 0, 0, 0, //  -48 -  -41
    0, 0, 0, 0, 0, 0, 0, 0, //  -40 -  -33
    0, 0, 0, 0, 0, 0, 0, 0, //  -32 -  -25
    0, 0, 0, 0, 0, 0, 0, 0, //  -24 -  -17
    0, 0, 0, 0, 0, 0, 0, 0, //  -16 -   -9
    0, 0, 0, 0, 0, 0, 0, 0, //   -8 -   -1
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88,
    89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
    114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136,
    137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
    160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
    183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205,
    206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228,
    229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251,
    252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 256-263
    255, 255, 255, 255, 255, 255, 255, 255,                     // 264-271
    255, 255, 255, 255, 255, 255, 255, 255,                     // 272-279
    255, 255, 255, 255, 255, 255, 255, 255,                     // 280-287
    255, 255, 255, 255, 255, 255, 255, 255,                     // 288-295
    255, 255, 255, 255, 255, 255, 255, 255,                     // 296-303
    255, 255, 255, 255, 255, 255, 255, 255,                     // 304-311
    255, 255, 255, 255, 255, 255, 255, 255,                     // 312-319
    255, 255, 255, 255, 255, 255, 255, 255,                     // 320-327
    255, 255, 255, 255, 255, 255, 255, 255,                     // 328-335
    255, 255, 255, 255, 255, 255, 255, 255,                     // 336-343
    255, 255, 255, 255, 255, 255, 255, 255,                     // 344-351
    255, 255, 255, 255, 255, 255, 255, 255,                     // 352-359
    255, 255, 255, 255, 255, 255, 255, 255,                     // 360-367
    255, 255, 255, 255, 255, 255, 255, 255,                     // 368-375
    255, 255, 255, 255, 255, 255, 255, 255,                     // 376-383
};

const int clipping_table_offset = 128;
static unsigned char CLIPVALUE(int val)
{
    return uchar_clipping_table[val + clipping_table_offset];
}

static void YUV2RGB(const unsigned char y, const unsigned char u, const unsigned char v, unsigned char *r,
                    unsigned char *g, unsigned char *b)
{
    const int y2 = (int)y;
    const int u2 = (int)u - 128;
    const int v2 = (int)v - 128;

    int r2 = y2 + ((v2 * 37221) >> 15);
    int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
    int b2 = y2 + ((u2 * 66883) >> 15);

    *r = CLIPVALUE(r2);
    *g = CLIPVALUE(g2);
    *b = CLIPVALUE(b2);
}

static void yuyv2rgb(char *YUV, char *RGB, int NumPixels)
{
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;

    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
        y0 = (unsigned char)YUV[i + 0];
        u = (unsigned char)YUV[i + 1];
        y1 = (unsigned char)YUV[i + 2];
        v = (unsigned char)YUV[i + 3];
        YUV2RGB(y0, u, v, &r, &g, &b);
        RGB[j + 0] = r;
        RGB[j + 1] = g;
        RGB[j + 2] = b;
        YUV2RGB(y1, u, v, &r, &g, &b);
        RGB[j + 3] = r;
        RGB[j + 4] = g;
        RGB[j + 5] = b;
    }
}

static void uyvy2rgb(char *YUV, char *RGB, int NumPixels)
{
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;
    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
        u = (unsigned char)YUV[i + 0];
        y0 = (unsigned char)YUV[i + 1];
        v = (unsigned char)YUV[i + 2];
        y1 = (unsigned char)YUV[i + 3];
        YUV2RGB(y0, u, v, &r, &g, &b);
        RGB[j + 0] = r;
        RGB[j + 1] = g;
        RGB[j + 2] = b;
        YUV2RGB(y1, u, v, &r, &g, &b);
        RGB[j + 3] = r;
        RGB[j + 4] = g;
        RGB[j + 5] = b;
    }
}

static void mono102mono8(char *RAW, char *MONO, int NumPixels)
{
    int i, j;
    for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1)
    {
        MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
    }
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
    else if (cameraInfo.deviceType == "hik_sdk")
    {
    }
    return 1;
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

    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
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
    if (xioctl(fd, VIDIOC_S_PARM, &params) == -1)
    {
        printf("%s can't setting capture parameters.\n", cameraInfo.address.c_str());
    }

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format:\n");
    while (xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
    {
        printf("\t%d.%s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = cameraInfo.width;
    fmt.fmt.pix.height = cameraInfo.height;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.pixelformat = FormatStringConvertor(cameraInfo.format);
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
    {
        printf("%s does not support those specified parameter.\n", cameraInfo.address.c_str());
        return -1;
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_FMT, &fmt) == -1)
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

//Open camera device by camera's type.
int gmsl_camera::OpenDevice()
{
    fd = open(cameraInfo.address.c_str(), O_RDWR /*| O_NONBLOCK*/, 0);
    if (fd < 0)
    {
        perror("Can't open camera.\n");
        return -1;
    }
    return 1;
}

//Close camera device.
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

//Start camera stream capture.
int gmsl_camera::StartCapture()
{
    //Set streaming on.
    enum v4l2_buf_type type;

    for (unsigned int i = 0; i < bufferCount; i++)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
        {
            errno_exit("VIDIOC_QBUF");
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) == -1)
    {
        printf("%s start streaming failed.\n", cameraInfo.address.c_str());
        return -1;
    }
    return 1;
}

//Stop camera stream capture.
int gmsl_camera::StopCapture()
{
    //Set streaming on.
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
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
    CLEAR(rb);

    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    rb.count = 4; //Most usage less than 5.
    if (xioctl(fd, VIDIOC_REQBUFS, &rb) == -1)
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
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = bufferCount;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
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

//Free buffer cache.
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
        ColorConvertor(image);
        PublishImage(image);
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

    if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1)
    {
        errno_exit("VIDIOC_DQBUF");
    }

    //image = cv::Mat(cv::Size(cameraInfo.width, cameraInfo.height), CV_8UC3, buffers[buf.index].start);

    image = ColorConvertor(buffers[buf.index].start, buf.bytesused);

    PublishImage(image);

    if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
    {
        errno_exit("VIDIOC_QBUF");
    }
}

//Publish Image which grab from memory.
void gmsl_camera::PublishImage(cv::Mat image)
{
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
uint32_t gmsl_camera::FormatStringConvertor(std::string format)
{
    switch (hash_(format.c_str()))
    {
    case "yuyv"_hash:
        return V4L2_PIX_FMT_YUYV;
        break;
    case "uyvy"_hash:
        return V4L2_PIX_FMT_UYVY;
        break;
    case "mjpeg"_hash:
        return V4L2_PIX_FMT_MJPEG; //Need Decoding
        break;
    case "yuvmono10"_hash:
        return V4L2_PIX_FMT_Y16;
        break;
    case "rgb24"_hash:
        return V4L2_PIX_FMT_RGB24;
        break;
    case "grey"_hash:
        return V4L2_PIX_FMT_GREY;
        break;
    default:
        return -1;
    }
}

cv::Mat gmsl_camera::ColorConvertor(const void *src, uint32_t len)
{
    switch (hash_(cameraInfo.format.c_str()))
    {
    case "yuyv"_hash:
        yuyv2rgb((char *)src, imageT->image, imageT->height * imageT->width);
        break;
    case "uyvy"_hash:
        //uyvy2rgb((char *)src, imageT->image, imageT->height * imageT->width);
        yuyv2rgb((char *)src, imageT->image, imageT->height * imageT->width);
        break;
    case "mjpeg"_hash:
        //cv::cvtColor(image, image, cv::COLOR_); //Need Decoding
        break;
    case "yuvmono10"_hash:
        mono102mono8((char *)src, imageT->image, imageT->height * imageT->width);
        break;
    case "rgb24"_hash:
        memcpy(imageT->image, (char *)src, imageT->height * imageT->width * 3);
        break;
    case "grey"_hash:
        memcpy(imageT->image, (char *)src, imageT->width * imageT->height);
        break;
    default:
        break;
    }

    cv::Mat image = cv::Mat(cv::Size(imageT->width, imageT->height), CV_8UC3, imageT->image);

    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    return image;
}

cv::Mat gmsl_camera::ColorConvertor(cv::Mat image)
{
    switch (hash_(cameraInfo.format.c_str()))
    {
    case "yuyv"_hash:
        cv::cvtColor(image, image, cv::COLOR_YUV2BGR_YUY2);
        break;
    case "uyvy"_hash:
        cv::cvtColor(image, image, cv::COLOR_YUV2BGR_UYVY);
        break;
    case "mjpeg"_hash:
        //cv::cvtColor(image, image, cv::COLOR_); //Need Decoding
        break;
    case "yuvmono10"_hash:
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR); //16BIT Gray
        break;
    case "rgb24"_hash:
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        break;
    case "grey"_hash:
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        break;
    default:
        break;
    }

    return image;
}
