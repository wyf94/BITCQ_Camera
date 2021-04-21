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

void gmsl_camera::RosInit()
{
    /**
     * Initialize ROS node
     */
    node = ros::NodeHandle("publisher");

    image_transport::ImageTransport it(node);
    pub = it.advertise("image_source", 1);

    std::string format;
    ros::param::get("/video_device", cameraInfo.name);
    ros::param::get("/image_width", cameraInfo.width);
    ros::param::get("/image_height", cameraInfo.height);
    ros::param::get("/pixel_format", format);
    cameraInfo.format = String2Enum(format);

    std::cout << cameraInfo.name << " " << cameraInfo.width << " " << cameraInfo.height << " " << format;
}

void gmsl_camera::Spin()
{
}

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