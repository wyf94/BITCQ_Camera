#include "rtsp_camera.h"

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

//Open camera device by camera's type.
cv::Mat rtsp_camera::GetImage()
{
    if (cap.isOpened())
    {
        cap >> image;
    }
    if (image.empty())
    {
        perror("FAILED: Get frame failed\n");
        continue;
    }
    ColorConvertor(image);

    return image;
}

cv::Mat rtsp_camera::ColorConvertor(cv::Mat image)
{
    switch (hash_(format.c_str()))
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