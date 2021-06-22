#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <pthread.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <error.h>
#include <errno.h>
#include <algorithm>
#include <sstream>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class gmsl_camera
{
private:
    struct buffer
    {
        void *start;
        size_t length;
    };

    typedef struct
    {
        int width;
        int height;
        int bytesPerPixel;
        int imageSize;
        char *image;
    } ImageT;

    struct buffer *buffers;
    unsigned int bufferCount;
    ImageT *imageT;
    int fd;
    std::string address;
    int width;
    int height;
    int fps;
    std::string format;

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

public:
    gmsl_camera();
    gmsl_camera(std::string addr, int w, int h, int f, std::string fm) : address(addr), width(w), height(h), fps(f), format(fm)
    {
        fd = -1;
        buffers = NULL;
        bufferCount = 0;

        imageT = (ImageT *)calloc(1, sizeof(ImageT));

        imageT->width = width;
        imageT->height = height;
        imageT->bytesPerPixel = 3;

        imageT->imageSize = imageT->width * imageT->height * imageT->bytesPerPixel;
        imageT->image = (char *)calloc(imageT->imageSize, sizeof(char));
        memset(imageT->image, 0, imageT->imageSize * sizeof(char));
    }

    void OpenDevice();
    void InitCamera();
    void StartCapture();
    void StopCapture();
    void InitBuffers();
    void FreeBuffers();
    void CloseDevice();

    cv::Mat GetImage();
    uint32_t FormatStringConvertor(std::string);
    cv::Mat ColorConvertor(const void *, uint32_t);
    cv::Mat ColorConvertor(cv::Mat);
};