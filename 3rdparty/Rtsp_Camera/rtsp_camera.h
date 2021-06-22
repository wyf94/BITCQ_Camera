#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class rtsp_camera
{
private: 
    cv::Mat ColorConvertor(cv::Mat);

    std::string address;
    int width; 
    int height; 
    int fps; 
    std::string format;

    cv::VideoCapture cap;
    cv::Mat image;

public:   
    rtsp_camera(std::string addr, int w, int h, int f, std::string fm) : address(addr), width(w), height(h), fps(f), format(fm) 
    {
        cap.open(address);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    }

    void RtspCamera(); 
};