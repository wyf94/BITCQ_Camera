#include <iostream>

#include "GxIAPI.h"
#include "DxImageProc.h"

class galaxy_camera
{
private:
    std::string address;
    int width; 
    int height; 
    int fps; 
    std::string format;

public:
    galaxy_camera(std::string addr, int w, int h, int f, std::string fm) : address(addr), width(w), height(h), fps(f), format(fm) {}
    int OpenDevice();
};