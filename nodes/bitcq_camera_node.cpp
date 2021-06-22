#include <bitcq_camera.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bitcq_camera");

    bitcq_camera Camera;

    Camera.Open();
}