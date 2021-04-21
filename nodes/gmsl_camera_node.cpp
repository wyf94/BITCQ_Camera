#include <gmsl_camera.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gmsl_camera");

    gmsl_camera Camera;

    Camera.RosInit();

    Camera.Spin();
}