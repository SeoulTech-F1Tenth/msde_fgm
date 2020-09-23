#include "ros/ros.h"
#include "msde_fgm/fgm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msde_fgm");
    ros::NodeHandle nh;

    fgm::FGM obj(nh);
    obj.start_driving();

    return 0;
}
