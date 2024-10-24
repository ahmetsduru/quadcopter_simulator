#include "../include/quadcopter_control/mid_level_cont.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mid_level_cont");

    MidLevelController midcontroller;
    midcontroller.spin();

    return 0;
}
