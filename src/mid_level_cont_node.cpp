#include "../include/quadcopter_control/mid_level_cont.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mid_level_cont_node");

    MidLevelController midcontroller;
    midcontroller.spin();

    return 0;
}
