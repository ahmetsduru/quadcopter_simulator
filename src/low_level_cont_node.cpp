#include <ros/ros.h>
#include "../include/quadcopter_control/low_level_cont.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "low_level_controller_node");

    LowLevelController low_controller;
    low_controller.spin();

    return 0;
}
