#include "../include/quadcopter_control/motor_dynamics.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_dynamics_node");
    RotorSpeedCalculator calculator;
    calculator.spin();
    return 0;
}
