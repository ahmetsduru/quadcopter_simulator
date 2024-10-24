#include "../include/quadcopter_control/data_logger.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node with the name "data_logger"
    ros::init(argc, argv, "data_logger_node");

    // Create an instance of the DataLogger class
    DataLogger data_logger;

    // Call the spin function to start logging data
    data_logger.spin();

    return 0;
}
