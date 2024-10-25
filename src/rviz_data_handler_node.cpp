#include "../include/quadcopter_control/rviz_data_handler.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_data");
    ros::NodeHandle nh;
    RVizDataHandler rviz_data_handler(nh);

    ros::Rate rate(500);  // Set the loop rate
    while (ros::ok())
    {
        ros::spinOnce();  // Process callbacks
        rate.sleep();     // Sleep to maintain loop rate
    }

    return 0;
}
