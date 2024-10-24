#include <ros/ros.h>
#include "../include/quadcopter_control/waypoint_server.h"  // Include the header file for WaypointServer

int main(int argc, char** argv) {
    // Initialize the ROS node with the name "waypoint_server"
    ros::init(argc, argv, "waypoint_server");

    // Create a ROS NodeHandle object to interact with ROS
    ros::NodeHandle nh;

    // Create an instance of the WaypointServer class
    WaypointServer traj_server(nh);

    // Enter a loop, pumping callbacks (waiting for service requests)
    ros::spin();

    // Return 0 when the program exits successfully
    return 0;
}
