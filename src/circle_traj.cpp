#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <cmath>

// Function to load parameters
bool loadParams(ros::NodeHandle& nh, double& radius, double& z_height, double& duration, double& update_frequency) {
    bool success = true;

    success &= nh.getParam("circle_traj/circle_radius", radius);
    success &= nh.getParam("circle_traj/z_height", z_height);
    success &= nh.getParam("circle_traj/duration", duration);
    success &= nh.getParam("circle_traj/update_frequency", update_frequency);

    if (!success) {
        ROS_ERROR("Failed to get all parameters from config file.");
    }

    return success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_trajectory_node");
    ros::NodeHandle nh;

    // Load parameters using nh.getParam
    double radius, z_height, duration, update_frequency;
    if (!loadParams(nh, radius, z_height, duration, update_frequency)) {
        ROS_ERROR("Could not load parameters from parameter server.");
        return -1;
    }

    // Define publishers
    ros::Publisher position_pub = nh.advertise<geometry_msgs::Vector3>("position", 10);
    ros::Publisher psi_pub = nh.advertise<std_msgs::Float64>("psi", 10);

    ros::Rate rate(update_frequency);

    double start_time = ros::Time::now().toSec();

    while (ros::ok()) {
        // Calculate the elapsed time
        double current_time = ros::Time::now().toSec() - start_time;
        double angle = 2 * M_PI * (current_time / duration);  // Calculate the angle for the circle

        // Compute position using sin and cos
        geometry_msgs::Vector3 position_msg;
        position_msg.x = radius * cos(angle);
        position_msg.y = radius * sin(angle);
        position_msg.z = z_height;  // Fixed height

        // Psi (yaw) angle is always zero
        std_msgs::Float64 psi_msg;
        psi_msg.data = 0.0;

        // Publish position and psi messages
        position_pub.publish(position_msg);
        psi_pub.publish(psi_msg);

        rate.sleep();
    }

    return 0;
}
