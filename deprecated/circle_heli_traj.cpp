#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <vector>
#include <cmath>

int main(int argc, char **argv) {
    ros::init(argc, argv, "helical_and_circle_traj");
    ros::NodeHandle nh;

    // Parameters for the Trajectory
    double helix_radius, circle_radius, helix_height, T_helix, T_circle, total_time, hz;

    // Retrieve parameters before any further initialization or computations
    if (!nh.getParam("circle_heli_traj/helix_radius", helix_radius) ||
        !nh.getParam("circle_heli_traj/circle_radius", circle_radius) ||
        !nh.getParam("circle_heli_traj/helix_height", helix_height) ||
        !nh.getParam("circle_heli_traj/T_helix", T_helix) ||
        !nh.getParam("circle_heli_traj/T_circle", T_circle) ||
        !nh.getParam("circle_heli_traj/total_time", total_time) ||
        !nh.getParam("circle_heli_traj/hz", hz)) {
        ROS_ERROR("Failed to retrieve one or more parameters. Exiting...");
        return -1;  // Exit if any parameter is missing
    }

    // Check for invalid parameters
    if (T_helix <= 0 || T_circle <= 0 || helix_radius <= 0 || circle_radius <= 0) {
        ROS_ERROR("Invalid parameter: Check T_helix, T_circle, helix_radius, and circle_radius");
        return -1;
    }

    // Publisher for position and psi
    ros::Publisher position_pub = nh.advertise<geometry_msgs::Vector3>("position", 10);
    ros::Publisher psi_pub = nh.advertise<std_msgs::Float64>("psi", 10);

    // Adjust total time to include 5 circles
    total_time = T_helix + 5 * T_circle;

    // Calculate time step based on frequency
    double dt = 1.0 / hz;  // Time step, where dt is the duration of each loop iteration
    int num_points = static_cast<int>(total_time / dt);  // Number of points in the trajectory

    // Initialize vectors to store trajectory points
    std::vector<double> x(num_points, 0.0);  // Initialize with zeros
    std::vector<double> y(num_points, 0.0);  // Initialize with zeros
    std::vector<double> z(num_points, 0.0);  // Initialize with zeros
    std::vector<double> psi(num_points, 0.0);  // psi remains constant at 0

    // Generate the helical and circular trajectory
    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
        if (t < T_helix) {
            // Helical ascent
            double theta = (2 * M_PI / T_helix) * t;  // Linear angular velocity for helix
            x[i] = helix_radius * cos(theta);         // X coordinate for helix
            y[i] = helix_radius * sin(theta);         // Y coordinate for helix
            z[i] = (helix_height / T_helix) * t;      // Z coordinate (ascending)
        } else {
            // Circular path after helical ascent
            double theta = (2 * M_PI / T_circle) * (t - T_helix);  // Linear angular velocity for circle
            theta = fmod(theta, 2 * M_PI);  // Keep theta within a single circle's range
            x[i] = circle_radius * cos(theta);                     // X coordinate for circle
            y[i] = circle_radius * sin(theta);                     // Y coordinate for circle
            z[i] = helix_height;                                   // Maintain constant height during the circle
        }

        // Check for NaN or invalid values
        if (std::isnan(x[i]) || std::isnan(y[i]) || std::isnan(z[i])) {
            ROS_ERROR("NaN value detected in trajectory: x=%f, y=%f, z=%f", x[i], y[i], z[i]);
            return -1;
        }
    }

    // Create a ROS rate object for the loop frequency
    ros::Rate rate(hz);  // Loop frequency in Hz

    // Main loop to publish the trajectory
    int index = 0;
    while (ros::ok() && index < num_points) {
        // Create and populate the Vector3 message for position
        geometry_msgs::Vector3 position;
        position.x = x[index];
        position.y = y[index];
        position.z = z[index];

        // Create and populate the Float64 message for psi
        std_msgs::Float64 psi_msg;
        psi_msg.data = psi[index];

        // Check for NaN values before publishing
        if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z)) {
            ROS_ERROR("Skipping NaN value in position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
            continue;
        }

        // Publish the messages
        position_pub.publish(position);
        psi_pub.publish(psi_msg);
        
        index++;
        rate.sleep();  // Ensure the loop runs at the correct rate defined by 'hz'
    }

    ros::spin();
    return 0;
}
