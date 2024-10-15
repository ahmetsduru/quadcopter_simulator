#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <cmath>

// PID controller state variables
double prev_error_torque1 = 0.0, integral_torque1 = 0.0;
double prev_error_torque2 = 0.0, integral_torque2 = 0.0;
double prev_error_torque3 = 0.0, integral_torque3 = 0.0;

// Variables to hold reference angles
double reference_phi = 0.0;
double reference_theta = 0.0;
double reference_psi = 0.0;

// Variables to hold current positions and angles
double current_x = 0.0;
double current_y = 0.0;
double current_z = 0.0;
double current_phi = 0.0;
double current_theta = 0.0;
double current_psi = 0.0;

// Function to compute PID controller output
double computePID(double setpoint, double measured_value, double& prev_error, double& integral, double kp, double ki, double kd, double dt) {
    double error = setpoint - measured_value;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    double output = kp * error + ki * integral + kd * derivative;
    return output;
}

// Callback function to update reference angles from /control/reference_angles topic
void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    reference_phi = msg->x;
    reference_theta = msg->y;
    reference_psi = msg->z;

    // ROS_INFO("Received reference angles low-level: phi=%.5f, theta=%.5f, psi=%.5f", reference_phi, reference_theta, reference_psi);
}

// Callback function to update current positions from /quadcopter/position topic
void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_x = msg->x;
    current_y = msg->y;
    current_z = msg->z;

    // ROS_INFO("Received current position low-level: [x=%.2f, y=%.2f, z=%.2f]", current_x, current_y, current_z);
}

// Callback function to update current Euler angles from /quadcopter/euler_angles topic
void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_phi = msg->x;   // Roll angle (phi)
    current_theta = msg->y; // Pitch angle (theta)
    current_psi = msg->z;   // Yaw angle (psi)

    // ROS_INFO("Current angles low-level: phi=%.5f, theta=%.5f, psi=%.5f", current_phi, current_theta, current_psi);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "low_level_cont");
    ros::NodeHandle nh;

    // Load PID gains and time step from the ROS parameter server
    double kp_torque1, ki_torque1, kd_torque1;
    double kp_torque2, ki_torque2, kd_torque2;
    double kp_torque3, ki_torque3, kd_torque3;
    double dt;

    nh.getParam("low_level_controller/kp_torque1", kp_torque1);
    nh.getParam("low_level_controller/ki_torque1", ki_torque1);
    nh.getParam("low_level_controller/kd_torque1", kd_torque1);
    nh.getParam("low_level_controller/kp_torque2", kp_torque2);
    nh.getParam("low_level_controller/ki_torque2", ki_torque2);
    nh.getParam("low_level_controller/kd_torque2", kd_torque2);
    nh.getParam("low_level_controller/kp_torque3", kp_torque3);
    nh.getParam("low_level_controller/ki_torque3", ki_torque3);
    nh.getParam("low_level_controller/kd_torque3", kd_torque3);
    nh.getParam("low_level_controller/dt", dt);

    // Set up ROS subscribers for reference angles, and current Euler angles
    ros::Subscriber reference_angles_sub = nh.subscribe("/control/reference_angles", 10, referenceAnglesCallback);
    ros::Subscriber current_euler_sub = nh.subscribe("/quadcopter/euler_angles", 10, currentEulerCallback);

    // Publisher for torques as a vector
    ros::Publisher torque_pub = nh.advertise<geometry_msgs::Vector3>("/control/torques", 10);

    ros::Rate rate(100); // Loop at 100 Hz

    while (ros::ok()) {
        // Spin once to update callbacks and get the latest reference and current angles
        ros::spinOnce();

        // Compute PID outputs for torques
        double torque1 = computePID(reference_phi, current_phi, prev_error_torque1, integral_torque1, kp_torque1, ki_torque1, kd_torque1, dt);
        double torque2 = computePID(reference_theta, current_theta, prev_error_torque2, integral_torque2, kp_torque2, ki_torque2, kd_torque2, dt);
        double torque3 = computePID(reference_psi, current_psi, prev_error_torque3, integral_torque3, kp_torque3, ki_torque3, kd_torque3, dt);

        // Publish the computed torques as a vector
        geometry_msgs::Vector3 torque_msg;
        torque_msg.x = torque1;
        torque_msg.y = torque2;
        torque_msg.z = torque3;
        torque_pub.publish(torque_msg);

        // Log the published values
        //ROS_INFO("Published torques low-level: [%.5f, %.5f, %.5f]", torque1, torque2, torque3);

        rate.sleep();
    }

    return 0;
}