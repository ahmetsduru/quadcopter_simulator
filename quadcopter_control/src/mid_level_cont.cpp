#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

// PID controller state variables
double prev_error_thrust = 0.0, integral_thrust = 0.0;
double prev_error_ref_phi = 0.0, integral_ref_phi = 0.0;
double prev_error_ref_theta = 0.0, integral_ref_theta = 0.0;

// Variables to hold reference positions and reference psi angle
double reference_x = 0.0;
double reference_y = 0.0;
double reference_z = 0.0;
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

// Function to apply saturation limits to angles
double applySaturation(double angle, double min_angle, double max_angle) {
    if (angle > max_angle) {
        return max_angle;
    } else if (angle < min_angle) {
        return min_angle;
    }
    return angle;
}

// Function to apply saturation limits to thrust
double applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    if (thrust > max_thrust) {
        return max_thrust;
    } else if (thrust < min_thrust) {
        return min_thrust;
    }
    return thrust;
}

// Callback function to update reference positions from the /position topic
void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    // Extract position (x, y, z)
    reference_x = msg->x;
    reference_y = msg->y;
    reference_z = msg->z;

    //ROS_INFO("Received reference position from position topic: [x: %.2f, y: %.2f, z: %.2f]", reference_x, reference_y, reference_z);
}

// Callback function to update reference psi from the /psi topic
void psiCallback(const std_msgs::Float64::ConstPtr& msg) {
    // Extract yaw (reference psi)
    reference_psi = msg->data;
    //ROS_INFO("Received reference psi from psi topic: %.2f", reference_psi);
}

// Callback function to update current positions from /quadcopter/position topic
void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_x = msg->x;
    current_y = msg->y;
    current_z = msg->z;

    //ROS_INFO("Received current position mid-level: [x: %.2f, y: %.2f, z: %.2f]", current_x, current_y, current_z);
}

// Callback function to update current Euler angles from /quadcopter/euler_angles topic
void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_phi = msg->x;   // Roll angle (phi)
    current_theta = msg->y; // Pitch angle (theta)
    current_psi = msg->z;   // Yaw angle (psi)

    //ROS_INFO("Received current Euler angles mid-level: [phi: %.2f, theta: %.2f, psi: %.2f]", current_phi, current_theta, current_psi);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mid_level_cont");
    ros::NodeHandle nh;

    // Load PID gains and time step from the ROS parameter server
    double kp_thrust, ki_thrust, kd_thrust;
    double kp_phi, ki_phi, kd_phi;
    double kp_theta, ki_theta, kd_theta;
    double dt;

    nh.getParam("mid_level_controller/kp_thrust", kp_thrust);
    nh.getParam("mid_level_controller/ki_thrust", ki_thrust);
    nh.getParam("mid_level_controller/kd_thrust", kd_thrust);
    nh.getParam("mid_level_controller/kp_phi", kp_phi);
    nh.getParam("mid_level_controller/ki_phi", ki_phi);
    nh.getParam("mid_level_controller/kd_phi", kd_phi);
    nh.getParam("mid_level_controller/kp_theta", kp_theta);
    nh.getParam("mid_level_controller/ki_theta", ki_theta);
    nh.getParam("mid_level_controller/kd_theta", kd_theta);
    nh.getParam("mid_level_controller/dt", dt);

    // Set up ROS subscribers for reference positions and psi, and current pose
    ros::Subscriber position_sub = nh.subscribe("/position", 10, positionCallback); //ref
    ros::Subscriber psi_sub = nh.subscribe("/psi", 10, psiCallback); //ref
    ros::Subscriber current_position_sub = nh.subscribe("/quadcopter/position", 10, currentPositionCallback);
    ros::Subscriber current_euler_sub = nh.subscribe("/quadcopter/euler_angles", 10, currentEulerCallback);

    // Publishers for thrust and reference angles
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>("/control/thrust", 10);
    ros::Publisher ref_angles_pub = nh.advertise<geometry_msgs::Vector3>("/control/reference_angles", 10); // Publisher for reference phi, theta, psi as a vector

    ros::Rate rate(20); // Loop at 20 Hz

    // Set saturation limits in radians (-30 degrees to 30 degrees)
    double min_angle = -M_PI / 6;
    double max_angle = M_PI / 6;

    // Set thrust saturation limits (min and max thrust in Newtons)
    double min_thrust = 0.0;
    double max_thrust = 4.905; // Equivalent to 50% throttle for a typical drone

    while (ros::ok()) {
        // Ensure the latest callback data is processed
        ros::spinOnce();

        // Compute PID outputs for thrust, reference phi, and reference theta
        double thrust = computePID(reference_z, current_z, prev_error_thrust, integral_thrust, kp_thrust, ki_thrust, kd_thrust, dt) + 0.382 * 9.81;

        // Apply thrust saturation
        thrust = applyThrustSaturation(thrust, min_thrust, max_thrust);

        double ref_phi = computePID(reference_y, current_y, prev_error_ref_phi, integral_ref_phi, kp_phi, ki_phi, kd_phi, dt);
        double ref_theta = computePID(reference_x, current_x, prev_error_ref_theta, integral_ref_theta, kp_theta, ki_theta, kd_theta, dt);

        // Apply saturation limits to phi and theta
        ref_phi = applySaturation(ref_phi, min_angle, max_angle);
        ref_theta = applySaturation(ref_theta, min_angle, max_angle);

        // Publish the thrust control command
        std_msgs::Float64 thrust_msg;
        thrust_msg.data = thrust;
        thrust_pub.publish(thrust_msg);

        // Publish the reference angles (phi, theta, psi) as a vector
        geometry_msgs::Vector3 ref_angles_msg;
        ref_angles_msg.x = ref_phi;
        ref_angles_msg.y = ref_theta;
        ref_angles_msg.z = reference_psi;
        ref_angles_pub.publish(ref_angles_msg);

        // Log the published values
        //ROS_INFO("Published thrust mid-level: %.5f", thrust);
        //ROS_INFO("Published reference angles mid-level: ref_phi=%.5f, ref_theta=%.5f, reference_psi=%.5f", ref_phi, ref_theta, reference_psi);

        rate.sleep();
    }

    return 0;
}