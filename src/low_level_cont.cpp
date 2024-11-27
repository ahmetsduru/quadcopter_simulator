#include "../include/quadcopter_control/low_level_cont.h"
#include <cmath>

namespace LowLevelNS {

double computePID(double setpoint, double measured_value, double& prev_error, double& integral,
                  double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double error = setpoint - measured_value;

    integral += error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }

    double derivative = (error - prev_error) / dt;
    prev_error = error;

    return kp * error + ki * integral + kd * derivative;
}

} 

LowLevelController::LowLevelController()
    : reference_phi(0.0), reference_theta(0.0), reference_psi(0.0), 
    current_phi(0.0), current_theta(0.0), current_psi(0.0), 
    prev_error_torque1(0.0), prev_error_torque2(0.0), prev_error_torque3(0.0),
    integral_torque1(0.0), integral_torque2(0.0), integral_torque3(0.0) {

    // Load PID gains and time step from the ROS parameter server
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
    nh.getParam("low_level_controller/integral_min", integral_min);
    nh.getParam("low_level_controller/integral_max", integral_max);

    reference_angles_sub = nh.subscribe("/reference_euler_angles", 10, &LowLevelController::referenceAnglesCallback, this);
    current_euler_sub = nh.subscribe("/actual_euler_angles", 10, &LowLevelController::currentEulerCallback, this);
    torque_pub = nh.advertise<geometry_msgs::Vector3>("/reference_torques", 10);
}

void LowLevelController::spin() {
    ros::Rate rate(1/dt); // Loop at 100 Hz
    while (ros::ok()) {
        ros::spinOnce();

        double torque1 = LowLevelNS::computePID(reference_phi, current_phi, prev_error_torque1, integral_torque1, kp_torque1, ki_torque1, kd_torque1, dt, integral_min, integral_max); 
        double torque2 = LowLevelNS::computePID(reference_theta, current_theta, prev_error_torque2, integral_torque2, kp_torque2, ki_torque2, kd_torque2, dt, integral_min, integral_max);
        double torque3 = LowLevelNS::computePID(reference_psi, current_psi, prev_error_torque3, integral_torque3, kp_torque3, ki_torque3, kd_torque3, dt, integral_min, integral_max);
        publishTorques(torque1, torque2, torque3);

        rate.sleep();
    }
}

void LowLevelController::referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    reference_phi = msg->x;
    reference_theta = msg->y;
    reference_psi = msg->z;
}

void LowLevelController::currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_phi = msg->x;
    current_theta = msg->y;
    current_psi = msg->z;
}

void LowLevelController::publishTorques(double torque1, double torque2, double torque3) {
    geometry_msgs::Vector3 torque_msg;
    torque_msg.x = torque1;
    torque_msg.y = torque2;
    torque_msg.z = torque3;
    torque_pub.publish(torque_msg);
}

