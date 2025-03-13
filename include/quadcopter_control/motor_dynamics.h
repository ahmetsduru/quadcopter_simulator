#ifndef MOTOR_DYNAMICS_H
#define MOTOR_DYNAMICS_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

class RotorSpeedCalculator {
public:
    RotorSpeedCalculator();
    void spin();

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_des_thrust_sub;
    ros::Subscriber m_des_torques_sub;
    ros::Publisher m_rotor_speeds_pub;

    double m_des_thrust, m_actual_thrust, m_tau_thrust, m_tau_torque;
    Eigen::Vector3d m_des_torques, m_actual_torques;

    // Parameters
    double m_kt_coeff;
    double m_km_coeff;
    double m_arm_length;

    void thrustCallback(const std_msgs::Float64::ConstPtr& msg);
    void torquesCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void computeAndPublishRotorSpeeds();
    void computeRotorSpeeds(double thrust, const Eigen::Vector3d& torques, double kt_coeff, double km_coeff, double l, Eigen::Vector4d& omega_squared);
    void updateThrust(double thrust_desired, double& thrust_actual, double tau_thrust, double dt);
    void updateTorques(const Eigen::Vector3d& torques_desired, Eigen::Vector3d& torques_actual, double tau_torque, double dt);
};

#endif // MOTOR_DYNAMICS_H
