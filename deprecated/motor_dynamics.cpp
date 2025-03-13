#include "../include/quadcopter_control/motor_dynamics.h"
#include <cmath>

RotorSpeedCalculator::RotorSpeedCalculator() {
    // ROS Subscribers
    m_des_thrust_sub = m_nh.subscribe("/reference_thrust", 10, &RotorSpeedCalculator::thrustCallback, this);
    m_des_torques_sub = m_nh.subscribe("/reference_torques", 10, &RotorSpeedCalculator::torquesCallback, this);

    // ROS Publisher
    m_rotor_speeds_pub = m_nh.advertise<std_msgs::Float64MultiArray>("/rotor_speeds", 10);

    // Initialize parameters
    m_nh.getParam("state_derivative_solver_node/kt_coeff", m_kt_coeff);
    m_nh.getParam("state_derivative_solver_node/km_coeff", m_km_coeff);
    m_nh.getParam("state_derivative_solver_node/l", m_arm_length);
    m_nh.getParam("state_derivative_solver_node/tau_thrust", m_tau_thrust);
    m_nh.getParam("state_derivative_solver_node/tau_torque", m_tau_torque);

    // Initialize variables
    m_des_thrust = 0.0;
    m_des_torques.setZero();
    m_actual_thrust = 0.0;
    m_actual_torques.setZero();
}

void RotorSpeedCalculator::spin() {
    ros::Rate rate(500); // 500 Hz loop rate
    while (ros::ok()) {
        // Update thrust and torques
        updateThrust(m_des_thrust, m_actual_thrust, m_tau_thrust, 1.0 / 500.0); // dt = 1/500 for 500 Hz
        updateTorques(m_des_torques, m_actual_torques, m_tau_torque, 1.0 / 500.0);

        // Compute and publish rotor speeds
        computeAndPublishRotorSpeeds();
        ros::spinOnce();
        rate.sleep();
    }
}

void RotorSpeedCalculator::thrustCallback(const std_msgs::Float64::ConstPtr& msg) {
    m_des_thrust = msg->data;
}

void RotorSpeedCalculator::torquesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_des_torques[0] = msg->x;
    m_des_torques[1] = msg->y;
    m_des_torques[2] = msg->z;
}

void RotorSpeedCalculator::computeAndPublishRotorSpeeds() {
    Eigen::Vector4d omega_squared;
    computeRotorSpeeds(m_actual_thrust, m_actual_torques, m_kt_coeff, m_km_coeff, m_arm_length, omega_squared);

    // Ensure non-negative values (for safety)
    for (int i = 0; i < 4; ++i) {
        if (omega_squared[i] < 0) {
            omega_squared[i] = 0.0;
        }
    }

    // Publish the rotor speeds
    std_msgs::Float64MultiArray rotor_speeds_msg;
    rotor_speeds_msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
        rotor_speeds_msg.data[i] = std::sqrt(omega_squared[i]);
    }
    m_rotor_speeds_pub.publish(rotor_speeds_msg);
}

void RotorSpeedCalculator::computeRotorSpeeds(double thrust, const Eigen::Vector3d& torques, double kt_coeff, double km_coeff, double l, Eigen::Vector4d& omega_squared) {
    Eigen::Matrix4d A;
    Eigen::Vector4d b;

    double a_term = kt_coeff * (l / sqrt(2));
    A << kt_coeff, kt_coeff, kt_coeff, kt_coeff,
         a_term, -a_term, -a_term, a_term,
         -a_term, a_term, -a_term, a_term,
         km_coeff, km_coeff, -km_coeff, -km_coeff;

    b << thrust, torques[0], torques[1], torques[2];
    omega_squared = A.colPivHouseholderQr().solve(b);
    //ROS_INFO_STREAM("Computed omega_squared values: " << omega_squared.transpose());
}

void RotorSpeedCalculator::updateThrust(double thrust_desired, double& thrust_actual, double tau_thrust, double dt) {
    double dF = (1.0 / tau_thrust) * (thrust_desired - thrust_actual);
    thrust_actual += dF * dt;
    //ROS_INFO_STREAM("Updated thrust: desired = " << thrust_desired << ", actual = " << thrust_actual);
}

void RotorSpeedCalculator::updateTorques(const Eigen::Vector3d& torques_desired, Eigen::Vector3d& torques_actual, double tau_torque, double dt) {
    Eigen::Vector3d dTau = (1.0 / tau_torque) * (torques_desired - torques_actual);
    torques_actual += dTau * dt;
    //ROS_INFO_STREAM("Updated torques: desired = " << torques_desired.transpose() << ", actual = " << torques_actual.transpose());
}
