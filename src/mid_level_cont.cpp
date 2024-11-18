#include "../include/quadcopter_control/mid_level_cont.h"

namespace MidLevelNS {

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

MidLevelController::MidLevelController() 
    : m_reference_x(0.0), m_reference_y(0.0), m_reference_z(0.0), m_reference_psi(0.0),
      m_current_x(0.0), m_current_y(0.0), m_current_z(0.0),
      m_current_phi(0.0), m_current_theta(0.0), m_current_psi(0.0),
      m_prev_error_thrust(0.0), m_integral_thrust(0.0),
      m_prev_error_ref_phi(0.0), m_integral_ref_phi(0.0),
      m_prev_error_ref_theta(0.0), m_integral_ref_theta(0.0) {

    // Load parameters from the parameter server
    m_nh.getParam("mid_level_controller/kp_thrust", m_kp_thrust);
    m_nh.getParam("mid_level_controller/ki_thrust", m_ki_thrust);
    m_nh.getParam("mid_level_controller/kd_thrust", m_kd_thrust);
    m_nh.getParam("mid_level_controller/kp_phi", m_kp_phi);
    m_nh.getParam("mid_level_controller/ki_phi", m_ki_phi);
    m_nh.getParam("mid_level_controller/kd_phi", m_kd_phi);
    m_nh.getParam("mid_level_controller/kp_theta", m_kp_theta);
    m_nh.getParam("mid_level_controller/ki_theta", m_ki_theta);
    m_nh.getParam("mid_level_controller/kd_theta", m_kd_theta);
    m_nh.getParam("mid_level_controller/dt", m_dt);
    m_nh.getParam("mid_level_controller/min_thrust", m_min_thrust);
    m_nh.getParam("mid_level_controller/max_thrust", m_max_thrust);
    m_nh.getParam("mid_level_controller/integral_min", m_integral_min);
    m_nh.getParam("mid_level_controller/integral_max", m_integral_max);

    // Initialize subscribers and publishers
    m_desired_position_sub = m_nh.subscribe("/reference_position", 10, &MidLevelController::positionCallback, this);
    m_current_position_sub = m_nh.subscribe("/actual_position", 10, &MidLevelController::currentPositionCallback, this);
    m_current_euler_sub = m_nh.subscribe("/actual_euler_angles", 10, &MidLevelController::currentEulerCallback, this);

    m_desired_thrust_pub = m_nh.advertise<std_msgs::Float64>("/reference_thrust", 10);
    m_desired_angles_pub = m_nh.advertise<geometry_msgs::Vector3>("/reference_euler_angles", 10);
}

void MidLevelController::spin() {
    ros::Rate rate(1 / m_dt); 
    while (ros::ok()) {
        ros::spinOnce();

        double thrust = computeThrust();
        geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
        publishControlSignals(thrust, ref_angles);

        rate.sleep();
    }
}

double MidLevelController::computeThrust() {
    double thrust = MidLevelNS::computePID(m_reference_z, m_current_z, m_prev_error_thrust, m_integral_thrust,
                               m_kp_thrust, m_ki_thrust, m_kd_thrust, m_dt, m_integral_min, m_integral_max) + 0.382 * 9.81;
    return applyThrustSaturation(thrust, m_min_thrust, m_max_thrust);
}

geometry_msgs::Vector3 MidLevelController::computeReferenceAngles() {
    geometry_msgs::Vector3 ref_angles;
    ref_angles.x = MidLevelNS::computePID(m_reference_y, m_current_y, m_prev_error_ref_phi, m_integral_ref_phi,
                              m_kp_phi, m_ki_phi, m_kd_phi, m_dt, m_integral_min, m_integral_max);
    ref_angles.y = MidLevelNS::computePID(m_reference_x, m_current_x, m_prev_error_ref_theta, m_integral_ref_theta,
                              m_kp_theta, m_ki_theta, m_kd_theta, m_dt, m_integral_min, m_integral_max);
    ref_angles.z = m_reference_psi;
    return ref_angles;
}

void MidLevelController::publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
    std_msgs::Float64 thrust_msg;
    thrust_msg.data = thrust;
    m_desired_thrust_pub.publish(thrust_msg);
    m_desired_angles_pub.publish(ref_angles);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    return std::max(min_thrust, std::min(thrust, max_thrust));
}

void MidLevelController::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_reference_x = msg->x;
    m_reference_y = msg->y;
    m_reference_z = msg->z;
}

void MidLevelController::currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_x = msg->x;
    m_current_y = msg->y;
    m_current_z = msg->z;
}

void MidLevelController::currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_phi = msg->x;
    m_current_theta = msg->y;
    m_current_psi = msg->z;
}
