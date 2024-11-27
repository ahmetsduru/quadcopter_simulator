#include "../include/quadcopter_control/mid_level_cont.h"

namespace MidLevelNS {

double computePID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
                  double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double pos_error = des_pos - a_pos;
    double vel_error = des_vel - a_vel;
    
    integral += pos_error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }

    return kp * pos_error + ki * integral + kd * vel_error;
}
}

MidLevelController::MidLevelController() 
    : m_des_x(0.0), m_des_y(0.0), m_des_z(0.0), m_des_psi(0.0),
      m_current_x(0.0), m_current_y(0.0), m_current_z(0.0),
      m_current_phi(0.0), m_current_theta(0.0), m_current_psi(0.0),
      m_prev_error_thrust_x(0.0), m_integral_thrust_x(0.0),
      m_prev_error_thrust_y(0.0), m_integral_thrust_y(0.0),
      m_prev_error_thrust_z(0.0), m_integral_thrust_z(0.0),
      m_prev_error_ref_phi(0.0), m_integral_ref_phi(0.0),
      m_prev_error_ref_theta(0.0), m_integral_ref_theta(0.0),
      m_desired_acceleration_x(0.0), m_desired_acceleration_y(0.0), m_desired_acceleration_z(0.0) {

    // Load parameters from the parameter server
    m_nh.getParam("mid_level_controller/kp_thrust_x", m_kp_thrust_x);
    m_nh.getParam("mid_level_controller/ki_thrust_x", m_ki_thrust_x);
    m_nh.getParam("mid_level_controller/kd_thrust_x", m_kd_thrust_x);
    m_nh.getParam("mid_level_controller/kp_thrust_y", m_kp_thrust_y);
    m_nh.getParam("mid_level_controller/ki_thrust_y", m_ki_thrust_y);
    m_nh.getParam("mid_level_controller/kd_thrust_y", m_kd_thrust_y);
    m_nh.getParam("mid_level_controller/kp_thrust_z", m_kp_thrust_z);
    m_nh.getParam("mid_level_controller/ki_thrust_z", m_ki_thrust_z);
    m_nh.getParam("mid_level_controller/kd_thrust_z", m_kd_thrust_z);
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
    m_desired_acceleration_sub = m_nh.subscribe("/reference_acceleration", 10, &MidLevelController::desiredAccelerationCallback, this);
    m_desired_velocity_sub = m_nh.subscribe("/reference_velocity", 10, &MidLevelController::desiredVelocityCallback, this);
    m_current_velocity_sub = m_nh.subscribe("/actual_velocity", 10, &MidLevelController::currentVelocityCallback, this);
    m_desired_psi_sub = m_nh.subscribe("/reference_psi", 10, &MidLevelController::desiredPsiCallback, this);

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
    // Kuvvetlerin dünya çerçevesinde hesaplanması
    m_thrust_x = MidLevelNS::computePID(m_des_x, m_current_x, m_desired_velocity_x, m_current_velocity_x, m_integral_thrust_x, m_kp_thrust_x, m_ki_thrust_x, m_kd_thrust_x, m_dt, m_integral_min, m_integral_max) + 0.382 * m_desired_acceleration_x;
    m_thrust_y = MidLevelNS::computePID(m_des_y, m_current_y, m_desired_velocity_y, m_current_velocity_y, m_integral_thrust_y, m_kp_thrust_y, m_ki_thrust_y, m_kd_thrust_y, m_dt, m_integral_min, m_integral_max) + 0.382 * m_desired_acceleration_y;
    m_thrust_z = MidLevelNS::computePID(m_des_z, m_current_z, m_desired_velocity_z, m_current_velocity_z, m_integral_thrust_z, m_kp_thrust_z, m_ki_thrust_z, m_kd_thrust_z, m_dt, m_integral_min, m_integral_max) + 0.382 * m_desired_acceleration_z + 0.382 * 9.81;

    // Dünya çerçevesindeki kuvvetlerin loglanması
    //ROS_INFO("World frame thrusts -> X: %f, Y: %f, Z: %f", m_thrust_x, m_thrust_y, m_thrust_z);

    // Referans Euler açılarını hesapla
    geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
    double phi_ref = ref_angles.x;   // Referans Roll
    double theta_ref = ref_angles.y; // Referans Pitch
    double psi_ref = ref_angles.z;   // Referans Yaw

    // Dönüşüm matrisi (world -> body)
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 
        cos(psi_ref) * cos(theta_ref), sin(psi_ref) * cos(theta_ref), -sin(theta_ref),
        cos(psi_ref) * sin(theta_ref) * sin(phi_ref) - sin(psi_ref) * cos(phi_ref), sin(psi_ref) * sin(theta_ref) * sin(phi_ref) + cos(psi_ref) * cos(phi_ref), cos(theta_ref) * sin(phi_ref),
        cos(psi_ref) * sin(theta_ref) * cos(phi_ref) + sin(psi_ref) * sin(phi_ref), sin(psi_ref) * sin(theta_ref) * cos(phi_ref) - cos(psi_ref) * sin(phi_ref), cos(theta_ref) * cos(phi_ref);

    Eigen::Vector3d world_forces(m_thrust_x, m_thrust_y, m_thrust_z);
    Eigen::Vector3d body_forces = rotation_matrix * world_forces;
    //ROS_INFO("Body frame thrusts -> X: %f, Y: %f, Z: %f", body_forces.x(), body_forces.y(), body_forces.z());

    return applyThrustSaturation(body_forces.z(), m_min_thrust, m_max_thrust);
}

geometry_msgs::Vector3 MidLevelController::computeReferenceAngles() {
    geometry_msgs::Vector3 ref_angles;

    // Safeguard against division by zero or small thrust values
    if (std::abs(m_thrust_z) < 1e-5) {
        ROS_WARN("Thrust Z is too small. Setting reference angles to zero.");
        ref_angles.x = 0.0;
        ref_angles.y = 0.0;
        ref_angles.z = m_des_psi;
        return ref_angles;
    }

    // Compute reference roll and pitch angles
    ref_angles.x = (m_thrust_x * sin(m_current_psi) - m_thrust_y * cos(m_current_psi)) / m_thrust_z;
    ref_angles.y = (m_thrust_x * cos(m_current_psi) + m_thrust_y * sin(m_current_psi)) / m_thrust_z;    
    ref_angles.z = m_des_psi;

    return ref_angles;
}

void MidLevelController::publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
    std_msgs::Float64 thrust_msg;
    thrust_msg.data = thrust;
    m_desired_thrust_pub.publish(thrust_msg);
    m_desired_angles_pub.publish(ref_angles);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    double saturated_thrust = std::max(min_thrust, std::min(thrust, max_thrust));
    return saturated_thrust;
}

void MidLevelController::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_des_x = msg->x;
    m_des_y = msg->y;
    m_des_z = msg->z;
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

void MidLevelController::desiredVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_velocity_x = msg->x;
    m_desired_velocity_y = msg->y;
    m_desired_velocity_z = msg->z;
}

void MidLevelController::currentVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_velocity_x = msg->x;
    m_current_velocity_y = msg->y;
    m_current_velocity_z = msg->z;
}

void MidLevelController::desiredAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_desired_acceleration_x = msg->x;
    m_desired_acceleration_y = msg->y;
    m_desired_acceleration_z = msg->z;
}

void MidLevelController::desiredPsiCallback(const std_msgs::Float64::ConstPtr& msg) {
    m_des_psi = msg->data;
}