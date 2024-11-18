#ifndef MID_LEVEL_CONTROLLER_H
#define MID_LEVEL_CONTROLLER_H

#include <std_msgs/Float64.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <Eigen/Dense>

namespace MidLevelNS {
    double computePID(double setpoint, double measured_value, double& prev_error, double& integral,
                      double kp, double ki, double kd, double dt, double integral_min, double integral_max);
}

class MidLevelController {
public:
    MidLevelController();
    void spin();

private:
    // ROS node handle, publishers, and subscribers
    ros::NodeHandle m_nh;
    ros::Subscriber m_desired_position_sub;
    ros::Subscriber m_current_position_sub;
    ros::Subscriber m_current_euler_sub;
    ros::Publisher m_desired_thrust_pub;
    ros::Publisher m_desired_angles_pub;

    // Target and current positions
    double m_reference_x, m_reference_y, m_reference_z, m_reference_psi;
    double m_current_x, m_current_y, m_current_z;
    double m_current_phi, m_current_theta, m_current_psi;

    // PID gains and parameters
    double m_kp_thrust, m_ki_thrust, m_kd_thrust;
    double m_kp_phi, m_ki_phi, m_kd_phi;
    double m_kp_theta, m_ki_theta, m_kd_theta;
    double m_min_thrust, m_max_thrust;
    double m_integral_min, m_integral_max;
    double m_dt;

    // PID state variables
    double m_prev_error_thrust, m_integral_thrust;
    double m_prev_error_ref_phi, m_integral_ref_phi;
    double m_prev_error_ref_theta, m_integral_ref_theta;

    // Core functions
    double computeThrust();
    geometry_msgs::Vector3 computeReferenceAngles();
    void publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles);
    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust);

    // Callbacks
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

#endif // MID_LEVEL_CONTROLLER_H
