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
    ros::NodeHandle nh;
    ros::Subscriber position_sub;
    ros::Subscriber current_position_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher thrust_pub;
    ros::Publisher ref_angles_pub;

    // Target and current positions
    double reference_x, reference_y, reference_z, reference_psi;
    double current_x, current_y, current_z;
    double current_phi, current_theta, current_psi;

    // PID gains and parameters
    double kp_thrust, ki_thrust, kd_thrust;
    double kp_phi, ki_phi, kd_phi;
    double kp_theta, ki_theta, kd_theta;
    double min_thrust, max_thrust;
    double integral_min, integral_max;
    double dt;

    // PID state variables
    double prev_error_thrust, integral_thrust;
    double prev_error_ref_phi, integral_ref_phi;
    double prev_error_ref_theta, integral_ref_theta;

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
