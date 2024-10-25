#ifndef LOWLEVELCONTROLLER_H
#define LOWLEVELCONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

namespace LowLevelNS {
    double computePID(double setpoint, double measured_value, double& prev_error, double& integral,
                      double kp, double ki, double kd, double dt, double integral_min, double integral_max);
}

class LowLevelController {
public:
    LowLevelController();
    void spin();

private:
    ros::NodeHandle nh;
    ros::Subscriber reference_angles_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher torque_pub;

    // PID gains and parameters
    double kp_torque1, ki_torque1, kd_torque1;
    double kp_torque2, ki_torque2, kd_torque2;
    double kp_torque3, ki_torque3, kd_torque3;
    double dt;
    double integral_min, integral_max;

    // Reference angles
    double reference_phi;
    double reference_theta;
    double reference_psi;

    // Current angles
    double current_phi;
    double current_theta;
    double current_psi;

    // PID internal states
    double prev_error_torque1;
    double prev_error_torque2;
    double prev_error_torque3;
    double integral_torque1;
    double integral_torque2;
    double integral_torque3;

    // Callback functions
    void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    // Helper functions
    void publishTorques(double torque1, double torque2, double torque3);
};

#endif // LOWLEVELCONTROLLER_H
