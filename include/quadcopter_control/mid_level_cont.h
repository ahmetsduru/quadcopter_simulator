#ifndef MIDLEVELCONTROLLER_H
#define MIDLEVELCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include "base_class.h"

class MidLevelController : public BaseClass {
public:
    MidLevelController();
    void spin();

private:
    ros::NodeHandle nh;
    ros::Subscriber position_sub;
    ros::Subscriber current_position_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher thrust_pub;
    ros::Publisher ref_angles_pub;

    double reference_x;
    double reference_y;
    double reference_z;
    double reference_psi;
    double current_x;
    double current_y;
    double current_z;
    double current_phi;
    double current_theta;
    double current_psi;

    double min_thrust;
    double max_thrust;

    double kp_thrust, ki_thrust, kd_thrust;
    double kp_phi, ki_phi, kd_phi;
    double kp_theta, ki_theta, kd_theta;
    double dt;
    double integral_min, integral_max;

    double prev_error_thrust = 0.0;
    double prev_error_ref_phi = 0.0;
    double prev_error_ref_theta = 0.0;

    double integral_thrust = 0.0;
    double integral_ref_phi = 0.0;
    double integral_ref_theta = 0.0;

    double computeThrust();
    geometry_msgs::Vector3 computeReferenceAngles();
    void publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles);
    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust);

    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg);
};

#endif // MIDLEVELCONTROLLER_H
