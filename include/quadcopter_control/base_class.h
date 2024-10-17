#ifndef BASE_CLASS_H
#define BASE_CLASS_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class BaseClass {
public:
    BaseClass();

    // PID denetleyici hesaplaması tüm türetilmiş sınıflar için
    double computePID(double setpoint, double measured_value, double& prev_error, double& integral,
                      double kp, double ki, double kd, double dt);

protected:
    // PID denetleyici durum değişkenleri
    double prev_error_thrust, integral_thrust;
    double prev_error_ref_phi, integral_ref_phi;
    double prev_error_ref_theta, integral_ref_theta;
    double prev_error_torque1, integral_torque1;
    double prev_error_torque2, integral_torque2;
    double prev_error_torque3, integral_torque3;

    // PID kazançları (türetilmiş sınıflarda ayarlanacak)
    double kp_thrust, ki_thrust, kd_thrust;
    double kp_phi, ki_phi, kd_phi;
    double kp_theta, ki_theta, kd_theta;
    double kp_torque1, ki_torque1, kd_torque1;
    double kp_torque2, ki_torque2, kd_torque2;
    double kp_torque3, ki_torque3, kd_torque3;
    double dt;
};

#endif // BASE_CLASS_H
