#include "../include/quadcopter_control/base_class.h"

BaseClass::BaseClass() {
    // PID durum değişkenlerini başlatma
    prev_error_thrust = 0.0;
    integral_thrust = 0.0;
    prev_error_ref_phi = 0.0;
    integral_ref_phi = 0.0;
    prev_error_ref_theta = 0.0;
    integral_ref_theta = 0.0;
    prev_error_torque1 = 0.0;
    integral_torque1 = 0.0;
    prev_error_torque2 = 0.0;
    integral_torque2 = 0.0;
    prev_error_torque3 = 0.0;
    integral_torque3 = 0.0;
}

double BaseClass::computePID(double setpoint, double measured_value, double& prev_error, double& integral,
                             double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double error = setpoint - measured_value;
    
    // Compute the integral with windup protection (bounded integral)
    integral += error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }
    
    // Compute derivative term
    double derivative = (error - prev_error) / dt;
    
    // Update previous error
    prev_error = error;
    
    // Return the PID output
    return kp * error + ki * integral + kd * derivative;
}
