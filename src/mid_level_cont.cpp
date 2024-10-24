#include "../include/quadcopter_control/mid_level_cont.h"

MidLevelController::MidLevelController() {
    // Load PID gains and time step from the ROS parameter server
    nh.getParam("mid_level_controller/kp_thrust", kp_thrust);
    nh.getParam("mid_level_controller/ki_thrust", ki_thrust);
    nh.getParam("mid_level_controller/kd_thrust", kd_thrust);
    nh.getParam("mid_level_controller/kp_phi", kp_phi);
    nh.getParam("mid_level_controller/ki_phi", ki_phi);
    nh.getParam("mid_level_controller/kd_phi", kd_phi);
    nh.getParam("mid_level_controller/kp_theta", kp_theta);
    nh.getParam("mid_level_controller/ki_theta", ki_theta);
    nh.getParam("mid_level_controller/kd_theta", kd_theta);
    nh.getParam("mid_level_controller/dt", dt);
    nh.getParam("mid_level_controller/min_thrust", min_thrust);
    nh.getParam("mid_level_controller/max_thrust", max_thrust);
    nh.getParam("mid_level_controller/integral_min", integral_min);
    nh.getParam("mid_level_controller/integral_max", integral_max);

    // Set up ROS subscribers and publishers
    position_sub = nh.subscribe("/reference_position", 10, &MidLevelController::positionCallback, this);
    current_position_sub = nh.subscribe("/actual_position", 10, &MidLevelController::currentPositionCallback, this);
    current_euler_sub = nh.subscribe("/actual_euler_angles", 10, &MidLevelController::currentEulerCallback, this);

    thrust_pub = nh.advertise<std_msgs::Float64>("/reference_thrust", 10);
    ref_angles_pub = nh.advertise<geometry_msgs::Vector3>("/reference_euler_angles", 10);
}

void MidLevelController::spin() {
    ros::Rate rate(1 / dt); // Loop at 20 Hz
    while (ros::ok()) {
        ros::spinOnce();

        double thrust = computeThrust();
        geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
        publishControlSignals(thrust, ref_angles);

        rate.sleep();
    }
}

double MidLevelController::computeThrust() {
    double thrust = computePID(reference_z, current_z, prev_error_thrust, integral_thrust, kp_thrust, ki_thrust, kd_thrust, dt, integral_min, integral_max) + 0.382 * 9.81;
    return applyThrustSaturation(thrust, min_thrust, max_thrust);
}

geometry_msgs::Vector3 MidLevelController::computeReferenceAngles() {
    geometry_msgs::Vector3 ref_angles;
    ref_angles.x = computePID(reference_y, current_y, prev_error_ref_phi, integral_ref_phi, kp_phi, ki_phi, kd_phi, dt, integral_min, integral_max);
    ref_angles.y = computePID(reference_x, current_x, prev_error_ref_theta, integral_ref_theta, kp_theta, ki_theta, kd_theta, dt, integral_min, integral_max);
    ref_angles.z = reference_psi;
    return ref_angles;
}

void MidLevelController::publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
    std_msgs::Float64 thrust_msg;
    thrust_msg.data = thrust;
    thrust_pub.publish(thrust_msg);
    ref_angles_pub.publish(ref_angles);
}

double MidLevelController::applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
    return std::max(min_thrust, std::min(thrust, max_thrust));
}

void MidLevelController::positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    reference_x = msg->x;
    reference_y = msg->y;
    reference_z = msg->z;
}

void MidLevelController::currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_x = msg->x;
    current_y = msg->y;
    current_z = msg->z;
}

void MidLevelController::currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    current_phi = msg->x;
    current_theta = msg->y;
    current_psi = msg->z;
}
