#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <cmath>

class LowLevelController {
public:
    LowLevelController() {
        // Initialize state variables
        prev_error_torque1 = 0.0;
        integral_torque1 = 0.0;
        prev_error_torque2 = 0.0;
        integral_torque2 = 0.0;
        prev_error_torque3 = 0.0;
        integral_torque3 = 0.0;

        // Load PID gains and time step from the ROS parameter server
        nh.getParam("low_level_controller/kp_torque1", kp_torque1);
        nh.getParam("low_level_controller/ki_torque1", ki_torque1);
        nh.getParam("low_level_controller/kd_torque1", kd_torque1);
        nh.getParam("low_level_controller/kp_torque2", kp_torque2);
        nh.getParam("low_level_controller/ki_torque2", ki_torque2);
        nh.getParam("low_level_controller/kd_torque2", kd_torque2);
        nh.getParam("low_level_controller/kp_torque3", kp_torque3);
        nh.getParam("low_level_controller/ki_torque3", ki_torque3);
        nh.getParam("low_level_controller/kd_torque3", kd_torque3);
        nh.getParam("low_level_controller/dt", dt);

        // Set up ROS subscribers for reference angles and current Euler angles
        reference_angles_sub = nh.subscribe("/control/reference_angles", 10, &LowLevelController::referenceAnglesCallback, this);
        current_euler_sub = nh.subscribe("/quadcopter/euler_angles", 10, &LowLevelController::currentEulerCallback, this);

        // Publisher for torques as a vector
        torque_pub = nh.advertise<geometry_msgs::Vector3>("/control/torques", 10);
    }

    void spin() {
        ros::Rate rate(1/dt); // Loop at 100 Hz
        while (ros::ok()) {
            ros::spinOnce();

            // Compute and publish control signals (torques)
            double torque1 = computePID(reference_phi, current_phi, prev_error_torque1, integral_torque1, kp_torque1, ki_torque1, kd_torque1, dt);
            double torque2 = computePID(reference_theta, current_theta, prev_error_torque2, integral_torque2, kp_torque2, ki_torque2, kd_torque2, dt);
            double torque3 = computePID(reference_psi, current_psi, prev_error_torque3, integral_torque3, kp_torque3, ki_torque3, kd_torque3, dt);

            publishTorques(torque1, torque2, torque3);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber reference_angles_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher torque_pub;

    // PID controller state variables
    double prev_error_torque1, integral_torque1;
    double prev_error_torque2, integral_torque2;
    double prev_error_torque3, integral_torque3;

    // PID gains and time step
    double kp_torque1, ki_torque1, kd_torque1;
    double kp_torque2, ki_torque2, kd_torque2;
    double kp_torque3, ki_torque3, kd_torque3;
    double dt;

    // Reference angles
    double reference_phi = 0.0;
    double reference_theta = 0.0;
    double reference_psi = 0.0;

    // Current angles
    double current_phi = 0.0;
    double current_theta = 0.0;
    double current_psi = 0.0;

    // PID controller computation
    double computePID(double setpoint, double measured_value, double& prev_error, double& integral, double kp, double ki, double kd, double dt) {
        double error = setpoint - measured_value;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    // Callback function to update reference angles
    void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        reference_phi = msg->x;
        reference_theta = msg->y;
        reference_psi = msg->z;
    }

    // Callback function to update current Euler angles
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        current_phi = msg->x;   // Roll angle (phi)
        current_theta = msg->y; // Pitch angle (theta)
        current_psi = msg->z;   // Yaw angle (psi)
    }

    // Publish computed torques
    void publishTorques(double torque1, double torque2, double torque3) {
        geometry_msgs::Vector3 torque_msg;
        torque_msg.x = torque1;
        torque_msg.y = torque2;
        torque_msg.z = torque3;
        torque_pub.publish(torque_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "low_level_cont");

    LowLevelController controller;
    controller.spin();

    return 0;
}
