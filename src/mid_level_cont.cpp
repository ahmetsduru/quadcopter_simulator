#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

class MidLevelController {
public:
    MidLevelController() {
        // Initialize state variables
        prev_error_thrust = 0.0;
        integral_thrust = 0.0;
        prev_error_ref_phi = 0.0;
        integral_ref_phi = 0.0;
        prev_error_ref_theta = 0.0;
        integral_ref_theta = 0.0;

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

        // Set up ROS subscribers for reference positions, psi, and current pose
        position_sub = nh.subscribe("/position", 10, &MidLevelController::positionCallback, this);
        psi_sub = nh.subscribe("/psi", 10, &MidLevelController::psiCallback, this);
        current_position_sub = nh.subscribe("/quadcopter/position", 10, &MidLevelController::currentPositionCallback, this);
        current_euler_sub = nh.subscribe("/quadcopter/euler_angles", 10, &MidLevelController::currentEulerCallback, this);

        // Publishers for thrust and reference angles
        thrust_pub = nh.advertise<std_msgs::Float64>("/control/thrust", 10);
        ref_angles_pub = nh.advertise<geometry_msgs::Vector3>("/control/reference_angles", 10);

        // Set thrust saturation limits (min and max thrust in Newtons)
        min_thrust = 0.0;
        max_thrust = 4.905; // Equivalent to 50% throttle for a typical drone
    }

    void spin() {
        ros::Rate rate(1/dt); // Loop at 20 Hz
        while (ros::ok()) {
            ros::spinOnce();

            // Compute and publish control signals
            double thrust = computeThrust();
            geometry_msgs::Vector3 ref_angles = computeReferenceAngles();

            publishControlSignals(thrust, ref_angles);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;

    // ROS subscribers and publishers
    ros::Subscriber position_sub;
    ros::Subscriber psi_sub;
    ros::Subscriber current_position_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher thrust_pub;
    ros::Publisher ref_angles_pub;

    // PID controller state variables
    double prev_error_thrust, integral_thrust;
    double prev_error_ref_phi, integral_ref_phi;
    double prev_error_ref_theta, integral_ref_theta;

    // PID gains
    double kp_thrust, ki_thrust, kd_thrust;
    double kp_phi, ki_phi, kd_phi;
    double kp_theta, ki_theta, kd_theta;
    double dt;

    // Reference positions and angles
    double reference_x = 0.0;
    double reference_y = 0.0;
    double reference_z = 0.0;
    double reference_psi = 0.0;

    // Current positions and angles
    double current_x = 0.0;
    double current_y = 0.0;
    double current_z = 0.0;
    double current_phi = 0.0;
    double current_theta = 0.0;
    double current_psi = 0.0;

    // Thrust saturation limits
    double min_thrust, max_thrust;

    // PID controller computation
    double computePID(double setpoint, double measured_value, double& prev_error, double& integral, double kp, double ki, double kd, double dt) {
        double error = setpoint - measured_value;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    // Apply saturation limits to thrust
    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
        return std::max(min_thrust, std::min(thrust, max_thrust));
    }

    // Compute thrust using PID
    double computeThrust() {
        double thrust = computePID(reference_z, current_z, prev_error_thrust, integral_thrust, kp_thrust, ki_thrust, kd_thrust, dt) + 0.382 * 9.81;
        return applyThrustSaturation(thrust, min_thrust, max_thrust);
    }

    // Compute reference angles using PID
    geometry_msgs::Vector3 computeReferenceAngles() {
        geometry_msgs::Vector3 ref_angles;

        ref_angles.x = computePID(reference_y, current_y, prev_error_ref_phi, integral_ref_phi, kp_phi, ki_phi, kd_phi, dt);
        ref_angles.y = computePID(reference_x, current_x, prev_error_ref_theta, integral_ref_theta, kp_theta, ki_theta, kd_theta, dt);
        ref_angles.z = reference_psi;

        return ref_angles;
    }

    // Publish control signals
    void publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
        std_msgs::Float64 thrust_msg;
        thrust_msg.data = thrust;
        thrust_pub.publish(thrust_msg);

        ref_angles_pub.publish(ref_angles);
    }

    // Callback function to update reference positions
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        reference_x = msg->x;
        reference_y = msg->y;
        reference_z = msg->z;
    }

    // Callback function to update reference psi
    void psiCallback(const std_msgs::Float64::ConstPtr& msg) {
        reference_psi = msg->data;
    }

    // Callback function to update current positions
    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        current_x = msg->x;
        current_y = msg->y;
        current_z = msg->z;
    }

    // Callback function to update current Euler angles
    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        current_phi = msg->x;
        current_theta = msg->y;
        current_psi = msg->z;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mid_level_cont");

    MidLevelController controller;
    controller.spin();

    return 0;
}
