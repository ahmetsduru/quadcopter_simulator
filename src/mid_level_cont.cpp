#include <std_msgs/Float64.h>
#include <Eigen/Geometry>
#include "../include/quadcopter_control/base_class.h"

class MidLevelController : public BaseClass { // Inherit from BaseClass
public:
    MidLevelController() {
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

    void spin() {
        ros::Rate rate(1/dt); // Loop at 20 Hz
        while (ros::ok()) {
            ros::spinOnce();

            double thrust = computeThrust();
            geometry_msgs::Vector3 ref_angles = computeReferenceAngles();
            publishControlSignals(thrust, ref_angles);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber position_sub;
    ros::Subscriber current_position_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher thrust_pub;
    ros::Publisher ref_angles_pub;

    double reference_x = 0.0;
    double reference_y = 0.0;
    double reference_z = 0.0;
    double reference_psi = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;
    double current_z = 0.0;
    double current_phi = 0.0;
    double current_theta = 0.0;
    double current_psi = 0.0;

    double min_thrust, max_thrust;

    double computeThrust() {
        double thrust = computePID(reference_z, current_z, prev_error_thrust, integral_thrust, kp_thrust, ki_thrust, kd_thrust, dt, integral_min, integral_max) + 0.382 * 9.81;
        return applyThrustSaturation(thrust, min_thrust, max_thrust);
    }

    geometry_msgs::Vector3 computeReferenceAngles() {
        geometry_msgs::Vector3 ref_angles;
        ref_angles.x = computePID(reference_y, current_y, prev_error_ref_phi, integral_ref_phi, kp_phi, ki_phi, kd_phi, dt, integral_min, integral_max);
        ref_angles.y = computePID(reference_x, current_x, prev_error_ref_theta, integral_ref_theta, kp_theta, ki_theta, kd_theta, dt, integral_min, integral_max);
        ref_angles.z = reference_psi;
        return ref_angles;
    }

    void publishControlSignals(double thrust, const geometry_msgs::Vector3& ref_angles) {
        std_msgs::Float64 thrust_msg;
        thrust_msg.data = thrust;
        thrust_pub.publish(thrust_msg);
        ref_angles_pub.publish(ref_angles);
    }

    double applyThrustSaturation(double thrust, double min_thrust, double max_thrust) {
        return std::max(min_thrust, std::min(thrust, max_thrust));
    }

    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        reference_x = msg->x;
        reference_y = msg->y;
        reference_z = msg->z;
    }

    void currentPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        current_x = msg->x;
        current_y = msg->y;
        current_z = msg->z;
    }

    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        current_phi = msg->x;
        current_theta = msg->y;
        current_psi = msg->z;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mid_level_cont");

    MidLevelController midcontroller;
    midcontroller.spin();

    return 0;
}
