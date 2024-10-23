#include "../include/quadcopter_control/base_class.h"

class LowLevelController : public BaseClass { // Inherit from BaseClass
public:
    LowLevelController() {
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
        nh.getParam("low_level_controller/integral_min", integral_min);
        nh.getParam("low_level_controller/integral_max", integral_max);

        reference_angles_sub = nh.subscribe("/reference_euler_angles", 10, &LowLevelController::referenceAnglesCallback, this);
        current_euler_sub = nh.subscribe("/actual_euler_angles", 10, &LowLevelController::currentEulerCallback, this);
        torque_pub = nh.advertise<geometry_msgs::Vector3>("/reference_torques", 10);
    }

    void spin() {
        ros::Rate rate(1/dt); // Loop at 100 Hz
        while (ros::ok()) {
            ros::spinOnce();

            double torque1 = computePID(reference_phi, current_phi, prev_error_torque1, integral_torque1, kp_torque1, ki_torque1, kd_torque1, dt, integral_min, integral_max);
            double torque2 = computePID(reference_theta, current_theta, prev_error_torque2, integral_torque2, kp_torque2, ki_torque2, kd_torque2, dt, integral_min, integral_max);
            double torque3 = computePID(reference_psi, current_psi, prev_error_torque3, integral_torque3, kp_torque3, ki_torque3, kd_torque3, dt, integral_min, integral_max);

            publishTorques(torque1, torque2, torque3);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber reference_angles_sub;
    ros::Subscriber current_euler_sub;
    ros::Publisher torque_pub;

    double reference_phi = 0.0;
    double reference_theta = 0.0;
    double reference_psi = 0.0;

    double current_phi = 0.0;
    double current_theta = 0.0;
    double current_psi = 0.0;

    void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        reference_phi = msg->x;
        reference_theta = msg->y;
        reference_psi = msg->z;
    }

    void currentEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        current_phi = msg->x;
        current_theta = msg->y;
        current_psi = msg->z;
    }

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

    LowLevelController lowcontroller;
    lowcontroller.spin();

    return 0;
}
