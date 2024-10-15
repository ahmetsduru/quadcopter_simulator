#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <tf2/LinearMath/Quaternion.h>

// Simple PID controller function
double pidControl(double desired, double current, double Kp, double Ki, double Kd, double &integral, double &previous_error, double dt)
{
    double error = desired - current;
    integral += error * dt;
    double derivative = (error - previous_error) / dt;
    previous_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

// Global variables for current position, reference position, and psi angle
geometry_msgs::Vector3 current_position;
geometry_msgs::Vector3 reference_position;
double current_psi = 0.0;

// PID constants and variables
double Kp_x, Ki_x, Kd_x;
double Kp_y, Ki_y, Kd_y;
double Kp_z, Ki_z, Kd_z;
double integral_x = 0.0, integral_y = 0.0, integral_z = 0.0;
double previous_error_x = 0.0, previous_error_y = 0.0, previous_error_z = 0.0;

// Thrust parameters
double max_thrust;
double min_thrust;
double hz;

// Callback function for current position data from vicon/quad/quad
void viconBridgeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    current_position.x = msg->transform.translation.x;
    current_position.y = msg->transform.translation.y;
    current_position.z = msg->transform.translation.z;
}

// Callback function for reference position data
void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    reference_position.x = msg->x;
    reference_position.y = msg->y;
    reference_position.z = msg->z;
}

// Callback function for psi angle data
void psiCallback(const std_msgs::Float64::ConstPtr& msg)
{
    current_psi = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "real_time_control");
    ros::NodeHandle nh;

    // Load PID parameters and other settings from the ROS parameter server using the real_time_control namespace
    if (!nh.getParam("real_time_control/Kp_x", Kp_x) ||
        !nh.getParam("real_time_control/Ki_x", Ki_x) ||
        !nh.getParam("real_time_control/Kd_x", Kd_x) ||
        !nh.getParam("real_time_control/Kp_y", Kp_y) ||
        !nh.getParam("real_time_control/Ki_y", Ki_y) ||
        !nh.getParam("real_time_control/Kd_y", Kd_y) ||
        !nh.getParam("real_time_control/Kp_z", Kp_z) ||
        !nh.getParam("real_time_control/Ki_z", Ki_z) ||
        !nh.getParam("real_time_control/Kd_z", Kd_z) ||
        !nh.getParam("real_time_control/max_thrust", max_thrust) ||
        !nh.getParam("real_time_control/min_thrust", min_thrust) ||
        !nh.getParam("real_time_control/hz", hz))
    {
        ROS_ERROR("Failed to load parameters. Please check the parameter server settings.");
        return 1;
    }

    // Initialize the publishers for control commands
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Publisher thrust_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_raw/thrust", 10);

    // Initialize the subscribers
    ros::Subscriber vicon_bridge_sub = nh.subscribe("vicon/quad/quad", 10, viconBridgeCallback);
    ros::Subscriber reference_position_sub = nh.subscribe("position", 10, positionCallback);
    ros::Subscriber psi_sub = nh.subscribe("/psi", 10, psiCallback);

    // Services to arm the drone and set the flight mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Arm the drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone armed");
    } else {
        ROS_ERROR("Failed to arm the drone");
        return 1;
    }

    // Set the flight mode to GUIDED_NOGPS
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "GUIDED_NOGPS";
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("GUIDED_NOGPS mode set");
    } else {
        ROS_ERROR("Failed to set GUIDED_NOGPS mode");
        return 1;
    }

    // Wait for 4 seconds after arming and setting the mode
    ros::Duration(4.0).sleep();

    ros::Rate loop_rate(hz); // Use hz from configuration
    double dt = 1.0 / hz; // Time step based on hz

    while (ros::ok())
    {
        ros::spinOnce();

        // Compute control commands using PID control
        double pitch_command = pidControl(reference_position.x, current_position.x, Kp_x, Ki_x, Kd_x, integral_x, previous_error_x, dt); // Theta
        double roll_command = pidControl(reference_position.y, current_position.y, Kp_y, Ki_y, Kd_y, integral_y, previous_error_y, dt); // Phi

        // Compute thrust and apply saturation
        double thrust_command = pidControl(reference_position.z, current_position.z, Kp_z, Ki_z, Kd_z, integral_z, previous_error_z, dt); // Thrust
        thrust_command = std::max(std::min(thrust_command, max_thrust), min_thrust); // Clamp thrust_command to min_thrust and max_thrust

        // Debugging information to monitor PID and thrust commands
        ROS_INFO("Reference Position Z: %f", reference_position.z);
        ROS_INFO("Current Position Z: %f", current_position.z);
        ROS_INFO("PID Computed Thrust Command: %f", thrust_command);

        double yaw_command = current_psi; // Directly using psi angle if needed

        // Scale the thrust command to the range [0, 1]
        double normalized_thrust_command = (thrust_command - min_thrust) / (max_thrust - min_thrust); // Normalize thrust to [0, 1]
        
        // Print normalized thrust value
        ROS_INFO("Normalized Thrust Command: %f", normalized_thrust_command);

        // Prepare the attitude target message
        mavros_msgs::AttitudeTarget attitude_msg;
        attitude_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

        tf2::Quaternion q;
        q.setRPY(roll_command, pitch_command, yaw_command); // Convert Euler angles to quaternion

        attitude_msg.orientation.x = q.x();
        attitude_msg.orientation.y = q.y();
        attitude_msg.orientation.z = q.z();
        attitude_msg.orientation.w = q.w();

        // Publish attitude command
        attitude_pub.publish(attitude_msg);

        // Prepare and publish thrust command
        mavros_msgs::Thrust thrust_msg;
        thrust_msg.thrust = normalized_thrust_command; // Scaled thrust in the range [0, 1]
        thrust_pub.publish(thrust_msg);

        loop_rate.sleep();
    }

    return 0;
}