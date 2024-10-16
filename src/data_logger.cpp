#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <string>

// Global variables to store the most recent values from the subscribers
geometry_msgs::Vector3 ref_angles_data;
geometry_msgs::Point position_data;
geometry_msgs::Pose rviz_quad_pose_data;
geometry_msgs::Vector3 velocity_data;
geometry_msgs::Vector3 angular_velocity_data;
geometry_msgs::Vector3 acceleration_data;

bool ref_angles_received = false;
bool position_received = false;
bool rviz_quad_pose_received = false;
bool velocity_received = false;
bool angular_velocity_received = false;
bool acceleration_received = false;

std::ofstream log_file_ref_pose;
std::ofstream log_file_states;

// Callback function for reference angles
void refAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_angles_data = *msg;
    ref_angles_received = true;
}

// Callback function for position
void positionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    position_data = *msg;
    position_received = true;
}

// Callback function for rviz quad pose (PoseStamped)
void rvizQuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    rviz_quad_pose_data = msg->pose;  // Extracting pose from PoseStamped
    rviz_quad_pose_received = true;
}

// Callback function for velocity
void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    velocity_data = *msg;
    velocity_received = true;
}

// Callback function for angular velocity
void angularVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    angular_velocity_data = *msg;
    angular_velocity_received = true;
}

// Callback function for acceleration
void accelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    acceleration_data = *msg;
    acceleration_received = true;
}

void logRefPoseData(const ros::Time& timestamp)
{
    if (log_file_ref_pose.is_open())
    {
        // Convert Euler angles (ref_angles_data) to quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(ref_angles_data.x, ref_angles_data.y, ref_angles_data.z); // Roll, Pitch, Yaw to Quaternion

        // Log position and quaternion data
        log_file_ref_pose << timestamp << ", " 
                          << position_data.x << ", " << position_data.y << ", " << position_data.z << ", "
                          << quaternion.x() << ", " << quaternion.y() << ", " << quaternion.z() << ", " << quaternion.w() << std::endl;
    }
}

void logStatesData(const ros::Time& timestamp)
{
    if (log_file_states.is_open())
    {
        log_file_states << timestamp << ", "
                        << rviz_quad_pose_data.position.x << ", " << rviz_quad_pose_data.position.y << ", " << rviz_quad_pose_data.position.z << ", "
                        << velocity_data.x << ", " << velocity_data.y << ", " << velocity_data.z << ", "
                        << acceleration_data.x << ", " << acceleration_data.y << ", " << acceleration_data.z << ", "
                        << rviz_quad_pose_data.orientation.x << ", " << rviz_quad_pose_data.orientation.y << ", " << rviz_quad_pose_data.orientation.z << ", " << rviz_quad_pose_data.orientation.w << ", "
                        << angular_velocity_data.x << ", " << angular_velocity_data.y << ", " << angular_velocity_data.z << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_logger");
    ros::NodeHandle nh;

    log_file_ref_pose.open("/home/asd/catkin_ws/src/quadcopter_control/log/ref_pose.txt", std::ios::out);
    log_file_states.open("/home/asd/catkin_ws/src/quadcopter_control/log/states.txt", std::ios::out);
    
    if (!log_file_ref_pose.is_open() || !log_file_states.is_open())
    {
        ROS_ERROR("Failed to open log files");
        return 1;
    }

    // Write header to ref_pose file
    log_file_ref_pose << "timestamp, ref_x, ref_y, ref_z, ref_quat_x, ref_quat_y, ref_quat_z, ref_quat_w" << std::endl;
    
    // Write header to states file
    log_file_states << "timestamp, x, y, z, "
                    << "vel_x, vel_y, vel_z, "
                    << "acc_x, acc_y, acc_z, "
                    << "orientation_x, orientation_y, orientation_z, orientation_w, "
                    << "ang_vel_x, ang_vel_y, ang_vel_z" << std::endl;


    ros::Subscriber ref_angles_sub = nh.subscribe("/control/reference_angles", 10, refAnglesCallback);
    ros::Subscriber position_sub = nh.subscribe("/position", 10, positionCallback);
    ros::Subscriber rviz_quad_pose_sub = nh.subscribe("/rviz_quad_pose", 10, rvizQuadPoseCallback); // PoseStamped subscriber
    ros::Subscriber velocity_sub = nh.subscribe("/quadcopter/velocity", 10, velocityCallback);
    ros::Subscriber angular_velocity_sub = nh.subscribe("/quadcopter/angular_velocity", 10, angularVelocityCallback);
    ros::Subscriber acceleration_sub = nh.subscribe("/quadcopter/acceleration", 10, accelerationCallback);

    // Set loop rate to 500 Hz or any frequency you need
    ros::Rate rate(500); // 500 Hz

    while (ros::ok())
    {
        // Call any pending callbacks
        ros::spinOnce();

        // Log ref_pose data at a fixed rate
        if (ref_angles_received && position_received)
        {
            logRefPoseData(ros::Time::now()); // Log data with timestamp
        }

        // Log states data at a fixed rate
        if (rviz_quad_pose_received && velocity_received && angular_velocity_received && acceleration_received)
        {
            logStatesData(ros::Time::now()); // Log data with timestamp
        }

        rate.sleep();
    }

    log_file_ref_pose.close();
    log_file_states.close();

    return 0;
}
