#include "../include/quadcopter_control/data_logger.h"

DataLogger::DataLogger()
{
    // Initialize subscribers
    ref_angles_sub_ = nh_.subscribe("/reference_euler_angles", 10, &DataLogger::refAnglesCallback, this);
    position_sub_ = nh_.subscribe("/reference_position", 10, &DataLogger::positionCallback, this);
    rviz_quad_pose_sub_ = nh_.subscribe("/rviz_quad_pose", 10, &DataLogger::rvizQuadPoseCallback, this);
    velocity_sub_ = nh_.subscribe("/actual_velocity", 10, &DataLogger::velocityCallback, this);
    angular_velocity_sub_ = nh_.subscribe("/actual_angular_velocity", 10, &DataLogger::angularVelocityCallback, this);
    acceleration_sub_ = nh_.subscribe("/actual_acceleration", 10, &DataLogger::accelerationCallback, this);

    // Open log files
    log_file_ref_pose_.open("/home/asd/catkin_ws/src/quadcopter_control/log/ref_pose.txt", std::ios::out);
    log_file_states_.open("/home/asd/catkin_ws/src/quadcopter_control/log/states.txt", std::ios::out);

    if (!log_file_ref_pose_.is_open() || !log_file_states_.is_open())
    {
        ROS_ERROR("Failed to open log files");
        ros::shutdown();
    }

    // Write headers
    log_file_ref_pose_ << "timestamp, ref_x, ref_y, ref_z, ref_quat_x, ref_quat_y, ref_quat_z, ref_quat_w" << std::endl;
    log_file_states_ << "timestamp, x, y, z, "
                     << "vel_x, vel_y, vel_z, "
                     << "acc_x, acc_y, acc_z, "
                     << "orientation_x, orientation_y, orientation_z, orientation_w, "
                     << "ang_vel_x, ang_vel_y, ang_vel_z" << std::endl;

    // Initialize received flags
    ref_angles_received_ = false;
    position_received_ = false;
    rviz_quad_pose_received_ = false;
    velocity_received_ = false;
    angular_velocity_received_ = false;
    acceleration_received_ = false;
}

DataLogger::~DataLogger()
{
    log_file_ref_pose_.close();
    log_file_states_.close();
}

void DataLogger::spin()
{
    ros::Rate rate(500); // 500 Hz

    while (ros::ok())
    {
        // Process callbacks
        ros::spinOnce();

        // Log ref_pose data
        if (ref_angles_received_ && position_received_)
        {
            logRefPoseData(ros::Time::now());
        }

        // Log states data
        if (rviz_quad_pose_received_ && velocity_received_ && angular_velocity_received_ && acceleration_received_)
        {
            logStatesData(ros::Time::now());
        }

        rate.sleep();
    }
}

void DataLogger::refAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_angles_data_ = *msg;
    ref_angles_received_ = true;
}

void DataLogger::positionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    position_data_ = *msg;
    position_received_ = true;
}

void DataLogger::rvizQuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    rviz_quad_pose_data_ = msg->pose;  // Extracting pose from PoseStamped
    rviz_quad_pose_received_ = true;
}

void DataLogger::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    velocity_data_ = *msg;
    velocity_received_ = true;
}

void DataLogger::angularVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    angular_velocity_data_ = *msg;
    angular_velocity_received_ = true;
}

void DataLogger::accelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    acceleration_data_ = *msg;
    acceleration_received_ = true;
}

void DataLogger::logRefPoseData(const ros::Time& timestamp)
{
    if (log_file_ref_pose_.is_open())
    {
        // Convert Euler angles (ref_angles_data_) to quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(ref_angles_data_.x, ref_angles_data_.y, ref_angles_data_.z); // Roll, Pitch, Yaw to Quaternion

        // Log position and quaternion data
        log_file_ref_pose_ << timestamp << ", " 
                          << position_data_.x << ", " << position_data_.y << ", " << position_data_.z << ", "
                          << quaternion.x() << ", " << quaternion.y() << ", " << quaternion.z() << ", " << quaternion.w() << std::endl;
    }
}

void DataLogger::logStatesData(const ros::Time& timestamp)
{
    if (log_file_states_.is_open())
    {
        log_file_states_ << timestamp << ", "
                        << rviz_quad_pose_data_.position.x << ", " << rviz_quad_pose_data_.position.y << ", " << rviz_quad_pose_data_.position.z << ", "
                        << velocity_data_.x << ", " << velocity_data_.y << ", " << velocity_data_.z << ", "
                        << acceleration_data_.x << ", " << acceleration_data_.y << ", " << acceleration_data_.z << ", "
                        << rviz_quad_pose_data_.orientation.x << ", " << rviz_quad_pose_data_.orientation.y << ", " << rviz_quad_pose_data_.orientation.z << ", " << rviz_quad_pose_data_.orientation.w << ", "
                        << angular_velocity_data_.x << ", " << angular_velocity_data_.y << ", " << angular_velocity_data_.z << std::endl;
    }
}
