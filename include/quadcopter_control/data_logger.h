#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <string>

class DataLogger
{
public:
    DataLogger();
    ~DataLogger();
    void spin();

private:
    ros::NodeHandle nh_;

    ros::Subscriber ref_angles_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber rviz_quad_pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber angular_velocity_sub_;
    ros::Subscriber acceleration_sub_;

    geometry_msgs::Vector3 ref_angles_data_;
    geometry_msgs::Point position_data_;
    geometry_msgs::Pose rviz_quad_pose_data_;
    geometry_msgs::Vector3 velocity_data_;
    geometry_msgs::Vector3 angular_velocity_data_;
    geometry_msgs::Vector3 acceleration_data_;

    bool ref_angles_received_;
    bool position_received_;
    bool rviz_quad_pose_received_;
    bool velocity_received_;
    bool angular_velocity_received_;
    bool acceleration_received_;

    std::ofstream log_file_ref_pose_;
    std::ofstream log_file_states_;

    void refAnglesCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void positionCallback(const geometry_msgs::Point::ConstPtr& msg);
    void rvizQuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void angularVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void accelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    void logRefPoseData(const ros::Time& timestamp);
    void logStatesData(const ros::Time& timestamp);
};

#endif // DATA_LOGGER_H
