#ifndef RVIZ_DATA_HANDLER_H
#define RVIZ_DATA_HANDLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class RVizDataHandler
{
public:
    RVizDataHandler(ros::NodeHandle& nh);

private:
    // Publishers
    ros::Publisher actual_path_pub, reference_path_pub;
    ros::Publisher actual_pose_array_pub, reference_pose_array_pub;
    ros::Publisher impulse_force_marker_pub, impulse_torque_marker_pub;

    // Subscribers
    ros::Subscriber actual_pose_sub, reference_angles_sub;
    ros::Subscriber reference_position_sub, impulse_force_sub;
    ros::Subscriber impulse_torque_sub;

    // Global variables for paths and poses
    nav_msgs::Path actual_path_msg, reference_path_msg;
    geometry_msgs::PoseArray actual_pose_array, reference_pose_array;
    geometry_msgs::Point reference_position, actual_position;

    // Global variables for impulse force and torque
    geometry_msgs::Vector3 latest_impulse_force, latest_impulse_torque;

    // Time management for pose publishing
    ros::Time last_actual_pose_publish_time, last_reference_pose_publish_time;
    ros::Duration pose_publish_interval;

    // Threshold for visualization
    double visualization_threshold;

    // Marker IDs for impulse force and torque
    int impulse_force_marker_id;
    int impulse_torque_marker_id;

    // Callback functions
    void actualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& ref_msg);
    void referencePositionCallback(const geometry_msgs::Point::ConstPtr& pos_msg);
    void impulseForceCallback(const geometry_msgs::Vector3::ConstPtr& force_msg);
    void impulseTorqueCallback(const geometry_msgs::Vector3::ConstPtr& torque_msg);

    // Function to visualize vectors
    void visualizeVector(const geometry_msgs::Vector3& vector, const geometry_msgs::Point& position, ros::Publisher& marker_pub, const std::string& ns, int& id, const std_msgs::ColorRGBA& color);
};

#endif // RVIZ_DATA_HANDLER_H
