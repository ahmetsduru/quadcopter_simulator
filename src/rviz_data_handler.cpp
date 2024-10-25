#include "../include/quadcopter_control/rviz_data_handler.h"
#include <std_msgs/ColorRGBA.h>

RVizDataHandler::RVizDataHandler(ros::NodeHandle& nh)
{
    // Initialize publishers
    actual_path_pub = nh.advertise<nav_msgs::Path>("/rviz_quadcopter_path", 10);
    reference_path_pub = nh.advertise<nav_msgs::Path>("/rviz_reference_path", 10);
    actual_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/rviz_actual_pose_array", 10);
    reference_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/rviz_reference_pose_array", 10);
    impulse_force_marker_pub = nh.advertise<visualization_msgs::Marker>("/rviz_impulse_force_marker", 10);
    impulse_torque_marker_pub = nh.advertise<visualization_msgs::Marker>("/rviz_impulse_torque_marker", 10);

    // Initialize subscribers
    actual_pose_sub = nh.subscribe("/rviz_quad_pose", 10, &RVizDataHandler::actualPoseCallback, this);
    reference_angles_sub = nh.subscribe("/actual_euler_angles", 10, &RVizDataHandler::referenceAnglesCallback, this);
    reference_position_sub = nh.subscribe("/reference_position", 10, &RVizDataHandler::referencePositionCallback, this);
    impulse_force_sub = nh.subscribe("/actual_impulse_force", 10, &RVizDataHandler::impulseForceCallback, this);
    impulse_torque_sub = nh.subscribe("/actual_impulse_torque", 10, &RVizDataHandler::impulseTorqueCallback, this);

    last_actual_pose_publish_time = ros::Time::now();
    last_reference_pose_publish_time = ros::Time::now();
    actual_pose_array.header.frame_id = "world";
    reference_pose_array.header.frame_id = "world";
    visualization_threshold = 0.0001;
    pose_publish_interval = ros::Duration(1.0);
    impulse_force_marker_id = 0;
    impulse_torque_marker_id = 0;
}

void RVizDataHandler::actualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    actual_path_msg.header.frame_id = "world";
    actual_path_msg.header.stamp = ros::Time::now();
    actual_position = pose_msg->pose.position;
    actual_path_msg.poses.push_back(*pose_msg);

    if (ros::Time::now() - last_actual_pose_publish_time >= pose_publish_interval)
    {
        actual_pose_array.poses.push_back(pose_msg->pose);
        last_actual_pose_publish_time = ros::Time::now();
        actual_pose_array_pub.publish(actual_pose_array);
    }

    actual_path_pub.publish(actual_path_msg);
}

void RVizDataHandler::referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& ref_msg)
{
    geometry_msgs::PoseStamped ref_pose;
    reference_path_msg.header.frame_id = "world";
    reference_path_msg.header.stamp = ros::Time::now();
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.header.frame_id = "world";
    ref_pose.pose.position = reference_position;
    ref_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ref_msg->x, ref_msg->y, ref_msg->z);
    reference_path_msg.poses.push_back(ref_pose);

    if (ros::Time::now() - last_reference_pose_publish_time >= pose_publish_interval)
    {
        reference_pose_array.poses.push_back(ref_pose.pose);
        last_reference_pose_publish_time = ros::Time::now();
        reference_pose_array_pub.publish(reference_pose_array);
    }

    reference_path_pub.publish(reference_path_msg);
}

void RVizDataHandler::referencePositionCallback(const geometry_msgs::Point::ConstPtr& pos_msg)
{
    reference_position = *pos_msg;
}

void RVizDataHandler::impulseForceCallback(const geometry_msgs::Vector3::ConstPtr& force_msg)
{
    latest_impulse_force = *force_msg;
    if (std::fabs(force_msg->x) > visualization_threshold || std::fabs(force_msg->y) > visualization_threshold || std::fabs(force_msg->z) > visualization_threshold)
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 1.0;
        visualizeVector(latest_impulse_force, actual_position, impulse_force_marker_pub, "impulse_force", impulse_force_marker_id, color);
    }
}

void RVizDataHandler::impulseTorqueCallback(const geometry_msgs::Vector3::ConstPtr& torque_msg)
{
    latest_impulse_torque = *torque_msg;
    if (std::fabs(torque_msg->x) > visualization_threshold || std::fabs(torque_msg->y) > visualization_threshold || std::fabs(torque_msg->z) > visualization_threshold)
    {
        std_msgs::ColorRGBA color;
        color.r = 0.8; color.g = 0.3; color.b = 1.0; color.a = 1.0;
        visualizeVector(latest_impulse_torque, actual_position, impulse_torque_marker_pub, "impulse_torque", impulse_torque_marker_id, color);
    }
}

void RVizDataHandler::visualizeVector(const geometry_msgs::Vector3& vector, const geometry_msgs::Point& position, ros::Publisher& marker_pub, const std::string& ns, int& id, const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);
    marker.points[0] = position;
    marker.points[1].x = position.x + vector.x;
    marker.points[1].y = position.y + vector.y;
    marker.points[1].z = position.z + vector.z;
    marker.scale.x = 0.01;
    marker.scale.y = 0.05;
    marker.scale.z = 0.03;
    marker.color = color;

    marker_pub.publish(marker);
}
