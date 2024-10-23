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

// Class to handle all RViz related data
class RVizDataHandler
{
public:
    RVizDataHandler(ros::NodeHandle& nh)
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

        // Set initial timestamps for pose publishing
        last_actual_pose_publish_time = ros::Time::now();
        last_reference_pose_publish_time = ros::Time::now();
        
        // Initialize PoseArray headers
        actual_pose_array.header.frame_id = "world";
        reference_pose_array.header.frame_id = "world";

        // Set thresholds
        visualization_threshold = 0.0001;
        pose_publish_interval = ros::Duration(1.0);  // 1 second interval
    }

    // Markerları sürekli olarak yayınlamak için bir döngü fonksiyonu
    void spin()
    {
        ros::Rate rate(500);  
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

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
    int impulse_force_marker_id = 0;  // Marker ID for impulse force
    int impulse_torque_marker_id = 0;  // Marker ID for impulse torque

    // Diğer callback fonksiyonları
    void actualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
    {
        actual_path_msg.header.frame_id = "world";
        actual_path_msg.header.stamp = ros::Time::now();

        actual_position = pose_msg->pose.position;
        actual_path_msg.poses.push_back(*pose_msg);

        geometry_msgs::Pose actual_pose = pose_msg->pose;

        if (ros::Time::now() - last_actual_pose_publish_time >= pose_publish_interval)
        {
            actual_pose_array.poses.push_back(actual_pose);
            last_actual_pose_publish_time = ros::Time::now();
            actual_pose_array_pub.publish(actual_pose_array);
        }

        actual_path_pub.publish(actual_path_msg);
    }

    void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& ref_msg)
    {
        geometry_msgs::PoseStamped ref_pose;
        geometry_msgs::Pose pose_array_element;

        reference_path_msg.header.frame_id = "world";
        reference_path_msg.header.stamp = ros::Time::now();

        ref_pose.header.stamp = ros::Time::now();
        ref_pose.header.frame_id = "world";
        ref_pose.pose.position = reference_position;

        double roll = ref_msg->x;
        double pitch = ref_msg->y;
        double yaw = ref_msg->z;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        ref_pose.pose.orientation = quat;

        reference_path_msg.poses.push_back(ref_pose);

        pose_array_element.position = reference_position;
        pose_array_element.orientation = quat;

        if (ros::Time::now() - last_reference_pose_publish_time >= pose_publish_interval)
        {
            reference_pose_array.poses.push_back(pose_array_element);
            last_reference_pose_publish_time = ros::Time::now();
            reference_pose_array_pub.publish(reference_pose_array);
        }

        reference_path_pub.publish(reference_path_msg);
    }

    void referencePositionCallback(const geometry_msgs::Point::ConstPtr& pos_msg)
    {
        reference_position = *pos_msg;
    }

    void impulseForceCallback(const geometry_msgs::Vector3::ConstPtr& force_msg)
    {
        latest_impulse_force = *force_msg;

        if (std::fabs(force_msg->x) > visualization_threshold ||
            std::fabs(force_msg->y) > visualization_threshold ||
            std::fabs(force_msg->z) > visualization_threshold)
        {
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;

            visualizeVector(latest_impulse_force, actual_position, impulse_force_marker_pub, "impulse_force", impulse_force_marker_id, color);
        }
    }

    void impulseTorqueCallback(const geometry_msgs::Vector3::ConstPtr& torque_msg)
    {
        latest_impulse_torque = *torque_msg;

        if (std::fabs(torque_msg->x) > visualization_threshold ||
            std::fabs(torque_msg->y) > visualization_threshold ||
            std::fabs(torque_msg->z) > visualization_threshold)
        {
            std_msgs::ColorRGBA color;
            color.r = 0.8;
            color.g = 0.3;
            color.b = 1.0;
            color.a = 1.0;

            visualizeVector(latest_impulse_torque, actual_position, impulse_torque_marker_pub, "impulse_torque", impulse_torque_marker_id, color);
        }
    }

    // Function to visualize vectors
    void visualizeVector(const geometry_msgs::Vector3& vector, const geometry_msgs::Point& position, ros::Publisher& marker_pub, const std::string& ns, int& id, const std_msgs::ColorRGBA& color)
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
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "rviz_data");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Create an instance of the RVizDataHandler class
    RVizDataHandler rviz_data_handler(nh);

    // Spin to process callbacks
    rviz_data_handler.spin();  // Marker'ları sürekli olarak yayınlamak için döngü

    return 0;
}
