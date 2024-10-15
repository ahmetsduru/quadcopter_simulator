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

// Global variables for path messages and publishers
nav_msgs::Path actual_path_msg;
nav_msgs::Path reference_path_msg;
ros::Publisher actual_path_pub;
ros::Publisher reference_path_pub;
ros::Publisher actual_pose_array_pub; // Publisher for the actual pose array
ros::Publisher reference_pose_array_pub; // Publisher for the reference pose array
ros::Publisher impulse_force_marker_pub;  // Publisher for impulse force marker
ros::Publisher impulse_torque_marker_pub; // Publisher for impulse torque marker
ros::Publisher waypoint_marker_pub; // Publisher for waypoint markers

geometry_msgs::Point reference_position;  // Global variable to hold the reference position
geometry_msgs::PoseArray actual_pose_array; // Global PoseArray to hold actual poses
geometry_msgs::PoseArray reference_pose_array; // Global PoseArray to hold reference poses
geometry_msgs::Point actual_position;  // Global variable for actual position

ros::Time last_actual_pose_publish_time;
ros::Time last_reference_pose_publish_time;
ros::Duration pose_publish_interval(1.0); // 0.01 second interval

// Variables to hold the latest impulse force and torque
geometry_msgs::Vector3 latest_impulse_force;
geometry_msgs::Vector3 latest_impulse_torque;

// Threshold to visualize only non-zero values
const double visualization_threshold = 0.0001;

// Vectors to store all the markers
std::vector<visualization_msgs::Marker> impulse_force_markers;
std::vector<visualization_msgs::Marker> impulse_torque_markers;

// Global variables to store the last marker IDs (to generate unique IDs)
int impulse_force_marker_id = 0;
int impulse_torque_marker_id = 0;
int waypoint_marker_id = 0; // Unique marker ID for waypoints

// Callback function for the actual PoseStamped messages
void actualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    // Set the header frame ID and timestamp for the actual path message
    actual_path_msg.header.frame_id = "world";  // Set to the correct frame
    actual_path_msg.header.stamp = ros::Time::now();

    // Update the current position for visualizing impulses
    actual_position = pose_msg->pose.position;

    // Add the new pose to the actual path
    actual_path_msg.poses.push_back(*pose_msg);

    // Convert PoseStamped to Pose for PoseArray
    geometry_msgs::Pose actual_pose = pose_msg->pose;

    // Add the pose to the actual PoseArray, but only if 1 second has passed since the last publish
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_actual_pose_publish_time) >= pose_publish_interval)
    {
        actual_pose_array.poses.push_back(actual_pose);
        last_actual_pose_publish_time = current_time;

        // Publish the actual PoseArray
        actual_pose_array_pub.publish(actual_pose_array);
    }

    // Publish the actual path
    actual_path_pub.publish(actual_path_msg);
}

// Callback function for the reference Euler angles (geometry_msgs::Vector3)
void referenceAnglesCallback(const geometry_msgs::Vector3::ConstPtr& ref_msg)
{
    geometry_msgs::PoseStamped ref_pose;
    geometry_msgs::Pose pose_array_element;

    // Set the header frame ID and timestamp for the reference path message
    reference_path_msg.header.frame_id = "world";  // Set to the correct frame
    reference_path_msg.header.stamp = ros::Time::now();

    // Set reference position from the global variable for Path
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.header.frame_id = "world";  // Set to the correct frame
    ref_pose.pose.position = reference_position;  // Use the position obtained from /position topic

    // Set Euler angles (roll, pitch, yaw) from the Vector3 message
    double roll = ref_msg->x;    // Assuming x is roll (phi)
    double pitch = ref_msg->y;   // Assuming y is pitch (theta)
    double yaw = ref_msg->z;     // Assuming z is yaw (psi)

    // Convert Euler angles to quaternion for orientation
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    ref_pose.pose.orientation = quat;

    // Add the reference pose to the reference path
    reference_path_msg.poses.push_back(ref_pose);

    // For PoseArray, we just use the pose (without header)
    pose_array_element.position = reference_position;
    pose_array_element.orientation = quat;

    // Add the pose to the PoseArray, but only if 1 second has passed since the last publish
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_reference_pose_publish_time) >= pose_publish_interval)
    {
        reference_pose_array.poses.push_back(pose_array_element);
        last_reference_pose_publish_time = current_time;

        // Publish the reference PoseArray
        reference_pose_array_pub.publish(reference_pose_array);
    }

    // Publish the reference path
    reference_path_pub.publish(reference_path_msg);
}

// Callback function for the reference position (geometry_msgs::Point)
void referencePositionCallback(const geometry_msgs::Point::ConstPtr& pos_msg)
{
    // Store the latest reference position in the global variable
    reference_position = *pos_msg;
}

// Function to visualize a vector as an arrow marker in RViz
void visualizeVectorContinuous(const geometry_msgs::Vector3& impulse, const geometry_msgs::Point& position, ros::Publisher& marker_pub, const std::string& ns, int& id, const std_msgs::ColorRGBA& color)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Start point of the arrow (quad position)
    marker.points.resize(2);
    marker.points[0].x = position.x;
    marker.points[0].y = position.y;
    marker.points[0].z = position.z;

    // End point of the arrow (quad position + impulse)
    marker.points[1].x = position.x + impulse.x;
    marker.points[1].y = position.y + impulse.y;
    marker.points[1].z = position.z + impulse.z;

    // Arrow appearance settings
    marker.scale.x = 0.01;  // Shaft diameter
    marker.scale.y = 0.05;  // Head diameter
    marker.scale.z = 0.03;  // Head length

    // Set the color of the arrow
    marker.color = color;

    // Store the marker in the corresponding vector
    if (ns == "impulse_force")
    {
        impulse_force_markers.push_back(marker);
    }
    else if (ns == "impulse_torque")
    {
        impulse_torque_markers.push_back(marker);
    }

    // Publish the marker to RViz
    marker_pub.publish(marker);
}

// Callback function for the impulse force (geometry_msgs::Vector3)
void impulseForceCallback(const geometry_msgs::Vector3::ConstPtr& force_msg)
{
    latest_impulse_force = *force_msg;

    // Check if the force is non-zero
    if (std::fabs(force_msg->x) > visualization_threshold ||
        std::fabs(force_msg->y) > visualization_threshold ||
        std::fabs(force_msg->z) > visualization_threshold)
    {
        // Define the color for the impulse force arrow (e.g., blue)
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;

        // Continuously visualize the impulse force as an arrow in RViz, using the actual quadcopter position
        visualizeVectorContinuous(latest_impulse_force, actual_position, impulse_force_marker_pub, "impulse_force", impulse_force_marker_id, color);
    }
}

// Callback function for the impulse torque (geometry_msgs::Vector3)
void impulseTorqueCallback(const geometry_msgs::Vector3::ConstPtr& torque_msg)
{
    latest_impulse_torque = *torque_msg;

    // Check if the torque is non-zero
    if (std::fabs(torque_msg->x) > visualization_threshold ||
        std::fabs(torque_msg->y) > visualization_threshold ||
        std::fabs(torque_msg->z) > visualization_threshold)
    {
        // Define the color for the impulse torque arrow
        std_msgs::ColorRGBA color;
        color.r = 0.8;
        color.g = 0.3;
        color.b = 1.0;
        color.a = 1.0;

        // Continuously visualize the impulse torque as an arrow in RViz, using the actual quadcopter position
        visualizeVectorContinuous(latest_impulse_torque, actual_position, impulse_torque_marker_pub, "impulse_torque", impulse_torque_marker_id, color);
    }
}

// Callback function for the waypoints (geometry_msgs::Vector3)
void waypointsCallback(const geometry_msgs::Vector3::ConstPtr& waypoint_msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Set the frame for RViz
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = waypoint_marker_id++;  // Unique ID for each waypoint marker
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the position of the marker (from the waypoint data)
    marker.pose.position.x = waypoint_msg->x;
    marker.pose.position.y = waypoint_msg->y;
    marker.pose.position.z = waypoint_msg->z;

    // Marker appearance
    marker.scale.x = 0.1;  // Set the size of the sphere marker
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color (red for waypoints)
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    // Publish the marker to RViz
    waypoint_marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "rviz_data");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Initialize the last publish time to 0
    last_actual_pose_publish_time = ros::Time::now();
    last_reference_pose_publish_time = ros::Time::now();

    // Create a subscriber for the actual quadcopter pose
    ros::Subscriber actual_pose_sub = nh.subscribe("/rviz_quad_pose", 10, actualPoseCallback);

    // Create a subscriber for the reference Euler angles
    ros::Subscriber reference_angles_sub = nh.subscribe("/quadcopter/euler_angles", 10, referenceAnglesCallback);

    // Create a subscriber for the reference position
    ros::Subscriber reference_position_sub = nh.subscribe("/position", 10, referencePositionCallback);

    // Create a subscriber for the impulse force
    ros::Subscriber impulse_force_sub = nh.subscribe("/quadcopter/impulse_force", 10, impulseForceCallback);

    // Create a subscriber for the impulse torque
    ros::Subscriber impulse_torque_sub = nh.subscribe("/quadcopter/impulse_torque", 10, impulseTorqueCallback);

    // Create a subscriber for waypoints
    ros::Subscriber waypoints_sub = nh.subscribe("/waypoints", 10, waypointsCallback);

    // Create a publisher for the actual path
    actual_path_pub = nh.advertise<nav_msgs::Path>("/rviz_quadcopter_path", 10);

    // Create a publisher for the reference path
    reference_path_pub = nh.advertise<nav_msgs::Path>("/rviz_reference_path", 10);

    // Create a publisher for the actual pose array (PoseArray)
    actual_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/rviz_actual_pose_array", 10);

    // Create a publisher for the reference pose array (PoseArray)
    reference_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/rviz_reference_pose_array", 10);

    // Create publishers for impulse force and torque markers
    impulse_force_marker_pub = nh.advertise<visualization_msgs::Marker>("/rviz_impulse_force_marker", 10);
    impulse_torque_marker_pub = nh.advertise<visualization_msgs::Marker>("/rviz_impulse_torque_marker", 10);

    // Create a publisher for waypoint markers
    waypoint_marker_pub = nh.advertise<visualization_msgs::Marker>("/rviz_waypoint_markers", 10);

    // Initialize the PoseArray headers and frames
    actual_pose_array.header.frame_id = "world";  // Set the correct frame
    reference_pose_array.header.frame_id = "world";  // Set the correct frame

    // Spin to process callbacks
    ros::spin();

    return 0;
}
