#ifndef WAYPOINT_SERVER_H
#define WAYPOINT_SERVER_H

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <quadcopter_control/WaypointService.h>  // Include the service header

class WaypointServer {
public:
    // Constructor
    WaypointServer(ros::NodeHandle& nh);

    // Service callback function
    bool getTrajectoryCallback(quadcopter_control::WaypointService::Request &req,
                               quadcopter_control::WaypointService::Response &res);

    // Load trajectory data
    void loadTrajectory(ros::NodeHandle& nh, const std::string& trajectory_namespace);

private:
    // Struct to store trajectory data
    struct TrajectoryData {
        std::vector<double> points_x;
        std::vector<double> points_y;
        std::vector<double> points_z;
        std::vector<double> times;
        double ros_rate;
        std::string method;
    };

    ros::ServiceServer service;  // ROS service server
    std::vector<TrajectoryData> trajectories;  // List of loaded trajectories
    size_t current_trajectory_index;  // Index to track the current trajectory
    double last_time_offset;  // Time offset from the last trajectory
    bool return_to_start;  // Indicates if the trajectory should return to start
    double return_duration;  // Duration for returning to start
};

#endif // WAYPOINT_SERVER_H
