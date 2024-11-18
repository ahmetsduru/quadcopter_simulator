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
        std::vector<double> m_points_x;
        std::vector<double> m_points_y;
        std::vector<double> m_points_z;
        std::vector<double> m_times;
        double m_ros_rate;
        std::string m_method;
    };

    ros::ServiceServer m_service;             // ROS service server
    std::vector<TrajectoryData> m_trajectories; // List of loaded trajectories
    size_t m_current_trajectory_index = 0;     // Index to track the current trajectory
    double m_last_time_offset = 0.0;           // Time offset from the last trajectory

    XmlRpc::XmlRpcValue m_active_trajectories; // Added as member variable
    TrajectoryData m_traj;                     // Temporary variable for each trajectory
};

#endif // WAYPOINT_SERVER_H
