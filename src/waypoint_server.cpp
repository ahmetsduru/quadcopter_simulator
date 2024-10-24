#include "../include/quadcopter_control/waypoint_server.h"

// Constructor: Initialize the server and load parameters
WaypointServer::WaypointServer(ros::NodeHandle& nh) : current_trajectory_index(0), last_time_offset(0.0) {
    // Create the service server
    service = nh.advertiseService("get_trajectory", &WaypointServer::getTrajectoryCallback, this);

    // Read the active trajectories parameter
    XmlRpc::XmlRpcValue active_trajectories;
    if (nh.getParam("trajectory_manager/active_trajectories", active_trajectories)) {
        ROS_ASSERT(active_trajectories.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int i = 0; i < active_trajectories.size(); ++i) {
            std::string trajectory_name = static_cast<std::string>(active_trajectories[i]);
            loadTrajectory(nh, trajectory_name);  // Load each active trajectory
        }
    } else {
        ROS_ERROR("Failed to get active_trajectories from trajectory_manager");
    }

    // Load return_to_start and return_duration parameters
    nh.param("trajectory_manager/return_to_start", return_to_start, false);
    nh.param("trajectory_manager/return_duration", return_duration, 0.0);
}

// Callback function for the service
bool WaypointServer::getTrajectoryCallback(quadcopter_control::WaypointService::Request &req,
                                           quadcopter_control::WaypointService::Response &res) {
    if (current_trajectory_index >= trajectories.size()) {
        res.points_x.clear();
        res.points_y.clear();
        res.points_z.clear();
        res.times.clear();
        ROS_INFO("All trajectories completed. Sending empty trajectory points and times.");
        return true;
    }

    const auto& traj = trajectories[current_trajectory_index];

    // Send the next trajectory data in the response
    res.method = traj.method;
    res.ros_rate = traj.ros_rate;
    res.return_to_start = return_to_start;
    res.return_duration = return_duration;
    res.points_x = traj.points_x;
    res.points_y = traj.points_y;
    res.points_z = traj.points_z;

    // Update times by adding last_time_offset
    res.times.clear();
    for (const auto& time : traj.times) {
        res.times.push_back(time + last_time_offset);
    }

    last_time_offset = res.times.back();
    current_trajectory_index++;

    ROS_INFO("Sent trajectory data for trajectory: %d", current_trajectory_index);
    return true;
}

// Function to load a trajectory from the parameter server
void WaypointServer::loadTrajectory(ros::NodeHandle& nh, const std::string& trajectory_namespace) {
    TrajectoryData traj;
    nh.getParam("/" + trajectory_namespace + "/points_x", traj.points_x);
    nh.getParam("/" + trajectory_namespace + "/points_y", traj.points_y);
    nh.getParam("/" + trajectory_namespace + "/points_z", traj.points_z);
    nh.getParam("/" + trajectory_namespace + "/times", traj.times);
    nh.getParam("/" + trajectory_namespace + "/ros_rate", traj.ros_rate);
    nh.getParam("/" + trajectory_namespace + "/method", traj.method);

    trajectories.push_back(traj);

    ROS_INFO("Loaded trajectory: %s", trajectory_namespace.c_str());
}

