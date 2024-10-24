#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <quadcopter_control/WaypointService.h> 

class TrajectoryGenerator {
public:
    TrajectoryGenerator(ros::NodeHandle& nh);

    bool getTrajectoryFromServer();

    void generateTrajectory();

    void solveNaturalCubicSpline();
    
    void solveCubicSpline();
    
    void solveMinimumJerk();
    
    void solveMinimumSnap();

private:
    ros::Publisher position_pub;
    ros::ServiceClient trajectory_client;
    std::vector<double> points_x, points_y, points_z, times;
    double ros_rate;
    bool return_to_start;
    double return_duration;
    std::string trajectory_method;
    bool no_more_trajectory;
};

#endif // TRAJECTORY_GENERATOR_H
