#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <complex>
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
    void generatePsi();

private:
    ros::Publisher m_position_pub;
    ros::Publisher m_velocity_pub;
    ros::Publisher m_acceleration_pub;
    ros::Publisher m_desired_psi_pub;
    ros::ServiceClient m_trajectory_client;
    std::vector<double> m_points_x, m_points_y, m_points_z, m_times;
    double m_ros_rate;
    double m_desired_psi, m_last_psi;
    std::string m_trajectory_method;
    bool m_no_more_trajectory;
    bool m_is_psi_active;
    geometry_msgs::Vector3 m_position;
    geometry_msgs::Vector3 m_velocity;
    geometry_msgs::Vector3 m_acceleration;
    geometry_msgs::Vector3 m_jerk;
    double m_des_psi;
    bool m_initial_trajectory;

};

#endif // TRAJECTORY_GENERATOR_H
