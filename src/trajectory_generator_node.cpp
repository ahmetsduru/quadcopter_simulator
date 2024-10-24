#include "../include/quadcopter_control/trajectory_generator.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "trajectory_generator_node");
    
    // Create a NodeHandle object, which is the main access point to communication with the ROS system
    ros::NodeHandle nh;

    // Create an instance of the TrajectoryGenerator class, passing the NodeHandle
    TrajectoryGenerator traj_gen(nh);

    // Call the generateTrajectory function to start the trajectory generation process
    traj_gen.generateTrajectory();

    // ROS spin will keep the node running and allow callbacks to be processed
    ros::spin();

    return 0;
}
