#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <functional>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Quadcopter parameters
double m;  // Mass of the quadcopter (kg)
double I_rotor;  // Rotor inertia
Eigen::Matrix3d I; // Inertia matrix (3x3) representing the moment of inertia of the quadcopter
Eigen::Vector3d g; // Gravity vector (m/s^2), usually [0, 0, -9.81] for Earth

// Drag-related parameters for translational motion
double rho; // Air density (kg/m^3)
Eigen::Matrix3d Cd_trans; // Drag coefficient matrix for translational forces
Eigen::Matrix3d A_trans;  // Area matrix for translational drag

// Drag-related parameters for rotational motion
double Cd_rot; // Drag coefficient for rotational torques
double A_rotor;   // Rotor area for drag torques
double l;   // Arm length (distance from the center of the quadcopter to each rotor)
double r_propeller;

// Motor dynamics parameters
double tau_thrust;  // Time constant for thrust response
double tau_torque;  // Time constant for torque response
double thrust_actual = 0.0;  // Actual thrust (updated using motor dynamics)
Eigen::Vector3d torques_actual(0.0, 0.0, 0.0);  // Actual torques (updated using motor dynamics)

// Time variables
double t = 0.0;  // Current time
double dt; // Time step for ODE integration

// Variables to hold desired thrust and torques
double thrust = 0.0;  // Desired thrust
Eigen::Vector3d torques(0.0, 0.0, 0.0);  // Desired torques for roll, pitch, and yaw

// Rotor and system dynamics
double kt_coeff; // Thrust coefficient
double km_coeff; // Yaw torque coefficient

// Impulse disturbance parameters
Eigen::Vector3d impulse_force;   // Impulse force
Eigen::Vector3d impulse_torque;  // Impulse torque

// Separate timings for force and torque disturbances
double force_impulse_start_time;       // Start time for force impulse
double force_impulse_duration;         // Duration for force impulse
double torque_impulse_start_time;      // Start time for torque impulse
double torque_impulse_duration;        // Duration for torque impulse

// Callback function to update thrust from the first node
void thrustCallback(const std_msgs::Float64::ConstPtr& msg) {
    thrust = msg->data;
}

// Callback function to update torques from the second node
void torquesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    torques.x() = msg->x;
    torques.y() = msg->y;
    torques.z() = msg->z;
}

// First-order lag for thrust
void updateThrust(double thrust_desired, double& thrust_actual, double tau_thrust, double dt) {
    // Update the actual thrust with a first-order lag
    double dF = (1.0 / tau_thrust) * (thrust_desired - thrust_actual);
    thrust_actual += dF * dt;
    //ROS_INFO("Total Thrust: %.5f", thrust_actual);
}

// First-order lag for torques
void updateTorques(const Eigen::Vector3d& torques_desired, Eigen::Vector3d& torques_actual, double tau_torque, double dt) {
    // Update the actual torques with a first-order lag
    Eigen::Vector3d dTau = (1.0 / tau_torque) * (torques_desired - torques_actual);
    torques_actual += dTau * dt;
}

// Function to compute the skew-symmetric matrix from a vector (used for cross products)
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew;
    skew << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return skew;
}

// Function to compute rotor speeds squared based on desired thrust and torques
void computeRotorSpeeds(double thrust, const Eigen::Vector3d& torques, double kt_coeff, double km_coeff, double l, Eigen::Vector4d& omega_squared) {
    // A matrix relates thrust and torques to rotor speeds squared
    Eigen::Matrix4d A;
    Eigen::Vector4d b;

    double a_term = kt_coeff * l / sqrt(2);
    A << kt_coeff, kt_coeff, kt_coeff, kt_coeff,
         a_term, -a_term, -a_term, a_term,
         -a_term, a_term, -a_term, a_term,
         km_coeff, km_coeff, -km_coeff, -km_coeff;

    b << thrust, torques[0], torques[1], torques[2];
    // Solve for the rotor speeds squared using a QR decomposition
    omega_squared = A.colPivHouseholderQr().solve(b);
}

// Function to get rotor speeds from thrust and torques
void getRotorSpeeds(double thrust, const Eigen::Vector3d& torques, double kt_coeff, double km_coeff, double l, Eigen::Vector4d& omega) {
    Eigen::Vector4d omega_squared;
    computeRotorSpeeds(thrust, torques, kt_coeff, km_coeff, l, omega_squared);
    // Take the square root of each rotor speed squared to get the actual speeds
    omega(0) = sqrt(omega_squared(0));
    omega(1) = sqrt(omega_squared(1));
    omega(2) = sqrt(omega_squared(2));
    omega(3) = sqrt(omega_squared(3));
}

// Function to compute drag forces based on linear velocity (translational motion)
Eigen::Vector3d computeDragForces(const Eigen::Vector3d& V, double rho, const Eigen::Matrix3d& Cd_trans, const Eigen::Matrix3d& A_trans) {
    // Compute drag force using drag equation: F_drag = 0.5 * rho * Cd * A * V^2
    Eigen::Vector3d V_squared = V.array().square();  // Square each velocity component
    Eigen::Vector3d drag_force = -0.5 * rho * (Cd_trans * A_trans * V_squared);  // Compute drag forces
    return drag_force;
}

// Function to compute drag torques based on rotor angular velocities
Eigen::Vector3d computeDragTorques(const Eigen::Vector4d& omega_squared, double rho, double Cd_rot, double r_propeller, double A_rotor, double km_coeff) {
    // Compute drag torques for each rotor based on angular velocity
    double b_term = -0.5 * rho * Cd_rot * A_rotor * std::pow(r_propeller, 2);
    Eigen::Vector4d drag_torques = b_term * omega_squared;

    // Matrix to map rotor torques to overall quadcopter torques
    Eigen::Matrix<double, 3, 4> rotor_matrix;
    double a_term = kt_coeff * l / sqrt(2);

    rotor_matrix << a_term, -a_term, -a_term, a_term,    // Roll torque contributions
                    -a_term, a_term, -a_term, a_term,    // Pitch torque contributions
                    km_coeff, km_coeff, -km_coeff, -km_coeff;  // Yaw torque contributions

    Eigen::Vector3d total_drag_torques = rotor_matrix * drag_torques;
    return total_drag_torques;
}

// Function to compute gyroscopic torque based on rotor speeds and quadcopter body angular velocity
Eigen::Vector3d computeGyroscopicTorque(const Eigen::Vector3d& omega_body, const Eigen::Vector4d& rotor_speeds, double I_rotor) {
    // Formula: Gyroscopic Torque = I_rotor * (omega_body x omega_rotor)
    // Define rotor directions (-1 for CW, 1 for CCW)
    Eigen::Vector4d rotor_directions;
    rotor_directions << -1.0, -1.0, 1.0, 1.0;  // Rotors 1 and 2 are CW, 3 and 4 are CCW
    
    // Initialize the total gyroscopic torque vector to zero
    Eigen::Vector3d gyro_torque = Eigen::Vector3d::Zero();
    
    for (int i = 0; i < 4; ++i) {
        // Rotor angular velocity vector (rotors spin along the z-axis)
        Eigen::Vector3d rotor_angular_velocity(0, 0, rotor_speeds(i) * rotor_directions(i));  // Rotor angular velocity along z-axis
        // Cross product of body angular velocity and rotor angular velocity
        gyro_torque += I_rotor * omega_body.cross(rotor_angular_velocity);  // Gyroscopic torque
    }
    return gyro_torque;
}

// Function to compute impulse force
Eigen::Vector3d applyImpulseForce(double current_time) {
    if (current_time >= force_impulse_start_time && current_time <= force_impulse_start_time + force_impulse_duration) {
        return impulse_force;
    }
    return Eigen::Vector3d::Zero();  // No impulse outside the specified duration
}

// Function to compute impulse torque
Eigen::Vector3d applyImpulseTorque(double current_time) {
    if (current_time >= torque_impulse_start_time && current_time <= torque_impulse_start_time + torque_impulse_duration) {
        return impulse_torque;
    }
    return Eigen::Vector3d::Zero();  // No impulse outside the specified duration
}

void computeStateDerivatives(const std::vector<double>& x, std::vector<double>& dxdt, double t, const Eigen::Vector3d& thrust_vector, const Eigen::Vector3d& torques, const Eigen::Matrix3d& I, double m, const Eigen::Vector3d& g, double rho, const Eigen::Matrix3d& Cd_trans, const Eigen::Matrix3d& A_trans, double Cd_rot, double A_rotor, double l, const Eigen::Vector4d& rotor_speeds, double I_rotor) {
    // State variables: position, rotation matrix, velocity, angular velocity
    Eigen::Vector3d r(x[0], x[1], x[2]);  // Position
    Eigen::Matrix3d R;  // Rotation matrix
    R << x[3], x[4], x[5],
         x[6], x[7], x[8],
         x[9], x[10], x[11];
    Eigen::Vector3d V(x[12], x[13], x[14]);  // Velocity
    Eigen::Vector3d w(x[15], x[16], x[17]);  // Angular velocity

    // Position derivative (velocity)
    dxdt[0] = V[0];
    dxdt[1] = V[1];
    dxdt[2] = V[2];

    // Rotation matrix derivative
    Eigen::Matrix3d skew_w = skewSymmetric(w);
    Eigen::Matrix3d dR = skew_w * R;
    dxdt[3] = dR(0, 0); dxdt[4] = dR(0, 1); dxdt[5] = dR(0, 2);
    dxdt[6] = dR(1, 0); dxdt[7] = dR(1, 1); dxdt[8] = dR(1, 2);
    dxdt[9] = dR(2, 0); dxdt[10] = dR(2, 1); dxdt[11] = dR(2, 2);

    // Compute forces acting on the quadcopter
    Eigen::Vector3d drag_forces = computeDragForces(V, rho, Cd_trans, A_trans);
    Eigen::Vector3d impulse_force = applyImpulseForce(t);  // Impulse force applied
    Eigen::Vector3d dV = (R * thrust_vector + drag_forces + impulse_force + m * g) / m;
    dxdt[12] = dV[0];
    dxdt[13] = dV[1];
    dxdt[14] = dV[2];

    // Compute angular acceleration (torque dynamics)
    Eigen::Vector3d drag_torques = computeDragTorques(rotor_speeds, rho, Cd_rot, l, A_rotor, km_coeff);
    Eigen::Vector3d gyro_torques = computeGyroscopicTorque(w, rotor_speeds, I_rotor);  // Gyroscopic torque
    Eigen::Vector3d impulse_torque = applyImpulseTorque(t);  // Impulse torque applied

    Eigen::Vector3d wxIw = skew_w * (I * w);
    Eigen::Vector3d dw = I.inverse() * (torques + drag_torques + gyro_torques + impulse_torque - wxIw);
    dxdt[15] = dw[0];
    dxdt[16] = dw[1];
    dxdt[17] = dw[2];
}

// Perform ODE integration with motor dynamics and compute the state derivatives
void performODEIntegration(std::vector<double>& x, const Eigen::Vector3d& torques_desired, double thrust_desired, const Eigen::Matrix3d& I, double m, const Eigen::Vector3d& g, double rho, const Eigen::Matrix3d& Cd_trans, const Eigen::Matrix3d& A_trans, double Cd_rot, double A_rotor, double l, double I_rotor) {
    using namespace boost::numeric::odeint;
    typedef runge_kutta_cash_karp54<std::vector<double>> error_stepper_type;
    typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
    controlled_stepper_type controlled_stepper;

    // Update thrust and torques using first-order dynamics
    updateThrust(thrust_desired, thrust_actual, tau_thrust, dt);
    updateTorques(torques_desired, torques_actual, tau_torque, dt);

    // Compute rotor speeds
    Eigen::Vector4d rotor_speeds;
    getRotorSpeeds(thrust_actual, torques_actual, kt_coeff, km_coeff, l, rotor_speeds);

    Eigen::Vector3d thrust_vector(0, 0, thrust_actual);

    // Integrate the state derivatives using adaptive Runge-Kutta
    integrate_adaptive(
        controlled_stepper,
        [&](const std::vector<double>& x, std::vector<double>& dxdt, double t) {
            computeStateDerivatives(x, dxdt, t, thrust_vector, torques_actual, I, m, g, rho, Cd_trans, A_trans, Cd_rot, A_rotor, l, rotor_speeds, I_rotor);
        },
        x, t, t + dt, dt
    );

    // Re-orthogonalize the rotation matrix to avoid numerical errors
    Eigen::Matrix3d R;
    R << x[3], x[4], x[5],
         x[6], x[7], x[8],
         x[9], x[10], x[11];
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    x[3] = R(0, 0); x[4] = R(0, 1); x[5] = R(0, 2);
    x[6] = R(1, 0); x[7] = R(1, 1); x[8] = R(1, 2);
    x[9] = R(2, 0); x[10] = R(2, 1); x[11] = R(2, 2);

    // Increment the time
    t += dt;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "eom");
    ros::NodeHandle nh;

    // Retrieve quadcopter parameters from ROS parameter server
    std::vector<double> inertia_values, gravity_values, drag_coefficients_trans, A_frame, impulse_force_vec, impulse_torque_vec;
    nh.getParam("state_derivative_solver_node/m", m);
    nh.getParam("state_derivative_solver_node/I", inertia_values);
    nh.getParam("state_derivative_solver_node/g", gravity_values);
    nh.getParam("state_derivative_solver_node/dt", dt);
    nh.getParam("state_derivative_solver_node/tau_thrust", tau_thrust);
    nh.getParam("state_derivative_solver_node/tau_torque", tau_torque);
    nh.getParam("state_derivative_solver_node/rho", rho);
    nh.getParam("state_derivative_solver_node/A_rotor", A_rotor);
    nh.getParam("state_derivative_solver_node/A_trans", A_frame);
    nh.getParam("state_derivative_solver_node/r_propeller", r_propeller);
    nh.getParam("state_derivative_solver_node/Cd_rot", Cd_rot); 
    nh.getParam("state_derivative_solver_node/Cd_trans", drag_coefficients_trans);
    nh.getParam("state_derivative_solver_node/kt_coeff", kt_coeff);
    nh.getParam("state_derivative_solver_node/km_coeff", km_coeff);
    nh.getParam("state_derivative_solver_node/I_rotor", I_rotor);
    nh.getParam("state_derivative_solver_node/l", l);
    nh.getParam("state_derivative_solver_node/impulse_force", impulse_force_vec);
    nh.getParam("state_derivative_solver_node/impulse_torque", impulse_torque_vec);
    nh.getParam("state_derivative_solver_node/force_impulse_start_time", force_impulse_start_time);
    nh.getParam("state_derivative_solver_node/force_impulse_duration", force_impulse_duration);
    nh.getParam("state_derivative_solver_node/torque_impulse_start_time", torque_impulse_start_time);
    nh.getParam("state_derivative_solver_node/torque_impulse_duration", torque_impulse_duration);

    // Assign retrieved parameters to the quadcopter system
    I << inertia_values[0], inertia_values[1], inertia_values[2],
         inertia_values[3], inertia_values[4], inertia_values[5],
         inertia_values[6], inertia_values[7], inertia_values[8];
    g << gravity_values[0], gravity_values[1], gravity_values[2];
    Cd_trans << drag_coefficients_trans[0], drag_coefficients_trans[1], drag_coefficients_trans[2],
               drag_coefficients_trans[3], drag_coefficients_trans[4], drag_coefficients_trans[5],
               drag_coefficients_trans[6], drag_coefficients_trans[7], drag_coefficients_trans[8];
    A_trans << A_frame[0], A_frame[1], A_frame[2],
               A_frame[3], A_frame[4], A_frame[5],
               A_frame[6], A_frame[7], A_frame[8];
    impulse_force << impulse_force_vec[0], impulse_force_vec[1], impulse_force_vec[2];
    impulse_torque << impulse_torque_vec[0], impulse_torque_vec[1], impulse_torque_vec[2];

    // Subscribe to topics for thrust and torque control inputs
    ros::Subscriber thrust_sub = nh.subscribe("/control/thrust", 10, thrustCallback);
    ros::Subscriber torques_sub = nh.subscribe("/control/torques", 10, torquesCallback);

    // Publishers for position, euler angles, pose, velocity, angular velocity, acceleration, and impulse force/torque
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/position", 10);
    ros::Publisher euler_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/euler_angles", 10);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rviz_quad_pose", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/velocity", 10);
    ros::Publisher angular_velocity_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/angular_velocity", 10);
    ros::Publisher acceleration_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/acceleration", 10);
    ros::Publisher impulse_force_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/impulse_force", 10);
    ros::Publisher impulse_torque_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/impulse_torque", 10);

    // Transform broadcaster for RViz visualization
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Initial state: [position, rotation matrix, velocity, angular velocity]
    std::vector<double> x(18, 0.0);
    x[0] = 0.0; x[1] = 0.0; x[2] = 0.0;
    x[3] = 1.0; x[4] = 0.0; x[5] = 0.0;
    x[6] = 0.0; x[7] = 1.0; x[8] = 0.0;
    x[9] = 0.0; x[10] = 0.0; x[11] = 1.0;
    x[12] = 0.0;
    x[13] = 0.0;
    x[14] = 0.0;
    x[15] = 0.0;
    x[16] = 0.0;
    x[17] = 0.0;

    ros::Rate rate(500);  // Loop rate of 500 Hz

    // Main loop
    while (ros::ok()) {
        // Perform ODE integration to update the quadcopter state
        performODEIntegration(x, torques, thrust, I, m, g, rho, Cd_trans, A_trans, Cd_rot, A_rotor, l, I_rotor);

        // Publish the quadcopter's position
        geometry_msgs::Vector3 pos_msg;
        pos_msg.x = x[0];
        pos_msg.y = x[1];
        pos_msg.z = x[2];
        pos_pub.publish(pos_msg);

        // Compute Euler angles from the rotation matrix
        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];

        double phi = atan2(R(2, 1), R(2, 2));  // Roll angle (phi)
        double theta = -asin(R(2, 0));         // Pitch angle (theta)
        double psi = atan2(R(1, 0), R(0, 0));  // Yaw angle (psi)

        // Publish Euler angles
        geometry_msgs::Vector3 euler_msg;
        euler_msg.x = phi;
        euler_msg.y = theta;
        euler_msg.z = psi;
        euler_pub.publish(euler_msg);

        // Publish the quadcopter's pose (position and orientation) for RViz
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";

        pose_msg.pose.position.x = x[0];
        pose_msg.pose.position.y = x[1];
        pose_msg.pose.position.z = x[2];

        Eigen::Quaterniond quaternion(R);
        pose_msg.pose.orientation.x = quaternion.x();
        pose_msg.pose.orientation.y = quaternion.y();
        pose_msg.pose.orientation.z = quaternion.z();
        pose_msg.pose.orientation.w = quaternion.w();

        pose_pub.publish(pose_msg);

        // Broadcast the transform for RViz visualization
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = x[0];
        transformStamped.transform.translation.y = x[1];
        transformStamped.transform.translation.z = x[2];

        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        tf_broadcaster.sendTransform(transformStamped);

        // Publish the quadcopter's velocity
        geometry_msgs::Vector3 vel_msg;
        vel_msg.x = x[12];
        vel_msg.y = x[13];
        vel_msg.z = x[14];
        velocity_pub.publish(vel_msg);

        // Publish the quadcopter's angular velocity
        geometry_msgs::Vector3 ang_vel_msg;
        ang_vel_msg.x = x[15];
        ang_vel_msg.y = x[16];
        ang_vel_msg.z = x[17];
        angular_velocity_pub.publish(ang_vel_msg);

        // Compute the acceleration
        Eigen::Vector3d acc = Eigen::Vector3d(x[12], x[13], x[14]) / dt;  // Assuming simple finite difference for acceleration
        geometry_msgs::Vector3 acc_msg;
        acc_msg.x = acc.x();
        acc_msg.y = acc.y();
        acc_msg.z = acc.z();
        acceleration_pub.publish(acc_msg);

        // Publish impulse force and torque
        Eigen::Vector3d applied_impulse_force = applyImpulseForce(t);
        Eigen::Vector3d applied_impulse_torque = applyImpulseTorque(t);
        
        geometry_msgs::Vector3 impulse_force_msg;
        impulse_force_msg.x = applied_impulse_force.x();
        impulse_force_msg.y = applied_impulse_force.y();
        impulse_force_msg.z = applied_impulse_force.z();
        impulse_force_pub.publish(impulse_force_msg);

        geometry_msgs::Vector3 impulse_torque_msg;
        impulse_torque_msg.x = applied_impulse_torque.x();
        impulse_torque_msg.y = applied_impulse_torque.y();
        impulse_torque_msg.z = applied_impulse_torque.z();
        impulse_torque_pub.publish(impulse_torque_msg);

        // ROS spin and sleep to maintain the loop rate
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
