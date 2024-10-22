#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <functional>
#include <cmath>

class Quadcopter {
public:
    Quadcopter(ros::NodeHandle& nh) {
        loadParameters(nh);
        initializeSubscribers(nh);
        initializePublishers(nh);
        initializeState();
    }

    void update() {
        performODEIntegration();
        publishState();
    }

private:
    // Quadcopter parameters
    double m;
    Eigen::Matrix3d I;
    Eigen::Vector3d g;
    double rho;
    Eigen::Matrix3d Cd_trans;
    Eigen::Matrix3d A_trans;
    double Cd_rot, A_rotor, l, r_propeller;
    double tau_thrust, tau_torque;
    double thrust_actual;
    Eigen::Vector3d torques_actual;
    double kt_coeff, km_coeff, I_rotor;
    double force_impulse_start_time, force_impulse_duration;
    double torque_impulse_start_time, torque_impulse_duration;
    Eigen::Vector3d impulse_force, impulse_torque;
    double dt, t;

    // State variables
    std::vector<double> x;
    double thrust;
    Eigen::Vector3d torques;

    // ROS publishers and subscribers
    ros::Subscriber thrust_sub;
    ros::Subscriber torques_sub;
    ros::Publisher pos_pub, euler_pub, pose_pub, velocity_pub, angular_velocity_pub, acceleration_pub, impulse_force_pub, impulse_torque_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Callback for thrust
    void thrustCallback(const std_msgs::Float64::ConstPtr& msg) {
        thrust = msg->data;
    }

    // Callback for torques
    void torquesCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        torques.x() = msg->x;
        torques.y() = msg->y;
        torques.z() = msg->z;
    }

    // Load parameters from the ROS parameter server
    void loadParameters(ros::NodeHandle& nh) {
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
    }

    // Initialize ROS subscribers
    void initializeSubscribers(ros::NodeHandle& nh) {
        thrust_sub = nh.subscribe("/control/thrust", 10, &Quadcopter::thrustCallback, this);
        torques_sub = nh.subscribe("/control/torques", 10, &Quadcopter::torquesCallback, this);
    }

    // Initialize ROS publishers
    void initializePublishers(ros::NodeHandle& nh) {
        pos_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/position", 10);
        euler_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/euler_angles", 10);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rviz_quad_pose", 10);
        velocity_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/velocity", 10);
        angular_velocity_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/angular_velocity", 10);
        acceleration_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/acceleration", 10);
        impulse_force_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/impulse_force", 10);
        impulse_torque_pub = nh.advertise<geometry_msgs::Vector3>("/quadcopter/impulse_torque", 10);
    }

    // Initialize the quadcopter's state
    void initializeState() {
        x = std::vector<double>(18, 0.0);
        x[3] = 1.0; x[7] = 1.0; x[11] = 1.0;
        thrust_actual = 0.0;
        torques_actual.setZero();
        t = 0.0;
    }

    // Update thrust and torques
    void updateThrustAndTorques() {
        updateThrust(thrust, thrust_actual, tau_thrust, dt);
        updateTorques(torques, torques_actual, tau_torque, dt);
    }

    // Perform ODE integration
    void performODEIntegration() {
        updateThrustAndTorques();

        Eigen::Vector4d rotor_speeds;
        getRotorSpeeds(thrust_actual, torques_actual, kt_coeff, km_coeff, l, rotor_speeds);

        Eigen::Vector3d thrust_vector(0, 0, thrust_actual);

        using namespace boost::numeric::odeint;
        typedef runge_kutta_cash_karp54<std::vector<double>> error_stepper_type;
        typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
        controlled_stepper_type controlled_stepper;

        integrate_adaptive(
            controlled_stepper,
            [&](const std::vector<double>& x, std::vector<double>& dxdt, double t) {
                computeStateDerivatives(x, dxdt, t, thrust_vector, torques_actual, I, m, g, rho, Cd_trans, A_trans, Cd_rot, A_rotor, l, rotor_speeds, I_rotor);
            },
            x, t, t + dt, dt
        );

        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();
        x[3] = R(0, 0); x[4] = R(0, 1); x[5] = R(0, 2);
        x[6] = R(1, 0); x[7] = R(1, 1); x[8] = R(1, 2);
        x[9] = R(2, 0); x[10] = R(2, 1); x[11] = R(2, 2);

        t += dt;
    }

    // Compute rotor speeds from thrust and torques
    void getRotorSpeeds(double thrust, const Eigen::Vector3d& torques, double kt_coeff, double km_coeff, double l, Eigen::Vector4d& omega) {
        Eigen::Vector4d omega_squared;
        computeRotorSpeeds(thrust, torques, kt_coeff, km_coeff, l, omega_squared);
        omega(0) = sqrt(omega_squared(0));
        omega(1) = sqrt(omega_squared(1));
        omega(2) = sqrt(omega_squared(2));
        omega(3) = sqrt(omega_squared(3));
    }

    void computeRotorSpeeds(double thrust, const Eigen::Vector3d& torques, double kt_coeff, double km_coeff, double l, Eigen::Vector4d& omega_squared) {
        Eigen::Matrix4d A;
        Eigen::Vector4d b;

        double a_term = kt_coeff * l / sqrt(2);
        A << kt_coeff, kt_coeff, kt_coeff, kt_coeff,
             a_term, -a_term, -a_term, a_term,
             -a_term, a_term, -a_term, a_term,
             km_coeff, km_coeff, -km_coeff, -km_coeff;

        b << thrust, torques[0], torques[1], torques[2];
        omega_squared = A.colPivHouseholderQr().solve(b);
    }

    // Publish the state (position, velocity, etc.)
    void publishState() {
        geometry_msgs::Vector3 pos_msg;
        pos_msg.x = x[0];
        pos_msg.y = x[1];
        pos_msg.z = x[2];
        pos_pub.publish(pos_msg);

        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];
        double phi = atan2(R(2, 1), R(2, 2));
        double theta = -asin(R(2, 0));
        double psi = atan2(R(1, 0), R(0, 0));

        geometry_msgs::Vector3 euler_msg;
        euler_msg.x = phi;
        euler_msg.y = theta;
        euler_msg.z = psi;
        euler_pub.publish(euler_msg);

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

        geometry_msgs::Vector3 vel_msg;
        vel_msg.x = x[12];
        vel_msg.y = x[13];
        vel_msg.z = x[14];
        velocity_pub.publish(vel_msg);

        geometry_msgs::Vector3 ang_vel_msg;
        ang_vel_msg.x = x[15];
        ang_vel_msg.y = x[16];
        ang_vel_msg.z = x[17];
        angular_velocity_pub.publish(ang_vel_msg);

        Eigen::Vector3d acc = Eigen::Vector3d(x[12], x[13], x[14]) / dt;
        geometry_msgs::Vector3 acc_msg;
        acc_msg.x = acc.x();
        acc_msg.y = acc.y();
        acc_msg.z = acc.z();
        acceleration_pub.publish(acc_msg);

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
    }

    Eigen::Vector3d applyImpulseForce(double current_time) {
        if (current_time >= force_impulse_start_time && current_time <= force_impulse_start_time + force_impulse_duration) {
            return impulse_force;
        }
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d applyImpulseTorque(double current_time) {
        if (current_time >= torque_impulse_start_time && current_time <= torque_impulse_start_time + torque_impulse_duration) {
            return impulse_torque;
        }
        return Eigen::Vector3d::Zero();
    }

    void updateThrust(double thrust_desired, double& thrust_actual, double tau_thrust, double dt) {
        double dF = (1.0 / tau_thrust) * (thrust_desired - thrust_actual);
        thrust_actual += dF * dt;
    }

    void updateTorques(const Eigen::Vector3d& torques_desired, Eigen::Vector3d& torques_actual, double tau_torque, double dt) {
        Eigen::Vector3d dTau = (1.0 / tau_torque) * (torques_desired - torques_actual);
        torques_actual += dTau * dt;
    }

    void computeStateDerivatives(const std::vector<double>& x, std::vector<double>& dxdt, double t, const Eigen::Vector3d& thrust_vector, const Eigen::Vector3d& torques, const Eigen::Matrix3d& I, double m, const Eigen::Vector3d& g, double rho, const Eigen::Matrix3d& Cd_trans, const Eigen::Matrix3d& A_trans, double Cd_rot, double A_rotor, double l, const Eigen::Vector4d& rotor_speeds, double I_rotor) {
        Eigen::Vector3d r(x[0], x[1], x[2]);
        Eigen::Matrix3d R;
        R << x[3], x[4], x[5],
             x[6], x[7], x[8],
             x[9], x[10], x[11];
        Eigen::Vector3d V(x[12], x[13], x[14]);
        Eigen::Vector3d w(x[15], x[16], x[17]);

        dxdt[0] = V[0];
        dxdt[1] = V[1];
        dxdt[2] = V[2];

        Eigen::Matrix3d skew_w = skewSymmetric(w);
        Eigen::Matrix3d dR = skew_w * R;
        dxdt[3] = dR(0, 0); dxdt[4] = dR(0, 1); dxdt[5] = dR(0, 2);
        dxdt[6] = dR(1, 0); dxdt[7] = dR(1, 1); dxdt[8] = dR(1, 2);
        dxdt[9] = dR(2, 0); dxdt[10] = dR(2, 1); dxdt[11] = dR(2, 2);

        Eigen::Vector3d drag_forces = computeDragForces(V, rho, Cd_trans, A_trans);
        Eigen::Vector3d impulse_force = applyImpulseForce(t);
        Eigen::Vector3d dV = (R * thrust_vector + drag_forces + impulse_force + m * g) / m;
        dxdt[12] = dV[0];
        dxdt[13] = dV[1];
        dxdt[14] = dV[2];

        Eigen::Vector3d drag_torques = computeDragTorques(rotor_speeds, rho, Cd_rot, l, A_rotor, km_coeff);
        Eigen::Vector3d gyro_torques = computeGyroscopicTorque(w, rotor_speeds, I_rotor);
        Eigen::Vector3d impulse_torque = applyImpulseTorque(t);
        Eigen::Vector3d wxIw = skew_w * (I * w);
        Eigen::Vector3d dw = I.inverse() * (torques + drag_torques + gyro_torques + impulse_torque - wxIw);
        dxdt[15] = dw[0];
        dxdt[16] = dw[1];
        dxdt[17] = dw[2];
    }

    Eigen::Vector3d computeDragForces(const Eigen::Vector3d& V, double rho, const Eigen::Matrix3d& Cd_trans, const Eigen::Matrix3d& A_trans) {
        Eigen::Vector3d V_squared = V.array().square();
        Eigen::Vector3d drag_force = -0.5 * rho * (Cd_trans * A_trans * V_squared);
        return drag_force;
    }

    Eigen::Vector3d computeDragTorques(const Eigen::Vector4d& omega_squared, double rho, double Cd_rot, double r_propeller, double A_rotor, double km_coeff) {
        double b_term = -0.5 * rho * Cd_rot * A_rotor * std::pow(r_propeller, 2);
        Eigen::Vector4d drag_torques = b_term * omega_squared;

        Eigen::Matrix<double, 3, 4> rotor_matrix;
        double a_term = kt_coeff * l / sqrt(2);
        rotor_matrix << a_term, -a_term, -a_term, a_term,
                        -a_term, a_term, -a_term, a_term,
                        km_coeff, km_coeff, -km_coeff, -km_coeff;

        Eigen::Vector3d total_drag_torques = rotor_matrix * drag_torques;
        return total_drag_torques;
    }

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vec) {
        Eigen::Matrix3d skew;
        skew << 0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
        return skew;
    }

    Eigen::Vector3d computeGyroscopicTorque(const Eigen::Vector3d& omega_body, const Eigen::Vector4d& rotor_speeds, double I_rotor) {
        Eigen::Vector4d rotor_directions;
        rotor_directions << -1.0, -1.0, 1.0, 1.0;

        Eigen::Vector3d gyro_torque = Eigen::Vector3d::Zero();
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d rotor_angular_velocity(0, 0, rotor_speeds(i) * rotor_directions(i));
            gyro_torque += I_rotor * omega_body.cross(rotor_angular_velocity);
        }
        return gyro_torque;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "quadcopter_eom");
    ros::NodeHandle nh;

    Quadcopter quadcopter(nh);

    ros::Rate rate(500);

    while (ros::ok()) {
        quadcopter.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
