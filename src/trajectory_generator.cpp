#include "../include/quadcopter_control/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle& nh) 
    : m_no_more_trajectory(false), m_initial_trajectory(true) {
    m_position_pub = nh.advertise<geometry_msgs::Vector3>("reference_position", 10);
    m_velocity_pub = nh.advertise<geometry_msgs::Vector3>("reference_velocity", 10);
    m_acceleration_pub = nh.advertise<geometry_msgs::Vector3>("reference_acceleration", 10);
    m_trajectory_client = nh.serviceClient<quadcopter_control::WaypointService>("get_trajectory");

    if (getTrajectoryFromServer()) {
        ROS_INFO("Trajectory data received from server.");
    } else {
        ROS_ERROR("Failed to get trajectory data from server.");
    }
}

bool TrajectoryGenerator::getTrajectoryFromServer() {
    quadcopter_control::WaypointService srv;

    if (m_trajectory_client.call(srv)) {
        m_points_x = srv.response.points_x;
        m_points_y = srv.response.points_y;
        m_points_z = srv.response.points_z;
        m_times = srv.response.times;
        m_ros_rate = srv.response.ros_rate;
        m_trajectory_method = srv.response.method;

        if (m_points_x.empty() || m_points_y.empty() || m_points_z.empty() || m_times.empty()) {
            ROS_WARN("Empty trajectory data received. No further requests will be made.");
            m_no_more_trajectory = true;
            return false;
        }

        return true;
    } else {
        return false;
    }
}

void TrajectoryGenerator::generateTrajectory() {
    ros::Rate loop_rate(m_ros_rate);
    while (ros::ok() && !m_no_more_trajectory) {
        if (m_trajectory_method == "cubic_spline") {
            ROS_INFO("Cubic spline has been selected.");
            solveCubicSpline();
        } else if (m_trajectory_method == "natural_cubic_spline") {
            ROS_INFO("Natural cubic spline has been selected.");
            solveNaturalCubicSpline();
        } else if (m_trajectory_method == "minimum_jerk") {
            ROS_INFO("Minimum jerk has been selected.");
            solveMinimumJerk();
        } else if (m_trajectory_method == "minimum_snap") {
            ROS_INFO("Minimum snap has been selected.");
            solveMinimumSnap();
        }

        if (!m_no_more_trajectory && getTrajectoryFromServer()) {
            ROS_INFO("New trajectory data received from server.");
            continue;
        } else if (m_no_more_trajectory) {
            ROS_INFO("No more trajectory data available. Stopping.");
            break;
        } else {
            ROS_ERROR("Failed to get new trajectory data from server.");
            break;
        }
    }
}

void TrajectoryGenerator::solveNaturalCubicSpline() {

   // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
    std::vector<double> points_x_mod = m_points_x;
    std::vector<double> points_y_mod = m_points_y;
    std::vector<double> points_z_mod = m_points_z;
    std::vector<double> times_mod = m_times;

    if (!m_initial_trajectory)
    {
        points_x_mod.insert(points_x_mod.begin(), m_position.x);
        points_y_mod.insert(points_y_mod.begin(), m_position.y);
        points_z_mod.insert(points_z_mod.begin(), m_position.z);
        double last_time = times_mod.back() + 4;
        times_mod.push_back(last_time);      
    }

    // times_mod dizisini ekrana yazdır
    ROS_INFO("Modified times_mod values:");
    for (size_t i = 0; i < times_mod.size(); ++i) {
        ROS_INFO("times_mod[%ld] = %f", i, times_mod[i]);
    }

    int n = points_x_mod.size() - 1; // Segment sayısı, nokta sayısından bir eksik
    int matrix_size = 4 * n;     // 4n boyutunda matris (a_i, b_i, c_i, d_i)
    
    // A matrisi (4n x 4n) ve b vektörü (4n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);

    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {

        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 4 * i + 0) = 1;
        A(2 * i, 4 * i + 1) = 0;
        A(2 * i, 4 * i + 2) = 0;
        A(2 * i, 4 * i + 3) = 0;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];

        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 4 * i + 0) = 1;
        A(2 * i + 1, 4 * i + 1) = dt;
        A(2 * i + 1, 4 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 4 * i + 3) = std::pow(dt, 3);
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + (i - 1), 4 * (i - 1) + 1) = 1;
        A(2 * n + (i - 1), 4 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + (i - 1), 4 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + (i - 1), 4 * i + 1) = -1;
        b_x(2 * n + (i - 1)) = 0;
        b_y(2 * n + (i - 1)) = 0;
        b_z(2 * n + (i - 1)) = 0;
    }
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(3 * n + (i - 2), 4 * (i - 1) + 2) = 2;
        A(3 * n - 1 + (i - 1), 4 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n - 1 + (i - 1), 4 * i + 2) = -2;
        b_x(3 * n + (i - 2)) = 0;
        b_y(3 * n + (i - 2)) = 0;
        b_z(3 * n + (i - 2)) = 0;
    }
    
    // 4. initial acceleration and final acceleration conditions
    A(4 * n - 2, 2) = 2;

    if (m_initial_trajectory)
    {
        b_x(4 * n - 2) = 0;
        b_y(4 * n - 2) = 0;
        b_z(4 * n - 2) = 0;

        m_initial_trajectory = false;
    } else
    {
        b_x(4 * n - 2) = m_acceleration.x;
        b_y(4 * n - 2) = m_acceleration.y;
        b_z(4 * n - 2) = m_acceleration.z;
    }
    
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(4 * n - 1, 4 * (n - 1) + 2) = 2;
    A(4 * n - 1, 4 * (n - 1) + 3) = 6 * dt_last;
    b_x(4 * n - 1) = 0;
    b_y(4 * n - 1) = 0;
    b_z(4 * n - 1) = 0;

    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);
   
    // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
    ros::Rate loop_rate(m_ros_rate); // Config dosyasından alınan ROS rate
    for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle

        // Hangi segmentte olduğumuzu bulalım
        int i = 0;
        while (i < n && t > times_mod[i + 1]) {
            i++;
        }

        double dt = t - times_mod[i];
        double pos_x = coeffs_x(4 * i + 0) + coeffs_x(4 * i + 1) * dt + coeffs_x(4 * i + 2) * std::pow(dt, 2) + coeffs_x(4 * i + 3) * std::pow(dt, 3);
        double pos_y = coeffs_y(4 * i + 0) + coeffs_y(4 * i + 1) * dt + coeffs_y(4 * i + 2) * std::pow(dt, 2) + coeffs_y(4 * i + 3) * std::pow(dt, 3);
        double pos_z = coeffs_z(4 * i + 0) + coeffs_z(4 * i + 1) * dt + coeffs_z(4 * i + 2) * std::pow(dt, 2) + coeffs_z(4 * i + 3) * std::pow(dt, 3);
        
        double vel_x = coeffs_x(4 * i + 1) + 2 * coeffs_x(4 * i + 2) * dt + 3 * coeffs_x(4 * i + 3) * std::pow(dt, 2);
        double vel_y = coeffs_y(4 * i + 1) + 2 * coeffs_y(4 * i + 2) * dt + 3 * coeffs_y(4 * i + 3) * std::pow(dt, 2);
        double vel_z = coeffs_z(4 * i + 1) + 2 * coeffs_z(4 * i + 2) * dt + 3 * coeffs_z(4 * i + 3) * std::pow(dt, 2);

        double acc_x = 2 * coeffs_x(4 * i + 2) + 6 * coeffs_x(4 * i + 3) * dt;
        double acc_y = 2 * coeffs_y(4 * i + 2) + 6 * coeffs_y(4 * i + 3) * dt;
        double acc_z = 2 * coeffs_z(4 * i + 2) + 6 * coeffs_z(4 * i + 3) * dt;

        double jerk_x = 6 * coeffs_x(4 * i + 3);
        double jerk_y = 6 * coeffs_y(4 * i + 3);
        double jerk_z = 6 * coeffs_z(4 * i + 3);

        // Konumu doldur
        m_position.x = pos_x;
        m_position.y = pos_y;
        m_position.z = pos_z;

        // Velocity Pub
        m_velocity.x = vel_x;
        m_velocity.y = vel_y;
        m_velocity.z = vel_z;

        // Acceleration Pub
        m_acceleration.x = acc_x;
        m_acceleration.y = acc_y;
        m_acceleration.z = acc_z;
        
        // Jerk
        m_jerk.x = jerk_x;
        m_jerk.y = jerk_y;
        m_jerk.z = jerk_z;

        // 3. segmente ulaşıldığında yeni waypoint seti iste
        if (i == 3 && getTrajectoryFromServer()) {
            ROS_INFO("New waypoint set received. The trajectory will be generated again.");
            generateTrajectory();  // Yeni verilerle yörüngeyi yeniden başlat
            return;
        }

        // Mesajları yayınla
        m_position_pub.publish(m_position);
        m_velocity_pub.publish(m_velocity);
        m_acceleration_pub.publish(m_acceleration);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TrajectoryGenerator::solveCubicSpline() {

   // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
    std::vector<double> points_x_mod = m_points_x;
    std::vector<double> points_y_mod = m_points_y;
    std::vector<double> points_z_mod = m_points_z;
    std::vector<double> times_mod = m_times;

    if (!m_initial_trajectory)
    {
        points_x_mod.insert(points_x_mod.begin(), m_position.x);
        points_y_mod.insert(points_y_mod.begin(), m_position.y);
        points_z_mod.insert(points_z_mod.begin(), m_position.z);
        double last_time = times_mod.back() + 7;
        times_mod.push_back(last_time);      
    }

    int n = points_x_mod.size() - 1; // Segment sayısı, nokta sayısından bir eksik
    int matrix_size = 4 * n;     // 4n boyutunda matris (a_i, b_i, c_i, d_i)
    
    // A matrisi (4n x 4n) ve b vektörü (4n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);

    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {

        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 4 * i + 0) = 1;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];

        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 4 * i + 0) = 1;
        A(2 * i + 1, 4 * i + 1) = dt;
        A(2 * i + 1, 4 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 4 * i + 3) = std::pow(dt, 3);
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + (i - 1), 4 * (i - 1) + 1) = 1;
        A(2 * n + (i - 1), 4 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + (i - 1), 4 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + (i - 1), 4 * i + 1) = -1;
    }
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(3 * n + (i - 2), 4 * (i - 1) + 2) = 2;
        A(3 * n - 1 + (i - 1), 4 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n - 1 + (i - 1), 4 * i + 2) = -2;
    }
    
    // 4. initial velocity and final velocity conditions
    A(4 * n - 2, 1) = 1;

    if (m_initial_trajectory)
    {
        b_x(4 * n - 2) = 0;
        b_y(4 * n - 2) = 0;
        b_z(4 * n - 2) = 0;
        m_initial_trajectory = false;
    } else
    {
        b_x(4 * n - 2) = m_velocity.x;
        b_y(4 * n - 2) = m_velocity.y;
        b_z(4 * n - 2) = m_velocity.z;
    }
    
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(4 * n - 1, 4 * (n - 1) + 1) = 1;
    A(4 * n - 1, 4 * (n - 1) + 2) = 2 * dt_last;
    A(4 * n - 1, 4 * (n - 1) + 3) = 3 * std::pow(dt_last, 2);

    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);
   
    // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
    ros::Rate loop_rate(m_ros_rate); // Config dosyasından alınan ROS rate
    for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle

        // Hangi segmentte olduğumuzu bulalım
        int i = 0;
        while (i < n && t > times_mod[i + 1]) {
            i++;
        }
        double dt = t - times_mod[i];
        ROS_INFO("t: %f", dt);
        double pos_x = coeffs_x(4 * i + 0) + coeffs_x(4 * i + 1) * dt + coeffs_x(4 * i + 2) * std::pow(dt, 2) + coeffs_x(4 * i + 3) * std::pow(dt, 3);
        double pos_y = coeffs_y(4 * i + 0) + coeffs_y(4 * i + 1) * dt + coeffs_y(4 * i + 2) * std::pow(dt, 2) + coeffs_y(4 * i + 3) * std::pow(dt, 3);
        double pos_z = coeffs_z(4 * i + 0) + coeffs_z(4 * i + 1) * dt + coeffs_z(4 * i + 2) * std::pow(dt, 2) + coeffs_z(4 * i + 3) * std::pow(dt, 3);
        ROS_INFO("x: %f   y: %f   z: %f", pos_x, pos_y, pos_z);

        double vel_x = coeffs_x(4 * i + 1) + 2 * coeffs_x(4 * i + 2) * dt + 3 * coeffs_x(4 * i + 3) * std::pow(dt, 2);
        double vel_y = coeffs_y(4 * i + 1) + 2 * coeffs_y(4 * i + 2) * dt + 3 * coeffs_y(4 * i + 3) * std::pow(dt, 2);
        double vel_z = coeffs_z(4 * i + 1) + 2 * coeffs_z(4 * i + 2) * dt + 3 * coeffs_z(4 * i + 3) * std::pow(dt, 2);

        double acc_x = 2 * coeffs_x(4 * i + 2) + 6 * coeffs_x(4 * i + 3) * dt;
        double acc_y = 2 * coeffs_y(4 * i + 2) + 6 * coeffs_y(4 * i + 3) * dt;
        double acc_z = 2 * coeffs_z(4 * i + 2) + 6 * coeffs_z(4 * i + 3) * dt;

        double jerk_x = 6 * coeffs_x(4 * i + 3);
        double jerk_y = 6 * coeffs_y(4 * i + 3);
        double jerk_z = 6 * coeffs_z(4 * i + 3);

        // Konumu doldur
        m_position.x = pos_x;
        m_position.y = pos_y;
        m_position.z = pos_z;

        // Velocity Pub
        m_velocity.x = vel_x;
        m_velocity.y = vel_y;
        m_velocity.z = vel_z;

        // Acceleration Pub
        m_acceleration.x = acc_x;
        m_acceleration.y = acc_y;
        m_acceleration.z = acc_z;

        // Jerk
        m_jerk.x = jerk_x;
        m_jerk.y = jerk_y;
        m_jerk.z = jerk_z;
        
        // Mesajları yayınla
        m_position_pub.publish(m_position);
        m_velocity_pub.publish(m_velocity);
        m_acceleration_pub.publish(m_acceleration);
        
        if (i == 3 && getTrajectoryFromServer()) {
            ROS_INFO("New waypoint set received. The trajectory will be generated again.");
            generateTrajectory();
            return;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TrajectoryGenerator::solveMinimumJerk() {

    // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
    std::vector<double> points_x_mod = m_points_x;
    std::vector<double> points_y_mod = m_points_y;
    std::vector<double> points_z_mod = m_points_z;
    std::vector<double> times_mod = m_times;

    if (!m_initial_trajectory)
    {
        points_x_mod.insert(points_x_mod.begin(), m_position.x);
        points_y_mod.insert(points_y_mod.begin(), m_position.y);
        points_z_mod.insert(points_z_mod.begin(), m_position.z);
        double last_time = times_mod.back() + 7;
        times_mod.push_back(last_time);      
    }
    

    int n = points_x_mod.size() - 1; // Segment sayısı
    int matrix_size = 6 * n;     // Minimum jerk polinom çözümü için 6n boyutunda matris

    // A matrisi (6n x 6n) ve b vektörleri (6n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);

    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {

        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 6 * i) = 1;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];

        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 6 * i + 0) = 1;
        A(2 * i + 1, 6 * i + 1) = dt;
        A(2 * i + 1, 6 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 6 * i + 3) = std::pow(dt, 3);
        A(2 * i + 1, 6 * i + 4) = std::pow(dt, 4);
        A(2 * i + 1, 6 * i + 5) = std::pow(dt, 5);
        
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }

    // Başlangıç hız koşulu (initial velocity)
    A(2 * n, 1) = 1;      // İlk segmentin başındaki hız için katsayılar
    if (m_initial_trajectory)
    {
        b_x(2 * n) = 0;  // X ekseni için başlangıç hızı
        b_y(2 * n) = 0;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = 0;  // Z ekseni için başlangıç hızı
    } else
    {
        b_x(2 * n) = m_velocity.x;  // X ekseni için başlangıç hızı
        b_y(2 * n) = m_velocity.y;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = m_velocity.z;  // Z ekseni için başlangıç hızı
    }
        
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + i, 6 * (i - 1) + 1) = 1;
        A(2 * n + i, 6 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + i, 6 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + i, 6 * (i - 1) + 4) = 4 * std::pow(dt_prev, 3);
        A(2 * n + i, 6 * (i - 1) + 5) = 5 * std::pow(dt_prev, 4);
        A(2 * n + i, 6 * i + 1) = -1;
        // b vektöründe hız sürekliliği denklemleri için sıfır ekleniyor
        b_x(2 * n + i) = 0;  // X ekseni için hız sürekliliği
        b_y(2 * n + i) = 0;  // Y ekseni için hız sürekliliği
        b_z(2 * n + i) = 0;  // Z ekseni için hız sürekliliği
    }

    // Son hız değeri (last velocity condition) -> dx_n(t_n) = v_n
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(3 * n, 6 * (n - 1) + 1) = 1;
    A(3 * n, 6 * (n - 1) + 2) = 2 * dt_last;
    A(3 * n, 6 * (n - 1) + 3) = 3 * std::pow(dt_last, 2);
    A(3 * n, 6 * (n - 1) + 4) = 4 * std::pow(dt_last, 3);
    A(3 * n, 6 * (n - 1) + 5) = 5 * std::pow(dt_last, 4);

    // Son hız değeri sağlanacak: b_x, b_y, b_z vektörlerine son hız koşulu eklenmeli
    b_x(3 * n) = 0;  // Son hız değeri
    b_y(3 * n) = 0;  // Son hız değeri
    b_z(3 * n) = 0;  // Son hız değeri

    // Başlangıç ivmesi (initial acceleration)
    A(3 * n + 1, 2) = 2;
    
    if (m_initial_trajectory)
    {
        b_x(3 * n + 1) = 0;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = 0;  // Z ekseni için başlangıç ivmesi
        m_initial_trajectory = false;
    } else
    {
        b_x(3 * n + 1) = m_acceleration.x;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = m_acceleration.y;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = m_acceleration.z;  // Z ekseni için başlangıç ivmesi
    }
       
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        // İvme sürekliliği denklemleri
        A(3 * n + 1 + i, 6 * (i - 1) + 2) = 2;
        A(3 * n + 1 + i, 6 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n + 1 + i, 6 * (i - 1) + 4) = 12 * std::pow(dt_prev, 2);
        A(3 * n + 1 + i, 6 * (i - 1) + 5) = 20 * std::pow(dt_prev, 3);
        A(3 * n + 1 + i, 6 * i + 2) = -2;
        // b vektörlerine ivme sürekliliği için sıfır ekleniyor
        b_x(3 * n + 1 + i) = 0;
        b_y(3 * n + 1 + i) = 0;
        b_z(3 * n + 1 + i) = 0;
    }
    
    // Bitiş ivmesi (final acceleration)
    A(4 * n + 1, 6 * (n - 1) + 2) = 2;
    A(4 * n + 1, 6 * (n - 1) + 3) = 6 * dt_last;
    A(4 * n + 1, 6 * (n - 1) + 4) = 12 * std::pow(dt_last, 2);
    A(4 * n + 1, 6 * (n - 1) + 5) = 20 * std::pow(dt_last, 3);
    b_x(4 * n + 1) = 0;  // X ekseni için bitiş ivmesi
    b_y(4 * n + 1) = 0;  // Y ekseni için bitiş ivmesi
    b_z(4 * n + 1) = 0;  // Z ekseni için bitiş ivmesi
    
    // 4. Jerk sürekliliği (Jerk Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(4 * n - 1 + (i + 2), 6 * (i - 1) + 3) = 6;
        A(4 * n - 1 + (i + 2), 6 * (i - 1) + 4) = 24 * dt_prev;
        A(4 * n - 1 + (i + 2), 6 * (i - 1) + 5) = 60 * std::pow(dt_prev, 2);
        A(4 * n - 1 + (i + 2), 6 * i + 3) = -6;
        b_x(4 * n - 1 + (i + 2)) = 0;  
        b_y(4 * n - 1 + (i + 2)) = 0;  
        b_z(4 * n - 1 + (i + 2)) = 0;  
    }
    // 5. Snap sürekliliği (Snap Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(5 * n + i, 6 * (i - 1) + 4) = 24;
        A(5 * n + i, 6 * (i - 1) + 5) = 120 * dt_prev;
        A(5 * n + i, 6 * i + 4) = -24;
        b_x(5 * n + i) = 0;  
        b_y(5 * n + i) = 0;  
        b_z(5 * n + i) = 0;  
    }
    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

    // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
    ros::Rate loop_rate(m_ros_rate); // Config dosyasından alınan ROS rate
    for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle

        // Hangi segmentte olduğumuzu bulalım
        int i = 0;
        while (i < n && t > times_mod[i + 1]) {
            i++;
        }

        double dt = t - times_mod[i];
        //ROS_INFO("t: %f", t);
        double pos_x = coeffs_x(6 * i + 0) + coeffs_x(6 * i + 1) * dt + coeffs_x(6 * i + 2) * std::pow(dt, 2) +
                    coeffs_x(6 * i + 3) * std::pow(dt, 3) + coeffs_x(6 * i + 4) * std::pow(dt, 4) + coeffs_x(6 * i + 5) * std::pow(dt, 5);
        double pos_y = coeffs_y(6 * i + 0) + coeffs_y(6 * i + 1) * dt + coeffs_y(6 * i + 2) * std::pow(dt, 2) +
                    coeffs_y(6 * i + 3) * std::pow(dt, 3) + coeffs_y(6 * i + 4) * std::pow(dt, 4) + coeffs_y(6 * i + 5) * std::pow(dt, 5);
        double pos_z = coeffs_z(6 * i + 0) + coeffs_z(6 * i + 1) * dt + coeffs_z(6 * i + 2) * std::pow(dt, 2) +
                    coeffs_z(6 * i + 3) * std::pow(dt, 3) + coeffs_z(6 * i + 4) * std::pow(dt, 4) + coeffs_z(6 * i + 5) * std::pow(dt, 5);
        
        double vel_x = coeffs_x(6 * i + 1) + 2 * coeffs_x(6 * i + 2) * std::pow(dt, 1) +
                    3 * coeffs_x(6 * i + 3) * std::pow(dt, 2) + 4 * coeffs_x(6 * i + 4) * std::pow(dt, 3) + 5 * coeffs_x(6 * i + 5) * std::pow(dt, 4);
        double vel_y = coeffs_y(6 * i + 1) + 2 * coeffs_y(6 * i + 2) * std::pow(dt, 1) +
                    3 * coeffs_y(6 * i + 3) * std::pow(dt, 2) + 4 * coeffs_y(6 * i + 4) * std::pow(dt, 3) + 5 * coeffs_y(6 * i + 5) * std::pow(dt, 4);
        double vel_z = coeffs_z(6 * i + 1) + 2 * coeffs_z(6 * i + 2) * std::pow(dt, 1) +
                    3 * coeffs_z(6 * i + 3) * std::pow(dt, 2) + 4 * coeffs_z(6 * i + 4) * std::pow(dt, 3) + 5 * coeffs_z(6 * i + 5) * std::pow(dt, 4);
        
        double acc_x = 2 * coeffs_x(6 * i + 2) + 6 * coeffs_x(6 * i + 3) * dt + 12 * coeffs_x(6 * i + 4) * std::pow(dt, 2) + 20 * coeffs_x(6 * i + 5) * std::pow(dt, 3);
        double acc_y = 2 * coeffs_y(6 * i + 2) + 6 * coeffs_y(6 * i + 3) * dt + 12 * coeffs_y(6 * i + 4) * std::pow(dt, 2) + 20 * coeffs_y(6 * i + 5) * std::pow(dt, 3);
        double acc_z = 2 * coeffs_z(6 * i + 2) + 6 * coeffs_z(6 * i + 3) * dt + 12 * coeffs_z(6 * i + 4) * std::pow(dt, 2) + 20 * coeffs_z(6 * i + 5) * std::pow(dt, 3);

        double jerk_x = 6 * coeffs_x(6 * i + 3) + 24 * coeffs_x(6 * i + 4) * dt + 60 * coeffs_x(6 * i + 5) * std::pow(dt, 2);
        double jerk_y = 6 * coeffs_y(6 * i + 3) + 24 * coeffs_y(6 * i + 4) * dt + 60 * coeffs_y(6 * i + 5) * std::pow(dt, 2);
        double jerk_z = 6 * coeffs_z(6 * i + 3) + 24 * coeffs_z(6 * i + 4) * dt + 60 * coeffs_z(6 * i + 5) * std::pow(dt, 2);

        // Konumu doldur
        m_position.x = pos_x;
        m_position.y = pos_y;
        m_position.z = pos_z;
        
        // Velocity Pub
        m_velocity.x = vel_x;
        m_velocity.y = vel_y;
        m_velocity.z = vel_z;

        // Acceleration Pub
        m_acceleration.x = acc_x;
        m_acceleration.y = acc_y;
        m_acceleration.z = acc_z;

        // Jerk
        m_jerk.x = jerk_x;
        m_jerk.y = jerk_y;
        m_jerk.z = jerk_z;

        // Mesajları yayınla
        m_position_pub.publish(m_position);
        m_velocity_pub.publish(m_velocity);
        m_acceleration_pub.publish(m_acceleration);

        // 3. segmente ulaşıldığında yeni waypoint seti iste
        if (i == 3 && getTrajectoryFromServer()) {
            ROS_INFO("New waypoint set received. The trajectory will be generated again.");
            generateTrajectory();  // Yeni verilerle yörüngeyi yeniden başlat
            return;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TrajectoryGenerator::solveMinimumSnap() {
    
    // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
    std::vector<double> points_x_mod = m_points_x;
    std::vector<double> points_y_mod = m_points_y;
    std::vector<double> points_z_mod = m_points_z;
    std::vector<double> times_mod = m_times;

    if (!m_initial_trajectory)
    {
        points_x_mod.insert(points_x_mod.begin(), m_position.x);
        points_y_mod.insert(points_y_mod.begin(), m_position.y);
        points_z_mod.insert(points_z_mod.begin(), m_position.z);
        double last_time = times_mod.back() + 7;
        times_mod.push_back(last_time);      

        // Print points_x_mod
        std::cout << "points_x_mod: ";
        for (const auto& x : points_x_mod) {
            std::cout << x << " ";
        }
        std::cout << std::endl;

        // Print points_y_mod
        std::cout << "points_y_mod: ";
        for (const auto& y : points_y_mod) {
            std::cout << y << " ";
        }
        std::cout << std::endl;

        // Print points_z_mod
        std::cout << "points_z_mod: ";
        for (const auto& z : points_z_mod) {
            std::cout << z << " ";
        }
        std::cout << std::endl;
    }

    int n = points_x_mod.size() - 1; // Segment sayısı
    int matrix_size = 8 * n;     // Minimum jerk polinom çözümü için 8n boyutunda matris
    
    // A matrisi (8n x 8n) ve b vektörleri (8n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);
    
    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {
        
        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 8 * i) = 1;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];
        
        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 8 * i + 0) = 1;
        A(2 * i + 1, 8 * i + 1) = dt;
        A(2 * i + 1, 8 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 8 * i + 3) = std::pow(dt, 3);
        A(2 * i + 1, 8 * i + 4) = std::pow(dt, 4);
        A(2 * i + 1, 8 * i + 5) = std::pow(dt, 5);
        A(2 * i + 1, 8 * i + 6) = std::pow(dt, 6);
        A(2 * i + 1, 8 * i + 7) = std::pow(dt, 7);
        
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }
    
    // Başlangıç hız koşulu (initial velocity)
    A(2 * n, 1) = 1;      // İlk segmentin başındaki hız için katsayılar
    
    if (m_initial_trajectory)
    {
        b_x(2 * n) = 0;  // X ekseni için başlangıç hızı
        b_y(2 * n) = 0;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = 0;  // Z ekseni için başlangıç hızı
    }
    else
    {
        b_x(2 * n) = m_velocity.x;  // X ekseni için başlangıç hızı
        b_y(2 * n) = m_velocity.y;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = m_velocity.z;  // Z ekseni için başlangıç hızı    /* code */
    }
    
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + i, 8 * (i - 1) + 1) = 1;
        A(2 * n + i, 8 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + i, 8 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + i, 8 * (i - 1) + 4) = 4 * std::pow(dt_prev, 3);
        A(2 * n + i, 8 * (i - 1) + 5) = 5 * std::pow(dt_prev, 4);
        A(2 * n + i, 8 * (i - 1) + 6) = 6 * std::pow(dt_prev, 5);
        A(2 * n + i, 8 * (i - 1) + 7) = 7 * std::pow(dt_prev, 6);
        A(2 * n + i, 8 * i + 1) = -1;
        // b vektöründe hız sürekliliği denklemleri için sıfır ekleniyor
        b_x(2 * n + i) = 0;  // X ekseni için hız sürekliliği
        b_y(2 * n + i) = 0;  // Y ekseni için hız sürekliliği
        b_z(2 * n + i) = 0;  // Z ekseni için hız sürekliliği
    }

    // Son hız değeri (last velocity condition) -> dx_n(t_n) = v_n
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(3 * n, 8 * (n - 1) + 1) = 1;
    A(3 * n, 8 * (n - 1) + 2) = 2 * dt_last;
    A(3 * n, 8 * (n - 1) + 3) = 3 * std::pow(dt_last, 2);
    A(3 * n, 8 * (n - 1) + 4) = 4 * std::pow(dt_last, 3);
    A(3 * n, 8 * (n - 1) + 5) = 5 * std::pow(dt_last, 4);
    A(3 * n, 8 * (n - 1) + 6) = 6 * std::pow(dt_last, 5);
    A(3 * n, 8 * (n - 1) + 7) = 7 * std::pow(dt_last, 6);

    // Son hız değeri sağlanacak: b_x, b_y, b_z vektörlerine son hız koşulu eklenmeli
    b_x(3 * n) = 0;  // Son hız değeri
    b_y(3 * n) = 0;  // Son hız değeri
    b_z(3 * n) = 0;  // Son hız değeri

    // Başlangıç ivmesi (initial acceleration)
    A(3 * n + 1, 2) = 2;
    
    if (m_initial_trajectory)
    {
        b_x(3 * n + 1) = 0;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = 0;  // Z ekseni için başlangıç ivmesi
    }
    else
    {
        b_x(3 * n + 1) = m_acceleration.x;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = m_acceleration.y;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = m_acceleration.z;  // Z ekseni için başlangıç ivmesi
    }
    
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        // İvme sürekliliği denklemleri
        A(3 * n + 1 + i, 8 * (i - 1) + 2) = 2;
        A(3 * n + 1 + i, 8 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n + 1 + i, 8 * (i - 1) + 4) = 12 * std::pow(dt_prev, 2);
        A(3 * n + 1 + i, 8 * (i - 1) + 5) = 20 * std::pow(dt_prev, 3);
        A(3 * n + 1 + i, 8 * (i - 1) + 6) = 30 * std::pow(dt_prev, 4);
        A(3 * n + 1 + i, 8 * (i - 1) + 7) = 42 * std::pow(dt_prev, 5);
        A(3 * n + 1 + i, 8 * i + 2) = -2;
        // b vektörlerine ivme sürekliliği için sıfır ekleniyor
        b_x(3 * n + 1 + i) = 0;
        b_y(3 * n + 1 + i) = 0;
        b_z(3 * n + 1 + i) = 0;
    }
    
    // Bitiş ivmesi (final acceleration)
    A(4 * n + 1, 8 * (n - 1) + 2) = 2;
    A(4 * n + 1, 8 * (n - 1) + 3) = 6 * dt_last;
    A(4 * n + 1, 8 * (n - 1) + 4) = 12 * std::pow(dt_last, 2);
    A(4 * n + 1, 8 * (n - 1) + 5) = 20 * std::pow(dt_last, 3);
    A(4 * n + 1, 8 * (n - 1) + 6) = 30 * std::pow(dt_last, 4);
    A(4 * n + 1, 8 * (n - 1) + 7) = 42 * std::pow(dt_last, 5);
    b_x(4 * n + 1) = 0;  // X ekseni için bitiş ivmesi
    b_y(4 * n + 1) = 0;  // Y ekseni için bitiş ivmesi
    b_z(4 * n + 1) = 0;  // Z ekseni için bitiş ivmesi

    // Başlangıç jerk (initial jerk)
    A(4 * n + 2, 3) = 6;
    
    if (m_initial_trajectory)
    {
        b_x(4 * n + 2) = 0;  // X ekseni için başlangıç jerk
        b_y(4 * n + 2) = 0;  // Y ekseni için başlangıç jerk
        b_z(4 * n + 2) = 0;  // Z ekseni için başlangıç jerk
        m_initial_trajectory = false;
    }
    else
    {
        b_x(4 * n + 2) = m_jerk.x;  // X ekseni için başlangıç jerk
        b_y(4 * n + 2) = m_jerk.y;  // Y ekseni için başlangıç jerk
        b_z(4 * n + 2) = m_jerk.z;  // Z ekseni için başlangıç jerk
    }
    
    // 4. Jerk sürekliliği (Jerk Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(4 * n + 2 + i, 8 * (i - 1) + 3) = 6;
        A(4 * n + 2 + i, 8 * (i - 1) + 4) = 24 * dt_prev;
        A(4 * n + 2 + i, 8 * (i - 1) + 5) = 60 * std::pow(dt_prev, 2);
        A(4 * n + 2 + i, 8 * (i - 1) + 6) = 120 * std::pow(dt_prev, 3);
        A(4 * n + 2 + i, 8 * (i - 1) + 7) = 210 * std::pow(dt_prev, 4);
        A(4 * n + 2 + i, 8 * i + 3) = -6;
        b_x(4 * n + 2 + i) = 0;  
        b_y(4 * n + 2 + i) = 0;  
        b_z(4 * n + 2 + i) = 0;  
    }
    
    // Bitiş jerk (final jerk)
    A(5 * n + 2, 8 * (n - 1) + 3) = 6;
    A(5 * n + 2, 8 * (n - 1) + 4) = 24 * dt_last;
    A(5 * n + 2, 8 * (n - 1) + 5) = 60 * std::pow(dt_last, 2);
    A(5 * n + 2, 8 * (n - 1) + 6) = 120 * std::pow(dt_last, 3);
    A(5 * n + 2, 8 * (n - 1) + 7) = 210 * std::pow(dt_last, 4);
    b_x(5 * n + 2) = 0;  // X ekseni için bitiş ivmesi
    b_y(5 * n + 2) = 0;  // Y ekseni için bitiş ivmesi
    b_z(5 * n + 2) = 0;  // Z ekseni için bitiş ivmesi
    
    // 5. Snap sürekliliği (Snap Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(5 * n + i + 2, 8 * (i - 1) + 4) = 24;
        A(5 * n + i + 2, 8 * (i - 1) + 5) = 120 * dt_prev;
        A(5 * n + i + 2, 8 * (i - 1) + 6) = 360 * std::pow(dt_prev, 2);
        A(5 * n + i + 2, 8 * (i - 1) + 7) = 840 * std::pow(dt_prev, 3);
        A(5 * n + i + 2, 8 * i + 4) = -24;
        b_x(5 * n + i + 2) = 0;  
        b_y(5 * n + i + 2) = 0;  
        b_z(5 * n + i + 2) = 0;  
    }
    
    // 6. Crackle sürekliliği (Crackle Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(6 * n + i + 1, 8 * (i - 1) + 5) = 120;
        A(6 * n + i + 1, 8 * (i - 1) + 6) = 720 * dt_prev;
        A(6 * n + i + 1, 8 * (i - 1) + 7) = 2520 * std::pow(dt_prev, 2);
        A(6 * n + i + 1, 8 * i + 5) = -120;
        b_x(6 * n + i + 1) = 0;  
        b_y(6 * n + i + 1) = 0;  
        b_z(6 * n + i + 1) = 0;  
    }
    
    // 7. Pop sürekliliği (Pop Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(7 * n + i, 8 * (i - 1) + 6) = 720;
        A(7 * n + i, 8 * (i - 1) + 7) = 5040 * dt_prev;
        A(7 * n + i, 8 * i + 6) = -720;
        b_x(7 * n + i) = 0;  
        b_y(7 * n + i) = 0;  
        b_z(7 * n + i) = 0;  
    }
    
    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);
    bool asd = true;
    // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
    ros::Rate loop_rate(m_ros_rate); // Config dosyasından alınan ROS rate
    for (double t = times_mod[0]; t <= times_mod[n]; t += 0.075) {  // Zaman adımlarıyla ilerle
        
        // Hangi segmentte olduğumuzu bulalım
        int i = 0;
        while (i < n && t > times_mod[i + 1]) {
            i++;
        }
      
        double dt = t - times_mod[i];
        ROS_INFO("t: %f", t);
        double pos_x = coeffs_x(8 * i + 0) + coeffs_x(8 * i + 1) * dt + coeffs_x(8 * i + 2) * std::pow(dt, 2) +
                    coeffs_x(8 * i + 3) * std::pow(dt, 3) + coeffs_x(8 * i + 4) * std::pow(dt, 4) + coeffs_x(8 * i + 5) * std::pow(dt, 5) + coeffs_x(8 * i + 6) * std::pow(dt, 6) + coeffs_x(8 * i + 7) * std::pow(dt, 7);
        double pos_y = coeffs_y(8 * i + 0) + coeffs_y(8 * i + 1) * dt + coeffs_y(8 * i + 2) * std::pow(dt, 2) +
                    coeffs_y(8 * i + 3) * std::pow(dt, 3) + coeffs_y(8 * i + 4) * std::pow(dt, 4) + coeffs_y(8 * i + 5) * std::pow(dt, 5) + coeffs_y(8 * i + 6) * std::pow(dt, 6) + coeffs_y(8 * i + 7) * std::pow(dt, 7);
        double pos_z = coeffs_z(8 * i + 0) + coeffs_z(8 * i + 1) * dt + coeffs_z(8 * i + 2) * std::pow(dt, 2) +
                    coeffs_z(8 * i + 3) * std::pow(dt, 3) + coeffs_z(8 * i + 4) * std::pow(dt, 4) + coeffs_z(8 * i + 5) * std::pow(dt, 5) + coeffs_z(8 * i + 6) * std::pow(dt, 6) + coeffs_z(8 * i + 7) * std::pow(dt, 7);
        //ROS_INFO("x: %f   y: %f   z: %f", pos_x, pos_y, pos_z);
        
        double vel_x = coeffs_x(8 * i + 1) + 2 * coeffs_x(8 * i + 2) * dt +
                    3 * coeffs_x(8 * i + 3) * std::pow(dt, 2) + 4 * coeffs_x(8 * i + 4) * std::pow(dt, 3) + 5 * coeffs_x(8 * i + 5) * std::pow(dt, 4) + 6 * coeffs_x(8 * i + 6) * std::pow(dt, 5) + 7 * coeffs_x(8 * i + 7) * std::pow(dt, 6);
        double vel_y = coeffs_y(8 * i + 1) + 2 * coeffs_y(8 * i + 2) * dt +
                    3 * coeffs_y(8 * i + 3) * std::pow(dt, 2) + 4 * coeffs_y(8 * i + 4) * std::pow(dt, 3) + 5 * coeffs_y(8 * i + 5) * std::pow(dt, 4) + 6 * coeffs_y(8 * i + 6) * std::pow(dt, 5) + 7 * coeffs_y(8 * i + 7) * std::pow(dt, 6);
        double vel_z = coeffs_z(8 * i + 1) + 2 * coeffs_z(8 * i + 2) * dt +
                    3 * coeffs_z(8 * i + 3) * std::pow(dt, 2) + 4 * coeffs_z(8 * i + 4) * std::pow(dt, 3) + 5 * coeffs_z(8 * i + 5) * std::pow(dt, 4) + 6 * coeffs_z(8 * i + 6) * std::pow(dt, 5) + 7 * coeffs_z(8 * i + 7) * std::pow(dt, 6);        

        double acc_x = 2 * coeffs_x(8 * i + 2) + 6 * coeffs_x(8 * i + 3) * dt + 12 * coeffs_x(8 * i + 4) * std::pow(dt, 2) + 20 * coeffs_x(8 * i + 5) * std::pow(dt, 3) + 30 * coeffs_x(8 * i + 6) * std::pow(dt, 4) + 42 * coeffs_x(8 * i + 7) * std::pow(dt, 5);
        double acc_y = 2 * coeffs_y(8 * i + 2) + 6 * coeffs_y(8 * i + 3) * dt + 12 * coeffs_y(8 * i + 4) * std::pow(dt, 2) + 20 * coeffs_y(8 * i + 5) * std::pow(dt, 3) + 30 * coeffs_y(8 * i + 6) * std::pow(dt, 4) + 42 * coeffs_y(8 * i + 7) * std::pow(dt, 5);
        double acc_z = 2 * coeffs_z(8 * i + 2) + 6 * coeffs_z(8 * i + 3) * dt + 12 * coeffs_z(8 * i + 4) * std::pow(dt, 2) + 20 * coeffs_z(8 * i + 5) * std::pow(dt, 3) + 30 * coeffs_z(8 * i + 6) * std::pow(dt, 4) + 42 * coeffs_z(8 * i + 7) * std::pow(dt, 5);

        double jerk_x = 6 * coeffs_x(8 * i + 3) + 24 * coeffs_x(8 * i + 4) * dt + 60 * coeffs_x(8 * i + 5) * std::pow(dt, 2) + 120 * coeffs_x(8 * i + 6) * std::pow(dt, 3) + 210 * coeffs_x(8 * i + 7) * std::pow(dt, 4);
        double jerk_y = 6 * coeffs_y(8 * i + 3) + 24 * coeffs_y(8 * i + 4) * dt + 60 * coeffs_y(8 * i + 5) * std::pow(dt, 2) + 120 * coeffs_y(8 * i + 6) * std::pow(dt, 3) + 210 * coeffs_y(8 * i + 7) * std::pow(dt, 4);
        double jerk_z = 6 * coeffs_z(8 * i + 3) + 24 * coeffs_z(8 * i + 4) * dt + 60 * coeffs_z(8 * i + 5) * std::pow(dt, 2) + 120 * coeffs_z(8 * i + 6) * std::pow(dt, 3) + 210 * coeffs_z(8 * i + 7) * std::pow(dt, 4);

        // Konumu doldur
        m_position.x = pos_x;
        m_position.y = pos_y;
        m_position.z = pos_z;
        
        // Velocity Pub
        m_velocity.x = vel_x;
        m_velocity.y = vel_y;
        m_velocity.z = vel_z;

        // Acceleration Pub
        m_acceleration.x = acc_x;
        m_acceleration.y = acc_y;
        m_acceleration.z = acc_z;

        // Jerk Pub
        m_jerk.x = jerk_x;
        m_jerk.y = jerk_y;
        m_jerk.z = jerk_z;

        // 3. segmente ulaşıldığında yeni waypoint seti iste
        if (i == 3 && getTrajectoryFromServer()) {
            ROS_INFO("New waypoint set received. The trajectory will be generated again.");
            generateTrajectory();  // Yeni verilerle yörüngeyi yeniden başlat
            return;
        }

        // Mesajları yayınla
        m_position_pub.publish(m_position);
        m_velocity_pub.publish(m_velocity);
        m_acceleration_pub.publish(m_acceleration);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
