#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <quadcopter_control/WaypointService.h> 

class TrajectoryGenerator {
public:
    TrajectoryGenerator(ros::NodeHandle& nh) {
        // Publisher'lar
        position_pub = nh.advertise<geometry_msgs::Vector3>("position", 10);
        psi_pub = nh.advertise<std_msgs::Float64>("psi", 10);
        waypoints_pub = nh.advertise<geometry_msgs::Vector3>("waypoints", 10);  // Waypoints yayınlayıcı

        // Servis istemcisi oluştur
        trajectory_client = nh.serviceClient<quadcopter_control::WaypointService>("get_trajectory");

        // Servisten verileri al ve yörüngeyi yükle
        if (getTrajectoryFromServer()) {
            publishWaypoints();  // Waypoints verilerini yayınla
        } else {
            ROS_ERROR("Failed to get trajectory data from server.");
        }
    }

    bool getTrajectoryFromServer() {
        // Servis talebi oluştur
        quadcopter_control::WaypointService srv;

        // Servisi çağır ve talep yanıtını al
        if (trajectory_client.call(srv)) {
            points_x = srv.response.points_x;
            points_y = srv.response.points_y;
            points_z = srv.response.points_z;
            times = srv.response.times;
            ros_rate = srv.response.ros_rate;
            return_to_start = srv.response.return_to_start;
            return_duration = srv.response.return_duration;
            trajectory_method = srv.response.method;

            // Verileri ekrana yazdır
            for (size_t i = 0; i < points_x.size(); ++i) {
            }
            ROS_INFO("ROS Rate: %.2f", ros_rate);
            ROS_INFO("Return to start: %s", return_to_start ? "true" : "false");
            if (return_to_start) {
                ROS_INFO("Return duration: %.2f", return_duration);
            }
            ROS_INFO("Trajectory method: %s", trajectory_method.c_str());

            return true;  // Servisten başarıyla veri alındı
        } else {
            return false;  // Servis çağrısı başarısız oldu
        }
    }

    void generateTrajectory() {
        // Trajectory yöntemi config dosyasından alınıyor
        if (trajectory_method == "cubic_spline") {
            ROS_INFO("Cubic spline trajectory selected.");
            solveCubicSpline();
        } else if (trajectory_method == "natural_cubic_spline") {
            ROS_INFO("Natural cubic spline trajectory selected.");
            solveNaturalCubicSpline();
        } else if (trajectory_method == "minimum_jerk") {
            ROS_INFO("Minimum jerk trajectory selected.");
            solveMinimumJerk();
        } else if (trajectory_method == "minimum_snap") {
            ROS_INFO("Minimum snap trajectory selected.");
            solveMinimumSnap();
        } else {
            ROS_WARN("Invalid trajectory method selected. Defaulting to cubic spline.");
            solveCubicSpline();
        }
    }

    void publishWaypoints() {
        ros::Rate rate(ros_rate);  // Set ROS loop rate

        // Wait 1 second before starting the publishing
        ros::Duration(1.0).sleep();  

        for (size_t i = 0; i < points_x.size(); ++i) {
            geometry_msgs::Vector3 waypoint;
            waypoint.x = points_x[i];
            waypoint.y = points_y[i];
            waypoint.z = points_z[i];
            waypoints_pub.publish(waypoint);

            ros::spinOnce();   // ROS message loop
            rate.sleep();      // Wait before publishing next waypoint

            // Wait 2 seconds after publishing each waypoint
            ros::Duration(0.5).sleep();  
        }
    }

    void solveNaturalCubicSpline() {
        // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
        std::vector<double> points_x_mod = points_x;
        std::vector<double> points_y_mod = points_y;
        std::vector<double> points_z_mod = points_z;
        std::vector<double> times_mod = times;

        if (return_to_start) {
            points_x_mod.push_back(points_x[0]);
            points_y_mod.push_back(points_y[0]);
            points_z_mod.push_back(points_z[0]);

            // Yapılandırma dosyasından alınan dönüş süresini ekleyelim
            times_mod.push_back(times.back() + return_duration);
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
            A(2 * i, 4 * i + 0) = 0;
            A(2 * i, 4 * i + 1) = 0;
            A(2 * i, 4 * i + 2) = 0;
            A(2 * i, 4 * i + 3) = 1;

            b_x(2 * i) = points_x_mod[i];
            b_y(2 * i) = points_y_mod[i];
            b_z(2 * i) = points_z_mod[i];

            // S_i(t_{i+1}) = P_{i+1}
            double dt = times_mod[i + 1] - times_mod[i];
            A(2 * i + 1, 4 * i + 0) = std::pow(dt, 3);
            A(2 * i + 1, 4 * i + 1) = std::pow(dt, 2);
            A(2 * i + 1, 4 * i + 2) = dt;
            A(2 * i + 1, 4 * i + 3) = 1;

            b_x(2 * i + 1) = points_x_mod[i + 1];
            b_y(2 * i + 1) = points_y_mod[i + 1];
            b_z(2 * i + 1) = points_z_mod[i + 1];
        }

        // 2. Hız sürekliliği (Velocity Continuity)
        for (int i = 1; i < n; ++i) {
            double dt_prev = times_mod[i] - times_mod[i - 1];

            A(2 * n + (i - 1), 4 * (i - 1) + 0) = 3 * std::pow(dt_prev, 2);
            A(2 * n + (i - 1), 4 * (i - 1) + 1) = 2 * dt_prev;
            A(2 * n + (i - 1), 4 * (i - 1) + 2) = 1;

            A(2 * n + (i - 1), 4 * i + 2) = -1;
        }

        // 3. İvme sürekliliği (Acceleration Continuity)
        for (int i = 1; i < n; ++i) {
            double dt_prev = times_mod[i] - times_mod[i - 1];

            A(3 * n - 1 + (i - 1), 4 * (i - 1) + 0) = 6 * dt_prev;
            A(3 * n - 1 + (i - 1), 4 * (i - 1) + 1) = 2;

            A(3 * n - 1 + (i - 1), 4 * i + 1) = -2;
        }

        // 4. Doğal spline sınır koşulları (Natural Spline Boundary Conditions)
        A(4 * n - 2, 0) = 0;
        A(4 * n - 2, 1) = 2;

        double dt_last = times_mod[n] - times_mod[n - 1];
        A(4 * n - 1, 4 * (n - 1) + 0) = 6 * dt_last;
        A(4 * n - 1, 4 * (n - 1) + 1) = 2;

        // Denklemi çöz (Ax = b) her eksen için
        Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
        Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
        Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

        // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
        ros::Rate loop_rate(ros_rate); // Config dosyasından alınan ROS rate

        for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle
            geometry_msgs::Vector3 position;
            std_msgs::Float64 psi;

            // Hangi segmentte olduğumuzu bulalım
            int i = 0;
            while (i < n && t > times_mod[i + 1]) {
                i++;
            }

            double dt = t - times_mod[i];

            double pos_x = coeffs_x(4 * i + 0) * std::pow(dt, 3) + coeffs_x(4 * i + 1) * std::pow(dt, 2) + coeffs_x(4 * i + 2) * dt + coeffs_x(4 * i + 3);
            double pos_y = coeffs_y(4 * i + 0) * std::pow(dt, 3) + coeffs_y(4 * i + 1) * std::pow(dt, 2) + coeffs_y(4 * i + 2) * dt + coeffs_y(4 * i + 3);
            double pos_z = coeffs_z(4 * i + 0) * std::pow(dt, 3) + coeffs_z(4 * i + 1) * std::pow(dt, 2) + coeffs_z(4 * i + 2) * dt + coeffs_z(4 * i + 3);

            // Konumu ve psi'yi doldur
            position.x = pos_x;
            position.y = pos_y;
            position.z = pos_z;
            psi.data = 0.0;    // Psi'yi sabit tutuyoruz

            // Mesajları yayınla
            position_pub.publish(position);
            psi_pub.publish(psi);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void solveCubicSpline() {
        // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
        std::vector<double> points_x_mod = points_x;
        std::vector<double> points_y_mod = points_y;
        std::vector<double> points_z_mod = points_z;
        std::vector<double> times_mod = times;

        if (return_to_start) {
            points_x_mod.push_back(points_x[0]);
            points_y_mod.push_back(points_y[0]);
            points_z_mod.push_back(points_z[0]);

            // Yapılandırma dosyasından alınan dönüş süresini ekleyelim
            times_mod.push_back(times.back() + return_duration);
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
            A(2 * i, 4 * i + 0) = 0;
            A(2 * i, 4 * i + 1) = 0;
            A(2 * i, 4 * i + 2) = 0;
            A(2 * i, 4 * i + 3) = 1;

            b_x(2 * i) = points_x_mod[i];
            b_y(2 * i) = points_y_mod[i];
            b_z(2 * i) = points_z_mod[i];

            // S_i(t_{i+1}) = P_{i+1}
            double dt = times_mod[i + 1] - times_mod[i];
            A(2 * i + 1, 4 * i + 0) = std::pow(dt, 3);
            A(2 * i + 1, 4 * i + 1) = std::pow(dt, 2);
            A(2 * i + 1, 4 * i + 2) = dt;
            A(2 * i + 1, 4 * i + 3) = 1;

            b_x(2 * i + 1) = points_x_mod[i + 1];
            b_y(2 * i + 1) = points_y_mod[i + 1];
            b_z(2 * i + 1) = points_z_mod[i + 1];
        }

        // 2. Hız sürekliliği (Velocity Continuity)
        for (int i = 1; i < n; ++i) {
            double dt_prev = times_mod[i] - times_mod[i - 1];

            A(2 * n + (i - 1), 4 * (i - 1) + 0) = 3 * std::pow(dt_prev, 2);
            A(2 * n + (i - 1), 4 * (i - 1) + 1) = 2 * dt_prev;
            A(2 * n + (i - 1), 4 * (i - 1) + 2) = 1;

            A(2 * n + (i - 1), 4 * i + 2) = -1;
        }

        // 3. İvme sürekliliği (Acceleration Continuity)
        for (int i = 1; i < n; ++i) {
            double dt_prev = times_mod[i] - times_mod[i - 1];

            A(3 * n - 1 + (i - 1), 4 * (i - 1) + 0) = 6 * dt_prev;
            A(3 * n - 1 + (i - 1), 4 * (i - 1) + 1) = 2;

            A(3 * n - 1 + (i - 1), 4 * i + 1) = -2;
        }

        double dt_last = times_mod[n] - times_mod[n - 1];
        A(4 * n - 1, 4 * (n - 1) + 0) = 3 * std::pow(dt_last, 2);
        A(4 * n - 1, 4 * (n - 1) + 1) = 2 * dt_last;

        // Denklemi çöz (Ax = b) her eksen için
        Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
        Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
        Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

        // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
        ros::Rate loop_rate(ros_rate); // Config dosyasından alınan ROS rate

        for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle
            geometry_msgs::Vector3 position;
            std_msgs::Float64 psi;

            // Hangi segmentte olduğumuzu bulalım
            int i = 0;
            while (i < n && t > times_mod[i + 1]) {
                i++;
            }

            double dt = t - times_mod[i];

            double pos_x = coeffs_x(4 * i + 0) * std::pow(dt, 3) + coeffs_x(4 * i + 1) * std::pow(dt, 2) + coeffs_x(4 * i + 2) * dt + coeffs_x(4 * i + 3);
            double pos_y = coeffs_y(4 * i + 0) * std::pow(dt, 3) + coeffs_y(4 * i + 1) * std::pow(dt, 2) + coeffs_y(4 * i + 2) * dt + coeffs_y(4 * i + 3);
            double pos_z = coeffs_z(4 * i + 0) * std::pow(dt, 3) + coeffs_z(4 * i + 1) * std::pow(dt, 2) + coeffs_z(4 * i + 2) * dt + coeffs_z(4 * i + 3);

            // Konumu ve psi'yi doldur
            position.x = pos_x;
            position.y = pos_y;
            position.z = pos_z;
            psi.data = 0.0;    // Psi'yi sabit tutuyoruz

            // Mesajları yayınla
            position_pub.publish(position);
            psi_pub.publish(psi);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void solveMinimumJerk() {
        // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
        std::vector<double> points_x_mod = points_x;
        std::vector<double> points_y_mod = points_y;
        std::vector<double> points_z_mod = points_z;
        std::vector<double> times_mod = times;

        if (return_to_start) {
            points_x_mod.push_back(points_x[0]);
            points_y_mod.push_back(points_y[0]);
            points_z_mod.push_back(points_z[0]);

            // Yapılandırma dosyasından alınan dönüş süresini ekleyelim
            times_mod.push_back(times.back() + return_duration);
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
            A(2 * i, 6 * i + 0) = 1;

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
        b_x(2 * n) = 0;  // X ekseni için başlangıç hızı
        b_y(2 * n) = 0;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = 0;  // Z ekseni için başlangıç hızı

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
        b_x(3 * n + 1) = 0;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = 0;  // Z ekseni için başlangıç ivmesi

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
        ros::Rate loop_rate(ros_rate); // Config dosyasından alınan ROS rate

        for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle
            geometry_msgs::Vector3 position;
            std_msgs::Float64 psi;

            // Hangi segmentte olduğumuzu bulalım
            int i = 0;
            while (i < n && t > times_mod[i + 1]) {
                i++;
            }

            double dt = t - times_mod[i];

            double pos_x = coeffs_x(6 * i + 0) + coeffs_x(6 * i + 1) * dt + coeffs_x(6 * i + 2) * std::pow(dt, 2) +
                        coeffs_x(6 * i + 3) * std::pow(dt, 3) + coeffs_x(6 * i + 4) * std::pow(dt, 4) + coeffs_x(6 * i + 5) * std::pow(dt, 5);
            double pos_y = coeffs_y(6 * i + 0) + coeffs_y(6 * i + 1) * dt + coeffs_y(6 * i + 2) * std::pow(dt, 2) +
                        coeffs_y(6 * i + 3) * std::pow(dt, 3) + coeffs_y(6 * i + 4) * std::pow(dt, 4) + coeffs_y(6 * i + 5) * std::pow(dt, 5);
            double pos_z = coeffs_z(6 * i + 0) + coeffs_z(6 * i + 1) * dt + coeffs_z(6 * i + 2) * std::pow(dt, 2) +
                        coeffs_z(6 * i + 3) * std::pow(dt, 3) + coeffs_z(6 * i + 4) * std::pow(dt, 4) + coeffs_z(6 * i + 5) * std::pow(dt, 5);

            // Konumu ve psi'yi doldur
            position.x = pos_x;
            position.y = pos_y;
            position.z = pos_z;
            psi.data = 0.0;    // Psi'yi sabit tutuyoruz

            // Mesajları yayınla
            position_pub.publish(position);
            psi_pub.publish(psi);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void solveMinimumSnap() {
        // Eğer başlangıç noktasına geri dönülecekse, son noktayı başlangıç noktası olarak ekleyelim
        std::vector<double> points_x_mod = points_x;
        std::vector<double> points_y_mod = points_y;
        std::vector<double> points_z_mod = points_z;
        std::vector<double> times_mod = times;

        if (return_to_start) {
            points_x_mod.push_back(points_x[0]);
            points_y_mod.push_back(points_y[0]);
            points_z_mod.push_back(points_z[0]);

            // Yapılandırma dosyasından alınan dönüş süresini ekleyelim
            times_mod.push_back(times.back() + return_duration);
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
        b_x(2 * n) = 0;  // X ekseni için başlangıç hızı
        b_y(2 * n) = 0;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = 0;  // Z ekseni için başlangıç hızı

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
        b_x(3 * n + 1) = 0;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = 0;  // Z ekseni için başlangıç ivmesi

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
        b_x(4 * n + 2) = 0;  // X ekseni için başlangıç ivmesi
        b_y(4 * n + 2) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(4 * n + 2) = 0;  // Z ekseni için başlangıç ivmesi

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

        // Çözülen katsayıları kullanarak konumları ve psi'yi zaman adımlarıyla yayınla
        ros::Rate loop_rate(ros_rate); // Config dosyasından alınan ROS rate

        for (double t = times_mod[0]; t <= times_mod[n]; t += 0.05) {  // Zaman adımlarıyla ilerle
            geometry_msgs::Vector3 position;
            std_msgs::Float64 psi;

            // Hangi segmentte olduğumuzu bulalım
            int i = 0;
            while (i < n && t > times_mod[i + 1]) {
                i++;
            }

            double dt = t - times_mod[i];

            double pos_x = coeffs_x(8 * i + 0) + coeffs_x(8 * i + 1) * dt + coeffs_x(8 * i + 2) * std::pow(dt, 2) +
                        coeffs_x(8 * i + 3) * std::pow(dt, 3) + coeffs_x(8 * i + 4) * std::pow(dt, 4) + coeffs_x(8 * i + 5) * std::pow(dt, 5) + coeffs_x(8 * i + 6) * std::pow(dt, 6) + coeffs_x(8 * i + 7) * std::pow(dt, 7);
            double pos_y = coeffs_y(8 * i + 0) + coeffs_y(8 * i + 1) * dt + coeffs_y(8 * i + 2) * std::pow(dt, 2) +
                        coeffs_y(8 * i + 3) * std::pow(dt, 3) + coeffs_y(8 * i + 4) * std::pow(dt, 4) + coeffs_y(8 * i + 5) * std::pow(dt, 5) + coeffs_y(8 * i + 6) * std::pow(dt, 6) + coeffs_y(8 * i + 7) * std::pow(dt, 7);
            double pos_z = coeffs_z(8 * i + 0) + coeffs_z(8 * i + 1) * dt + coeffs_z(8 * i + 2) * std::pow(dt, 2) +
                        coeffs_z(8 * i + 3) * std::pow(dt, 3) + coeffs_z(8 * i + 4) * std::pow(dt, 4) + coeffs_z(8 * i + 5) * std::pow(dt, 5) + coeffs_z(8 * i + 6) * std::pow(dt, 6) + coeffs_z(8 * i + 7) * std::pow(dt, 7);

            // Konumu ve psi'yi doldur
            position.x = pos_x;
            position.y = pos_y;
            position.z = pos_z;
            psi.data = 0.0;    // Psi'yi sabit tutuyoruz

            // Mesajları yayınla
            position_pub.publish(position);
            psi_pub.publish(psi);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::Publisher position_pub;
    ros::Publisher psi_pub;
    ros::Publisher waypoints_pub;  // Waypoints yayıncı
    ros::ServiceClient trajectory_client;  // Servis istemcisi
    std::vector<double> points_x, points_y, points_z, times;
    double ros_rate;
    bool return_to_start;
    double return_duration;  // Dönüş süresi yapılandırma dosyasından okunur
    std::string trajectory_method;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    // TrajectoryGenerator sınıfını oluştur ve çözümü başlat
    TrajectoryGenerator traj_gen(nh);
    traj_gen.generateTrajectory();

    ros::spin();
    return 0;
}