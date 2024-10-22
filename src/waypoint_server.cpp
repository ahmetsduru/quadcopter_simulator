#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <quadcopter_control/WaypointService.h>  // Servis başlık dosyasını dahil et

class WaypointServer {
public:
    WaypointServer(ros::NodeHandle& nh) {
        // Servis sunucusunu oluştur
        service = nh.advertiseService("get_trajectory", &WaypointServer::getTrajectoryCallback, this);

        // active_trajectories parametresini oku (birden fazla olabilir)
        XmlRpc::XmlRpcValue active_trajectories;
        if (nh.getParam("trajectory_manager/active_trajectories", active_trajectories)) {
            ROS_ASSERT(active_trajectories.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int i = 0; i < active_trajectories.size(); ++i) {
                std::string trajectory_name = static_cast<std::string>(active_trajectories[i]);
                loadTrajectory(nh, trajectory_name);  // Her aktif trajectory'yi yükle
            }
        } else {
            ROS_ERROR("Failed to get active_trajectories from trajectory_manager");
        }

        // return_to_start ve return_duration parametrelerini trajectory_manager'dan kontrol et
        nh.param("trajectory_manager/return_to_start", return_to_start, false);  // Varsayılan false
        nh.param("trajectory_manager/return_duration", return_duration, 0.0);   // Varsayılan 0.0
    }

    bool getTrajectoryCallback(quadcopter_control::WaypointService::Request &req,
                               quadcopter_control::WaypointService::Response &res) {
        // Gelen talebe yanıt olarak birleştirilmiş yörünge verilerini döndür
        res.method = trajectory_method;
        res.ros_rate = ros_rate;
        res.return_to_start = return_to_start;
        res.return_duration = return_duration;
        res.points_x = points_x;
        res.points_y = points_y;
        res.points_z = points_z;
        res.times = times;

        ROS_INFO("Combined trajectory data sent to client.");
        return true;
    }

    void loadTrajectory(ros::NodeHandle& nh, const std::string& trajectory_namespace) {
        // Geçici değişkenlerde yeni yörünge verilerini sakla
        std::vector<double> temp_points_x, temp_points_y, temp_points_z, temp_times;
        double temp_ros_rate;
        std::string temp_trajectory_method;

        // İsim alanına göre yörünge verilerini yükle
        std::string trajectory_ns = "/" + trajectory_namespace;
        nh.getParam(trajectory_ns + "/points_x", temp_points_x);
        nh.getParam(trajectory_ns + "/points_y", temp_points_y);
        nh.getParam(trajectory_ns + "/points_z", temp_points_z);
        nh.getParam(trajectory_ns + "/times", temp_times);
        nh.getParam(trajectory_ns + "/ros_rate", temp_ros_rate);
        nh.getParam(trajectory_ns + "/method", temp_trajectory_method);

        // Yörüngeleri birleştir: Zaman ve pozisyonları mevcut yörünge listesine ekle
        if (points_x.empty()) {
            ros_rate = temp_ros_rate;  // İlk yörünge olduğunda ros_rate'i ayarla
            trajectory_method = temp_trajectory_method;  // İlk yörünge için method ayarla
        }

        // Zamanları birleştirirken son zaman değerine ekleyerek ilerleyelim
        double last_time = times.empty() ? 0.0 : times.back();
        for (size_t i = 0; i < temp_points_x.size(); ++i) {
            points_x.push_back(temp_points_x[i]);
            points_y.push_back(temp_points_y[i]);
            points_z.push_back(temp_points_z[i]);
            times.push_back(temp_times[i] + last_time);
        }

        ROS_INFO("Loaded trajectory: %s", trajectory_namespace.c_str());
    }

private:
    ros::ServiceServer service;
    std::vector<double> points_x, points_y, points_z, times;
    double ros_rate;
    bool return_to_start;
    double return_duration;
    std::string trajectory_method;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_server");
    ros::NodeHandle nh;

    // WaypointServer sınıfını oluştur
    WaypointServer traj_server(nh);

    ros::spin();
    return 0;
}
