#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <quadcopter_control/WaypointService.h>  // Servis başlık dosyasını dahil et

class WaypointServer {
public:
    WaypointServer(ros::NodeHandle& nh) : current_trajectory_index(0), last_time_offset(0.0) {
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
        // Eğer tüm yörüngeler işlendi ise boş verileri gönder, ancak method, ros_rate ve diğer parametreler neyse o kalsın
        if (current_trajectory_index >= trajectories.size()) {
            res.points_x.clear();
            res.points_y.clear();
            res.points_z.clear();
            res.times.clear();
            // ros_rate, method, return_to_start ve return_duration değişmiyor, son geçerli değerler neyse onları tutuyoruz
            ROS_INFO("All trajectories completed. Sending empty trajectory points and times.");
            return true;
        }

        // Sıradaki yörüngeyi alın
        const auto& traj = trajectories[current_trajectory_index];

        // Gelen talebe yanıt olarak sıradaki yörünge verilerini döndür
        res.method = traj.method;
        res.ros_rate = traj.ros_rate;
        res.return_to_start = return_to_start;
        res.return_duration = return_duration;
        res.points_x = traj.points_x;
        res.points_y = traj.points_y;
        res.points_z = traj.points_z;

        // Zamanları last_time_offset ekleyerek güncelle
        res.times.clear();
        for (const auto& time : traj.times) {
            res.times.push_back(time + last_time_offset);
        }

        // Son yörüngenin zamanının son değerini güncelle
        last_time_offset = res.times.back();

        // Sonraki istekte bir sonraki yörüngeyi göndermek için indeksi güncelle
        current_trajectory_index++;

        ROS_INFO("Sent trajectory data for trajectory: %d", current_trajectory_index);
        return true;
    }

    void loadTrajectory(ros::NodeHandle& nh, const std::string& trajectory_namespace) {
        // Geçici değişkenlerde yeni yörünge verilerini sakla
        TrajectoryData traj;
        nh.getParam("/" + trajectory_namespace + "/points_x", traj.points_x);
        nh.getParam("/" + trajectory_namespace + "/points_y", traj.points_y);
        nh.getParam("/" + trajectory_namespace + "/points_z", traj.points_z);
        nh.getParam("/" + trajectory_namespace + "/times", traj.times);
        nh.getParam("/" + trajectory_namespace + "/ros_rate", traj.ros_rate);
        nh.getParam("/" + trajectory_namespace + "/method", traj.method);

        trajectories.push_back(traj);  // Yörüngeyi listeye ekle

        ROS_INFO("Loaded trajectory: %s", trajectory_namespace.c_str());
    }

private:
    struct TrajectoryData {
        std::vector<double> points_x;
        std::vector<double> points_y;
        std::vector<double> points_z;
        std::vector<double> times;
        double ros_rate;
        std::string method;
    };

    ros::ServiceServer service;
    std::vector<TrajectoryData> trajectories;
    size_t current_trajectory_index;  // Sıradaki yörüngeyi takip eden indeks
    double last_time_offset;          // Son yörüngenin son zaman değeri
    bool return_to_start;
    double return_duration;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_server");
    ros::NodeHandle nh;

    // WaypointServer sınıfını oluştur
    WaypointServer traj_server(nh);

    ros::spin();
    return 0;
}
