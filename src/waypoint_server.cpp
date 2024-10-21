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

        // trajectory_manager yapılandırmasını oku
        std::string active_trajectory;
        nh.getParam("trajectory_manager/active_trajectory", active_trajectory);

        // available_trajectories parametresini oku
        XmlRpc::XmlRpcValue available_trajectories;
        if (nh.getParam("trajectory_manager/available_trajectories", available_trajectories)) {
            ROS_ASSERT(available_trajectories.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int i = 0; i < available_trajectories.size(); ++i) {
                ROS_ASSERT(available_trajectories[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                
                std::string name = static_cast<std::string>(available_trajectories[i]["name"]);
                std::string file = static_cast<std::string>(available_trajectories[i]["file"]);

                if (name == active_trajectory) {
                    loadTrajectory(nh, name);  // Seçilen trajectory dosyasını yükle
                    break;
                }
            }
        } else {
            ROS_ERROR("Failed to get available_trajectories from trajectory_manager");
        }

        // return_to_start ve return_duration parametrelerini trajectory_manager'dan kontrol et
        nh.param("trajectory_manager/return_to_start", return_to_start, false);  // Varsayılan false
        nh.param("trajectory_manager/return_duration", return_duration, 0.0);   // Varsayılan 0.0
    }

    bool getTrajectoryCallback(quadcopter_control::WaypointService::Request &req,
                               quadcopter_control::WaypointService::Response &res) {
        // Gelen talebe yanıt olarak yörünge verilerini döndür
        res.method = trajectory_method;
        res.ros_rate = ros_rate;
        res.return_to_start = return_to_start;
        res.return_duration = return_duration;
        res.points_x = points_x;
        res.points_y = points_y;
        res.points_z = points_z;
        res.times = times;

        ROS_INFO("Trajectory data sent to client.");
        return true;
    }

    void loadTrajectory(ros::NodeHandle& nh, const std::string& trajectory_namespace) {
        // Yörünge verilerini seçilen isim alanından yükle
        std::string trajectory_ns = "/" + trajectory_namespace;  // İsim alanını başına ekleyelim
        nh.getParam(trajectory_ns + "/points_x", points_x);
        nh.getParam(trajectory_ns + "/points_y", points_y);
        nh.getParam(trajectory_ns + "/points_z", points_z);
        nh.getParam(trajectory_ns + "/times", times);
        nh.getParam(trajectory_ns + "/ros_rate", ros_rate);
        nh.getParam(trajectory_ns + "/method", trajectory_method);  // Trajectory generation yöntemi

        // return_to_start ve return_duration trajectory_manager seviyesinde kontrol edileceği için burada tekrar okunmasına gerek yok
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
