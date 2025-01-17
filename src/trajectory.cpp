#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <random>

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Trajectory Node");

        // Inicialización de la trayectoria base
        base_trajectory_ = {
            {0.0, 0.5, 0.0},
            {0.5, 1.3, 0.2},
            {1.0, 2.2, 0.5},
            {2.0, 2.5, 0.9},
            {3.0, 2.8, 1.3},
            {4.0, 3.0, 1.8},
            {5.0, 2.5, 2.0},
            {5.5, 2.0, 2.0},
            {6.0, 1.0, 1.7},
            {6.5, 0.0, 1.2},
            {7.0, -1.0, 1.1},
            {7.5, -2.0, 1.0},
            {8.0, -2.5, 0.9},
            {9.0, -2.5, 0.5},
            {10.0, -2.0, 0.2},
            {10.5, -1.0, 0.1}
        };

        for (auto &point : base_trajectory_)
        {
            point[0] /= 10.0;
            point[1] /= 10.0;
            point[2] /= 10.0;
        }

        // Publicador de la trayectoria
        trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_points", 10);

        // Generar 20 trayectorias
        generate_trajectories(1);

        // Llama a la función run en un hilo separado
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TrajectoryNode::publish_trajectories, this));
    }

private:
    void generate_trajectories(size_t num_trajectories)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(-0.05, 0.05);

        for (size_t i = 0; i < num_trajectories; ++i)
        {
            std::vector<std::array<double, 3>> trajectory;
            for (const auto &point : base_trajectory_)
            {
                trajectory.push_back({
                    point[0] + dist(gen),
                    point[1] + dist(gen),
                    point[2] + dist(gen)
                });
            }
            trajectories_.push_back(trajectory);
        }
    }

    void publish_trajectories()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        for (size_t t = 0; t < trajectories_.size(); ++t)
        {
            const auto &trajectory = trajectories_[t];

            for (size_t i = 0; i < trajectory.size(); ++i)
            {
                auto marker = visualization_msgs::msg::Marker();
                marker.header.frame_id = "camera_link";
                marker.header.stamp = this->now();
                marker.id = static_cast<int>(t * base_trajectory_.size() + i); // Unique ID
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = trajectory[i][0];
                marker.pose.position.y = trajectory[i][1];
                marker.pose.position.z = trajectory[i][2];
                marker.scale.x = 0.02;
                marker.scale.y = 0.02;
                marker.scale.z = 0.02;

                // Color único para cada trayectoria
                marker.color.a = 1.0;
                marker.color.r = static_cast<float>(t) / trajectories_.size();
                marker.color.g = 1.0 - static_cast<float>(t) / trajectories_.size();
                marker.color.b = 0.5;

                marker_array.markers.push_back(marker);
            }
        }

        trajectory_publisher_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
    std::vector<std::array<double, 3>> base_trajectory_;
    std::vector<std::vector<std::array<double, 3>>> trajectories_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
