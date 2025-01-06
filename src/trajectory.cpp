#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Trajectory Node");

        // Inicialización de la trayectoria
        trajectory_ = {
            {0.0, 0.5, 0.0},
            {0.5, 1.3, 0.0},
            {1.0, 2.2, 0.0},
            {2.0, 2.5, 0.0},
            {3.0, 2.8, 0.0},
            {4.0, 3.0, 0.0},
            {5.0, 2.5, 0.0},
            {5.5, 2.0, 0.0},
            {6.0, 1.0, 0.0},
            {6.5, 0.0, 0.0},
            {7.0, -1.0, 0.0},
            {7.5, -2.0, 0.0},
            {8.0, -2.5, 0.0},
            {9.0, -2.5, 0.0},
            {10.0, -2.0, 0.0},
            {10.5, -1.0, 0.0}
        };

        // Publicador de la trayectoria
        trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_points", 10);

        // Llama a la función run en un hilo separado
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryNode::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        for (size_t i = 0; i < trajectory_.size(); ++i)
        {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "camera_link";
            marker.header.stamp = this->now();
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = trajectory_[i][0];
            marker.pose.position.y = trajectory_[i][1];
            marker.pose.position.z = trajectory_[i][2];
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Opacidad
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }

        trajectory_publisher_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
    std::vector<std::array<double, 3>> trajectory_;
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
