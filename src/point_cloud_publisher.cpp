#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include <random>

class SlidingPointCloudPublisher : public rclcpp::Node
{
public:
    SlidingPointCloudPublisher() : Node("sliding_point_cloud_publisher"), offset_(0.0f), direction_(1)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SlidingPointCloudPublisher::publishPointCloud, this));
    }

private:
    void publishPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        const float x_distance = 0.8f;
        const float z_range = 1.0f;
        const int points_per_franja = 2000;

        const float franja_width = 0.5f;
        const float gap = franja_width / 2.0f;

        std::random_device rd;
        std::mt19937 gen(rd());

        for (int i = 0; i < 3; ++i)
        {
            float y_center = offset_ - 1.5f + (i * (franja_width + gap)) + (franja_width / 2);
            float y_min = y_center - (franja_width / 2);
            float y_max = y_center + (franja_width / 2);

            std::uniform_real_distribution<float> dist_y(y_min, y_max);
            std::uniform_real_distribution<float> dist_z(-z_range, z_range);

            for (int j = 0; j < points_per_franja; ++j)
            {
                pcl::PointXYZ point;
                point.x = x_distance;
                point.y = dist_y(gen);
                point.z = dist_z(gen);
                cloud->points.push_back(point);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "camera_link";
        cloud_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(cloud_msg);

        RCLCPP_INFO(this->get_logger(), "Published sliding point cloud with %zu points", cloud->points.size());


        updateOffset();
    }

    void updateOffset()
    {
        const float step = 0.05f;
        const float max_offset = 0.6f;

        offset_ += direction_ * step;

        if (offset_ >= max_offset || offset_ <= -max_offset)
        {
            direction_ *= -1;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float offset_;
    int direction_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlidingPointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}
