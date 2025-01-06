#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class GroundSegmentation : public rclcpp::Node {
public:
    GroundSegmentation() : Node("cloud_separation") {
        this->declare_parameter<double>("near_distance", 0.5);
        this->declare_parameter<double>("middle_distance", 1.5);

        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points",
            10,
            std::bind(&GroundSegmentation::point_cloud_cb, this, std::placeholders::_1)
        );

        near_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("near_points", 10);
        middle_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("middle_points", 10);
        far_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("far_points", 10);
    }

private:
    void point_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *cloud);

        double near_distance, middle_distance;
        this->get_parameter("near_distance", near_distance);
        this->get_parameter("middle_distance", middle_distance);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr middle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (const auto &point : cloud->points) {
            double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance <= near_distance) {
                near_cloud->points.push_back(point);
            } else if (distance <= middle_distance) {
                middle_cloud->points.push_back(point);
            } else {
                far_cloud->points.push_back(point);
            }
        }

        sensor_msgs::msg::PointCloud2 near_msg, middle_msg, far_msg;
        pcl::toROSMsg(*near_cloud, near_msg);
        pcl::toROSMsg(*middle_cloud, middle_msg);
        pcl::toROSMsg(*far_cloud, far_msg);

        near_msg.header = msg->header;
        middle_msg.header = msg->header;
        far_msg.header = msg->header;

        near_pointcloud_publisher_->publish(near_msg);
        middle_pointcloud_publisher_->publish(middle_msg);
        far_pointcloud_publisher_->publish(far_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr near_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr middle_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr far_pointcloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundSegmentation>());
    rclcpp::shutdown();
    return 0;
}
