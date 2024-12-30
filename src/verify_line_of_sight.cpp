#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>

class TrajectoryVerify : public rclcpp::Node
{
public:
    TrajectoryVerify() : Node("trajectory_verify"), margin_width_(1.0), margin_height_(1.0), margin_depth_(1.0)
    {
        origin_ = {0.0, 0.0, 0.0};

        trajectory_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/trajectory_points", 10,
            std::bind(&TrajectoryVerify::trajectoryCallback, this, std::placeholders::_1));

        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&TrajectoryVerify::pointCloudCallback, this, std::placeholders::_1));

        trajectory_verify_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trajectory_verify", 10);

        box_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_boxes", 10);
    }

private:
    struct Point3D
    {
        float x, y, z;
    };

    Point3D origin_;
    std::vector<Point3D> trajectory_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    float margin_width_;
    float margin_height_;
    float margin_depth_;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_verify_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_visualization_publisher_;

    void trajectoryCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        trajectory_points_.clear();
        for (const auto &marker : msg->markers)
        {
            Point3D point = {marker.pose.position.x, marker.pose.position.y, marker.pose.position.z};
            trajectory_points_.push_back(point);
        }
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        point_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *point_cloud_);
        verifyLineOfSight();
    }

    void verifyLineOfSight()
    {
        if (!point_cloud_ || trajectory_points_.empty())
            return;

        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto box_visualization_array = visualization_msgs::msg::MarkerArray();

        for (size_t i = 0; i < trajectory_points_.size(); ++i)
        {
            const auto &point = trajectory_points_[i];
            bool obstructed = checkObstruction(point);

            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "camera_link";
            marker.header.stamp = this->now();
            marker.ns = "trajectory_lines";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.02;

            if (obstructed)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                RCLCPP_INFO(this->get_logger(), "Point %zu is obstructed", i);
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                RCLCPP_INFO(this->get_logger(), "Point %zu is not obstructed", i);
            }

            geometry_msgs::msg::Point start_point;
            start_point.x = origin_.x;
            start_point.y = origin_.y;
            start_point.z = origin_.z;

            geometry_msgs::msg::Point end_point;
            end_point.x = point.x;
            end_point.y = point.y;
            end_point.z = point.z;

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);
            marker_array.markers.push_back(marker);

            // Add box visualization
            auto box_marker = createBoxMarker(i, point);
            box_visualization_array.markers.push_back(box_marker);
        }

        trajectory_verify_publisher_->publish(marker_array);
        box_visualization_publisher_->publish(box_visualization_array);
    }

    visualization_msgs::msg::Marker createBoxMarker(size_t id, const Point3D &point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "camera_link";
        marker.header.stamp = this->now();
        marker.ns = "trajectory_boxes";
        marker.id = static_cast<int>(id);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the center of the box
        marker.pose.position.x = (origin_.x + point.x) / 2.0;
        marker.pose.position.y = (origin_.y + point.y) / 2.0;
        marker.pose.position.z = (origin_.z + point.z) / 2.0;

        // Set box orientation (aligned with the axis)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the box
        marker.scale.x = margin_width_;
        marker.scale.y = margin_height_;
        marker.scale.z = std::sqrt(
            std::pow(point.x - origin_.x, 2) +
            std::pow(point.y - origin_.y, 2) +
            std::pow(point.z - origin_.z, 2));

        // Set the color with alpha for visualization
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.6; // Transparency

        return marker;
    }

    bool checkObstruction(const Point3D &target_point)
    {
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setMin(Eigen::Vector4f(
            origin_.x - margin_width_ / 2.0,
            origin_.y - margin_height_ / 2.0,
            origin_.z,
            1.0));
        crop_box.setMax(Eigen::Vector4f(
            target_point.x + margin_width_ / 2.0,
            target_point.y + margin_height_ / 2.0,
            target_point.z + margin_depth_,
            1.0));
        crop_box.setInputCloud(point_cloud_);

        auto cropped_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        crop_box.filter(*cropped_cloud);

        return !cropped_cloud->empty();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryVerify>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
