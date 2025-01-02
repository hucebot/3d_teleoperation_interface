#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/crop_box.h>
#include <Eigen/Core>

class TrajectoryVerify : public rclcpp::Node
{
public:
    TrajectoryVerify() : Node("trajectory_verify"), margin_width_(0.05), margin_height_(0.05), margin_depth_(1.5)
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
    std::vector<Point3D> rotated_trajectory_points_;
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
        rotated_trajectory_points_.clear();

        Eigen::Matrix3f rotation_y;
        rotation_y << 0, 0, 1,
                      0, 1, 0,
                      1, 0, 0;

        Eigen::Matrix3f rotation_z;
        rotation_z << 0, -1, 0,
                      -1, 0, 0,
                      0, 0, 1;

        for (const auto &marker : msg->markers)
        {
            Point3D original_point = {marker.pose.position.x, marker.pose.position.y, marker.pose.position.z};
            trajectory_points_.push_back(original_point);

            Eigen::Vector3f point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);

            point = rotation_y * point;
            point = rotation_z * point;

            Point3D rotated_point = {point.x(), point.y(), point.z()};
            rotated_trajectory_points_.push_back(rotated_point);
        }

        publishRotatedPoints();
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        point_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *point_cloud_);
        verifyLineOfSight();
    }

    void verifyLineOfSight()
    {
        if (!point_cloud_ || rotated_trajectory_points_.empty())
            return;

        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto box_visualization_array = visualization_msgs::msg::MarkerArray();
        auto obstacle_markers = visualization_msgs::msg::MarkerArray();

        for (size_t i = 0; i < rotated_trajectory_points_.size(); ++i)
        {
            const auto &rotated_point = rotated_trajectory_points_[i];
            const auto &original_point = trajectory_points_[i];

            bool obstructed = checkObstruction(rotated_point, obstacle_markers);

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
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            }

            geometry_msgs::msg::Point start_point;
            start_point.x = origin_.x;
            start_point.y = origin_.y;
            start_point.z = origin_.z;

            geometry_msgs::msg::Point end_point;
            end_point.x = original_point.x;
            end_point.y = original_point.y;
            end_point.z = original_point.z;

            marker.points.push_back(start_point);
            marker.points.push_back(end_point);
            marker_array.markers.push_back(marker);

            auto box_marker = createBoxMarker(i, original_point);
            box_visualization_array.markers.push_back(box_marker);
        }

        trajectory_verify_publisher_->publish(marker_array);
        box_visualization_publisher_->publish(box_visualization_array);

        //box_visualization_publisher_->publish(obstacle_markers);
    }

    bool checkObstruction(const Point3D &target_point, visualization_msgs::msg::MarkerArray &obstacle_markers)
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

        size_t marker_id_offset = 1000;
        for (size_t i = 0; i < cropped_cloud->points.size(); ++i)
        {
            const auto &obstacle_point = cropped_cloud->points[i];

            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "camera_link";
            marker.header.stamp = this->now();
            marker.ns = "obstacle_points";
            marker.id = marker_id_offset + static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Posición del marcador
            marker.pose.position.x = obstacle_point.x;
            marker.pose.position.y = obstacle_point.y;
            marker.pose.position.z = obstacle_point.z;

            // Escala del marcador
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            // Color del marcador (por ejemplo, rojo)
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

            obstacle_markers.markers.push_back(marker);
        }

        return !cropped_cloud->empty();
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

        marker.pose.position.x = (origin_.x + point.x) / 2.0;
        marker.pose.position.y = (origin_.y + point.y) / 2.0;
        marker.pose.position.z = (origin_.z + point.z) / 2.0;

        Eigen::Vector3f direction_vector(point.x - origin_.x, point.y - origin_.y, point.z - origin_.z);
        direction_vector.normalize();

        Eigen::Vector3f default_axis(1.0, 0.0, 0.0);
        Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(default_axis, direction_vector);

        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        marker.scale.x = direction_vector.norm();
        marker.scale.y = margin_width_;
        marker.scale.z = margin_height_;

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.6;

        return marker;
    }

    void publishRotatedPoints()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        for (size_t i = 0; i < rotated_trajectory_points_.size(); ++i)
        {
            const auto &point = rotated_trajectory_points_[i];

            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "camera_link";
            marker.header.stamp = this->now();
            marker.ns = "rotated_points";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        trajectory_verify_publisher_->publish(marker_array);
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
