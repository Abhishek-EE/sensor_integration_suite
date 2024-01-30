#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include<map>
#include<mutex>

class PointCloudFusionNode : public rclcpp::Node {
public:
    PointCloudFusionNode() : Node("pointcloud_fusion_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Initialize subscribers to the point cloud topics
        this->declare_parameter<std::string>("zed_cloud_topic_name", "/zed/point_cloud");
        this->declare_parameter<std::string>("vertical_pc_topic_name", "/lidar/points_vt");
        this->declare_parameter<std::string>("horizontal_pc_topic_name", "/lidar/points_hz");

        std::string zed_cloud_topic_name = this->get_parameter("zed_cloud_topic_name").as_string();
        std::string vertical_pc_topic_name = this->get_parameter("vertical_pc_topic_name").as_string();
        std::string horizontal_pc_topic_name = this->get_parameter("horizontal_pc_topic_name").as_string();

        // Subscribers
        zed_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            zed_cloud_topic_name, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFusionNode::pointCloudCallback, this, std::placeholders::_1));

        lidar_hz_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            horizontal_pc_topic_name, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFusionNode::pointCloudCallback, this, std::placeholders::_1));

        lidar_vt_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            vertical_pc_topic_name, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFusionNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for the combined point cloud
        fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged/point_cloud", 10);

        // Initialize the fused cloud as a shared pointer
        fused_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

private:

 void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Transform the point cloud to the common frame
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        if (!transformPointCloud(msg, transformed_cloud, msg->header.frame_id)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform point cloud from frame: %s", msg->header.frame_id.c_str());
            return;
        }

        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *current_cloud);

        // Store the transformed cloud
        point_clouds_[msg->header.frame_id] = current_cloud;
        clouds_received_[msg->header.frame_id] = true;

        // Check if all clouds have been received
        if (std::all_of(clouds_received_.begin(), clouds_received_.end(), [](const auto& pair) { return pair.second; })) {
            // All point clouds received, process and publish the fused point cloud
            publishFusedCloud();
            // Reset the flags for the next set of point clouds
            for (auto& received : clouds_received_) {
                received.second = false;
            }
        }
    }

    bool transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, 
                             sensor_msgs::msg::PointCloud2& transformed_cloud, 
                             const std::string& frame_id) {
        std::string error;
        if (!tf_buffer_.canTransform("base_link", frame_id, cloud->header.stamp, tf2::durationFromSec(1.0), &error)) {
            RCLCPP_ERROR(this->get_logger(), "Can't transform point cloud: %s", error.c_str());
            return false;
        }

        tf2::doTransform(*cloud, transformed_cloud, tf_buffer_.lookupTransform("base_link", frame_id, cloud->header.stamp));
        return true;
    }

    void publishFusedCloud() {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*fused_cloud_, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "base_link";
        fused_pub_->publish(output);
    }

    std::map<std::string, bool> clouds_received_;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds_;
    std::mutex mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr zed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_hz_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_vt_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> fused_cloud_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
