#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class PointCloudFusionNode : public rclcpp::Node {
public:
    PointCloudFusionNode() : Node("pointcloud_fusion_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Initialize subscribers to the point cloud topics

        this->declare_parameter<std::string>("zed_cloud_topic_name", "/zed/point_cloud");
        this->declare_parameter<std::string>("vertical_pc_topic_name", "/lidar/points_vt");
        this->declare_parameter<std::string>("horizontal_pc_topic_name", "/lidar/points_hz");
        std::string zed_cloud_topic_name;
        std::string vertical_pc_topic_name;
        std::string horizontal_pc_topic_name;
        
        // Retrieve the parameters
        this->get_parameter("zed_cloud_topic_name", zed_cloud_topic_name);
        this->get_parameter("vertical_pc_topic_name", vertical_pc_topic_name);
        this->get_parameter("horizontal_pc_topic_name", horizontal_pc_topic_name);
        
        //Subscribers
        zed_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            zed_cloud_topic_name, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFusionNode::pointCloudCallback, this, std::placeholders::_1, "zed_left_camera_optical_frame"));

        lidar_hz_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            horizontal_pc_topic_name, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFusionNode::pointCloudCallback, this, std::placeholders::_1, "lidar_frame_hz"));

        lidar_vt_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            vertical_pc_topic_name, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFusionNode::pointCloudCallback, this, std::placeholders::_1, "lidar_frame_vt"));

        // Publisher for the combined point cloud
        fused_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged/point_cloud", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string& frame_id) {
        // Transform the point cloud to the common frame
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        if (!transformPointCloud(msg, transformed_cloud, frame_id)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform point cloud from frame: %s", frame_id.c_str());
            return;
        }

        // Convert to PCL point cloud and merge
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(transformed_cloud, *current_cloud);
        fused_cloud_ += *current_cloud;

        // Publish the fused point cloud
        publishFusedCloud();
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
        pcl::toROSMsg(fused_cloud_, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "base_link";
        fused_pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr zed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_hz_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_vt_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
    pcl::PointCloud<pcl::PointXYZ> fused_cloud_;
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