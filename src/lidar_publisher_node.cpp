#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_integration_suite/lidarkit.hpp" // Adjust the include path based on your setup
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>


class LidarPublisherNode : public rclcpp::Node {
public:
    LidarPublisherNode() : Node("lidar_publisher_node") {
        // Existing parameter declarations
        this->declare_parameter<std::string>("lidar_uri", "/dev/ttyUSB0");
        this->declare_parameter<std::string>("topic_name", "/lidar/points");
        
        // Declare the frame_id parameter
        this->declare_parameter<std::string>("frame_id", "lidar_frame");

        std::string lidar_uri;
        std::string topic_name;
        
        // Retrieve the parameters
        this->get_parameter("lidar_uri", lidar_uri);
        this->get_parameter("topic_name", topic_name);
        this->get_parameter("frame_id", frame_id);
        // Initialize the LiDAR with the URI
        lidar.set_dev_uri(lidar_uri);
        lidar.start();

        // Create a publisher on the specified topic
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);

        // Use a timer to fetch and publish data at a regular interval
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Adjust the rate as needed
            std::bind(&LidarPublisherNode::publish_points, this));

    }
    ~LidarPublisherNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down LidarPublisherNode");
        lidar.stop(); // Stop the lidar before exiting
        // Wait for any other cleanup if necessary
        RCLCPP_INFO(this->get_logger(), "LiDAR stopped successfully");

    }

private:
    void publish_points() {
        RCLCPP_DEBUG(this->get_logger(), "Publishing points start");
        
        if (!rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "ROS context not valid, exiting publish_points");
            return; // Early exit if ROS context is not valid (e.g., during shutdown)
        }

        // Existing logic to fetch and publish points...
        auto points = lidar.get_points();
        if (points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No points received from LiDAR");
        } else {
            auto msg = convert_to_point_cloud2(points);
            publisher_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Published %zu points", points.size());
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate work (consider removing or adjusting for real use cases)

        RCLCPP_DEBUG(this->get_logger(), "Publishing points end");

    }

sensor_msgs::msg::PointCloud2 convert_to_point_cloud2(const std::vector<LidarPoint>& points) {
    sensor_msgs::msg::PointCloud2 msg;

    // Set the header of the PointCloud2 message
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = this->frame_id; 

    // Define the data structure of the PointCloud2 message
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    // Calculate point step and row step
    msg.point_step = 12; // 4 bytes each for x, y, z
    msg.row_step = msg.point_step * points.size();

    // Allocate space for all points
    msg.data.resize(msg.row_step);

    // Iterate over the points and populate the PointCloud2 message
    unsigned char* ptr = msg.data.data();
    for (const auto& point : points) {
        memcpy(ptr, &point.x, sizeof(float));
        ptr += sizeof(float);
        memcpy(ptr, &point.y, sizeof(float));
        ptr += sizeof(float);
        float z = 0.0f; // Assuming z is zero
        memcpy(ptr, &z, sizeof(float));
        ptr += sizeof(float);
    }

    // Set other necessary fields
    msg.height = 1; // Unordered point cloud
    msg.width = points.size();
    msg.is_dense = true; // Assuming no invalid (NaN, Inf) points
    return msg;
}

    LidarKit lidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}