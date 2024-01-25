#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sl/Camera.hpp>

// Replace with any other required headers
class ZedMiniPointCloudPublisher : public rclcpp::Node {
public:
    ZedMiniPointCloudPublisher() : Node("zed_mini_point_cloud_publisher") {
        // Initialize ZED camera
        sl::InitParameters init_params;
        init_params.camera_resolution = sl::RESOLUTION::HD720; // Example resolution
        init_params.depth_mode = sl::DEPTH_MODE::ULTRA; // Example depth mode

         // Declare the frame_id parameter
        this->declare_parameter<std::string>("frame_id", "zed_mini");
        std::string frame_id;
        
        this->get_parameter("frame_id", frame_id);

        zed.open(init_params);
        if (!zed.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ZED camera");
        }

        // Create a publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/zed_mini/points", 10);

        // Start a timer to capture and publish point cloud data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Adjust as needed
            std::bind(&ZedMiniPointCloudPublisher::publish_point_cloud, this));
    }

private:
    void publish_point_cloud() {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // Retrieve point cloud from ZED camera
            sl::Mat point_cloud;
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            // Convert the ZED point cloud to ROS message and publish
            auto ros_point_cloud_msg = convert_to_ros_point_cloud2(point_cloud);
            publisher_->publish(ros_point_cloud_msg);
        }
    }

    sensor_msgs::msg::PointCloud2 convert_to_ros_point_cloud2(const sl::Mat& zed_point_cloud) {
        sensor_msgs::msg::PointCloud2 msg;

        // Set the header
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "frame_id"; // Set the appropriate frame ID

        // Define the point fields
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

        // Set the point step and row step
        msg.point_step = 12; // Assuming each point consists of three float32s (x, y, z)
        msg.row_step = msg.point_step * zed_point_cloud.getResolution().area();

        // Resize the message data buffer
        msg.data.resize(zed_point_cloud.getResolution().area() * msg.point_step);

        // Copy data from ZED point cloud to ROS message
        float* data_ptr = reinterpret_cast<float*>(msg.data.data());
        int index = 0;
        for (size_t y = 0; y < zed_point_cloud.getHeight(); ++y) {
            for (size_t x = 0; x < zed_point_cloud.getWidth(); ++x, index += 3) {
                sl::float4 point;
                zed_point_cloud.getValue(x, y, &point);

                // Copy (x, y, z) coordinates
                std::memcpy(&data_ptr[index], &point.x, sizeof(float));
                std::memcpy(&data_ptr[index + 1], &point.y, sizeof(float));
                std::memcpy(&data_ptr[index + 2], &point.z, sizeof(float));
            }
        }

        // Set other necessary fields
        msg.height = 1; // Unordered point cloud
        msg.width = zed_point_cloud.getResolution().area();
        msg.is_dense = false; // Assuming the point cloud may have invalid points

        return msg;
    }

    sl::Camera zed;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedMiniPointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
