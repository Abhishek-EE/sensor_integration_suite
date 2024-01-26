#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>

class FrameBroadcasterNode : public rclcpp::Node {
public:
    FrameBroadcasterNode() : Node("frame_broadcaster_node") {
        this->declare_parameter<std::string>("frames_yaml", {});
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        load_and_broadcast_transforms();
    }

private:
    void load_and_broadcast_transforms() {
        auto frames_yaml = this->get_parameter("frames_yaml").as_string();
        YAML::Node transform_configs = YAML::LoadFile(frames_yaml)
        for (const auto& transform : transform_configs) {
            YAML::Node transform = YAML::Load(transform_param);

            geometry_msgs::msg::TransformStamped t;
            t.header.frame_id = transform["parent_frame"].as<std::string>();
            t.child_frame_id = transform["child_frame"].as<std::string>();
            t.header.stamp = this->get_clock()->now();
            
            auto translation = transform["translation"];
            t.transform.translation.x = translation[0].as<double>();
            t.transform.translation.y = translation[1].as<double>();
            t.transform.translation.z = translation[2].as<double>();

            auto rotation = transform["rotation"];
            tf2::Quaternion q;
            q.setRPY(rotation[0].as<double>(),rotation[1].as<double>(),rotation[2].as<double>());
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t);
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameBroadcasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
