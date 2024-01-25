#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/TransformStamped.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>

class FrameBroadcasterNode : public rclcpp::Node {
public:
    FrameBroadcasterNode() : Node("frame_broadcaster_node") {
        this->declare_parameter<std::vector<std::string>>("transforms", {});
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        load_and_broadcast_transforms();
    }

private:
    void load_and_broadcast_transforms() {
        auto transforms_param = this->get_parameter("transforms").as_string_array();
        for (const auto& transform_param : transforms_param) {
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
            q.setRPY(rotaion[0].as<double>(),rotaion[1].as<double>(),rotaion[2].as<double>());
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
