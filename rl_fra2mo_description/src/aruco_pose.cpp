#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ArucoTransformNode : public rclcpp::Node {
public:
    ArucoTransformNode() : Node("aruco_transform_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer) {
        // Sottoscrizione al topic ArUco
        aruco_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&ArucoTransformNode::arucoCallback, this, std::placeholders::_1));

        // Publisher per la posizione e orientamento trasformati
        transformed_marker_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco_marker_transformed", 10);
    }

private:
    void arucoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        try {
            // Trasforma la PoseStamped (completa di posizione e orientamento)
            geometry_msgs::msg::PoseStamped transformed_pose;
            transformed_pose.header = msg->header; // Mantieni il header originale

            // Trasforma la posizione e l'orientamento nel frame "map"
            tf_buffer.transform(*msg, transformed_pose, "map", tf2::durationFromSec(1.0));

            // Pubblica il risultato trasformato
            transformed_marker_pub->publish(transformed_pose);

            RCLCPP_INFO(this->get_logger(), "Marker Position and Orientation in map frame:");
            RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f]",
                        transformed_pose.pose.position.x,
                        transformed_pose.pose.position.y,
                        transformed_pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation: [%.2f, %.2f, %.2f, %.2f]",
                        transformed_pose.pose.orientation.x,
                        transformed_pose.pose.orientation.y,
                        transformed_pose.pose.orientation.z,
                        transformed_pose.pose.orientation.w);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to transform marker pose: %s", ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_marker_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
