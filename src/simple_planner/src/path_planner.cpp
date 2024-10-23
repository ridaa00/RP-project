#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "custom_path_finding.hpp"
#include <opencv2/opencv.hpp>

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : Node("path_planner") {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        start_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&PathPlanner::start_callback, this, std::placeholders::_1));

        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PathPlanner::goal_callback, this, std::placeholders::_1));

        start_received_ = false;
        goal_received_ = false;

        RCLCPP_INFO(this->get_logger(), "PathPlanner node has been initialized.");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;

    geometry_msgs::msg::PoseStamped start_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool start_received_;
    bool goal_received_;

    void start_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        start_pose_.header = msg->header;
        start_pose_.pose = msg->pose.pose;
        start_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Received start position.");

        try_compute_path();
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        goal_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Received goal position.");

        try_compute_path();
    }

    void try_compute_path() {
        if (start_received_ && goal_received_) {
            RCLCPP_INFO(this->get_logger(), "Computing path...");

            // Create a dummy distance map
            cv::Mat distance_map; // Use an empty matrix if not needed

            auto path_points = planner::find_path(distance_map, start_pose_, goal_pose_);

            if (path_points.empty()) {
                RCLCPP_WARN(this->get_logger(), "Path is empty. Generating a straight-line path for testing.");
                path_points = generate_test_path(start_pose_, goal_pose_);
            }

            publish_path(path_points);

            start_received_ = false;
            goal_received_ = false;
        }
    }

    void publish_path(const std::vector<geometry_msgs::msg::PoseStamped>& path_points) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        path_msg.poses = path_points;

        path_publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu points.", path_points.size());
    }

    std::vector<geometry_msgs::msg::PoseStamped> generate_test_path(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal
    ) {
        std::vector<geometry_msgs::msg::PoseStamped> path_points;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.orientation.w = 1.0;

        int num_points = 20;
        for (int i = 0; i <= num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            pose.header.stamp = this->now();
            pose.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
            pose.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
            path_points.push_back(pose);
        }

        return path_points;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}