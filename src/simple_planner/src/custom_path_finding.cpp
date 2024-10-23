#include "custom_path_finding.hpp"
#include <queue>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"



namespace planner {
    std::vector<geometry_msgs::msg::PoseStamped> find_path(
        const cv::Mat& distance_map,
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
            pose.header.stamp = start.header.stamp;
            pose.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
            pose.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
            path_points.push_back(pose);
        }

        return path_points;
    }
}

