#ifndef CUSTOM_PATH_FINDING_HPP
#define CUSTOM_PATH_FINDING_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace planner {
    std::vector<geometry_msgs::msg::PoseStamped> find_path(
        const cv::Mat& distance_map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal
    );
}

#endif // CUSTOM_PATH_FINDING_HPP
