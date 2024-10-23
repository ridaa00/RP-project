#ifndef OBSTACLE_DISTANCE_MAP_HPP
#define OBSTACLE_DISTANCE_MAP_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace planner {
    cv::Mat compute_obstacle_distance_map(const nav_msgs::msg::OccupancyGrid& map);
}

#endif  // OBSTACLE_DISTANCE_MAP_HPP
