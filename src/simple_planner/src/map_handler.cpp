#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "obstacle_distance_map.hpp"

class MapHandler : public rclcpp::Node {
public:
    MapHandler() : Node("map_handler") {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapHandler::map_callback, this, std::placeholders::_1));
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
        auto distance_map = planner::compute_obstacle_distance_map(*map_msg);
        RCLCPP_INFO(this->get_logger(), "Distance map computed for a map of size %d x %d", map_msg->info.width, map_msg->info.height);
        // Handle the distance map as needed
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
