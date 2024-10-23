#include "obstacle_distance_map.hpp"

namespace planner {

    cv::Mat compute_obstacle_distance_map(const nav_msgs::msg::OccupancyGrid& map) {
        cv::Mat distance_map(map.info.height, map.info.width, CV_32S, cv::Scalar(255));
        std::queue<std::pair<int, int>*> frontier;
        std::vector<std::vector<bool>> visited(map.info.height, std::vector<bool>(map.info.width, false));
        
        // Initialize the frontier with the obstacles from the map
        for(unsigned int col = 0; col < map.info.width; col++) {
            for(unsigned int row = 0; row < map.info.height; row++) {
                int index = (map.info.height - row - 1) * map.info.width + col;
                if(map.data[index] == 100) { // Occupied cells
                    distance_map.at<int>(row, col) = 0;
                    frontier.push(new std::pair<int, int>(row, col));
                    visited[row][col] = true;
                }
            }
        }

        // Perform BFS to compute the shortest distances
        while(!frontier.empty()) {
            std::pair<int, int>* current = frontier.front();
            frontier.pop();

            for(int dy = -1; dy <= 1; dy++) {
                for(int dx = -1; dx <= 1; dx++) {
                    if(dx == 0 && dy == 0) continue;
                    int new_row = current->first + dy;
                    int new_col = current->second + dx;
                    if(new_row >= 0 && new_row < map.info.height && new_col >= 0 && new_col < map.info.width && !visited[new_row][new_col]) {
                        distance_map.at<int>(new_row, new_col) = distance_map.at<int>(current->first, current->second) + 1;
                        frontier.push(new std::pair<int, int>(new_row, new_col));
                        visited[new_row][new_col] = true;
                    }
                }
            }
        }

        return distance_map;
    }

}

