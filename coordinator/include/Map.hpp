#ifndef COORD_MAP_H
#define COORD_MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <vector>
#include <mutex>

#define VISITED 1
#define NOT_VISITED 0

class Map
{
    public:
        Map(ros::NodeHandle, std::string topic);

        void map_callback(const nav_msgs::OccupancyGrid& m);
        bool updated();
        void reset_updated();
        
        std::vector<geometry_msgs::Point> get_frontiers_centroids();

    private:
        ros::Subscriber map_sub;
        nav_msgs::OccupancyGrid map;
        std::mutex map_mtx;
        std::vector<int8_t> map_labels;
        bool map_updated;

        std::vector<std::vector<geometry_msgs::Point>> find_frontiers();
        bool is_frontier(int pos);
        std::vector<geometry_msgs::Point> extract_frontier(int pos);
        geometry_msgs::Point find_centroid(std::vector<geometry_msgs::Point> frontier);
};

#endif