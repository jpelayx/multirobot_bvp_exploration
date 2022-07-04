#ifndef #COORD_MAP_H
#define #COORD_MAP_H

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

        void map_callback(const nav_msgs::OccupancyGrid& m)
        bool updated();
        
        std::vector<geometry_msgs::Point> get_frontiers_centroids();

    private:
        nav_msgs::OccupancyGrid map;
        std::mutex map_mtx;
        char map_labels[];

        std::vector<std::vector<geometry_msgs::Point>> find_frontiers();
        geometry_msgs::Point find_centroid(std::vector<geometry_msgs::Point> frontier);
}

#endif