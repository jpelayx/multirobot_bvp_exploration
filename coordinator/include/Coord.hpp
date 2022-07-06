#ifndef COORD_H
#define COORD_H

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include <vector>
#include <tuple>
#include <mutex>
#include <algorithm>

#include "Map.hpp"

struct CoordTarget
{
    std::string ns; // namespace
    ros::Publisher pub;
    geometry_msgs::Point objective;
    bool is_merged;
};


class Coord 
{
    public:
        Coord(ros::NodeHandle);

        /* Set local map names 
         * (use namespaces only) */
        void add_maps(std::vector<std::string> name_spaces);

        void run();

    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;

        std::vector<CoordTarget*> map_list;
        Map* global_map;

        // checks which maps are already merged
        void update_merged_maps();
        int merged_maps_count;

        // assigns objectives to maps
        void assign_frontiers(std::vector<int> frontiers);

        // publishes each map's /objective
        void publish_objectives();

        // find robot's position 
        geometry_msgs::Point get_position(CoordTarget* t);

        // get the n closest frontiers to point p in fs
        std::vector<int> find_closest_frontiers(geometry_msgs::Point p, std::vector<int> fs, int n);
        bool sort_euclidian(int i, int j);
};

std::vector<std::string> parse_names(std::string);

#endif