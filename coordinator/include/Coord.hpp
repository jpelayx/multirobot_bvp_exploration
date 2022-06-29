#ifndef COORD_H
#define COORD_H

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <vector>
#include <mutex>

class Coord 
{
    public:
        Coord(ros::NodeHandle);

        /* Set local map names 
         * (use namespaces only) */
        void add_maps(std::vector<std::string> name_spaces);

        void run();
        
        void global_map_callback(const nav_msgs::OccupancyGrid& map);

    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;

        std::vector<ros::Publisher*> objective_pubs;
        ros::Subscriber global_map_sub;
        std::vector<std::string> map_list;
        std::vector<std::string> merged_maps;

        std::mutex map_mtx;
        nav_msgs::OccupancyGrid map; 

        void update_merged_maps();
};

#endif