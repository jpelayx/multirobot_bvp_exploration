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
        
        void global_map_callback(const nav_msgs::OccupancyGrid& map);

    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;

        ros::Subscriber global_map_sub;
        std::vector<CoordTarget*> map_list;

        std::mutex map_mtx;
        nav_msgs::OccupancyGrid map; 

        // checks which maps are already merged
        void update_merged_maps();
        int merged_maps_count;
        bool map_updated;

        // find all frontiers' centroids in last received map 
        std::vector<geometry_msgs::Point> find_frontiers();

        // assigns objectives to maps
        void assign_frontiers(std::vector<geometry_msgs::Point> frontiers);

        void publish_objectives();


};

#endif