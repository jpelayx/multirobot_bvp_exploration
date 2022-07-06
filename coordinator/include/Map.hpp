#ifndef COORD_MAP_H
#define COORD_MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <vector>
#include <mutex>
#include <math.h>

#define VISITED 1
#define NOT_VISITED 0

#define MIN_FRONTIER_SIZE 2

#define FREE 30
#define OCCUPIED 70
#define UNEXPLORED -1

class Map
{
    public:
        Map(ros::NodeHandle, std::string topic);
        
        void map_callback(const nav_msgs::OccupancyGrid& m);
        bool updated();
        void reset_updated();
        
        // returns a list with the closest frontier cell to each frontier's centroid
        std::vector<int> get_frontiers_centroids();

        // euclidian distance between two positions in OccupancyGrid
        float get_euclidian_distance(int i, int j);
        /* given a OccupancyGrid position,
         * get word position */
        geometry_msgs::Point get_point(int pos);
        int get_pos(geometry_msgs::Point p);

    private:
        ros::Subscriber map_sub;
        nav_msgs::OccupancyGrid map;
        std::mutex map_mtx;
        std::vector<int8_t> map_labels;
        bool map_updated;

        /* OccupancyGrid data is a 1D vector
         * The following functions facilitate 2D matrix-like access
         */

        int get_x(int pos);
        int get_y(int pos);
        /* given a x,y 2D matrix position, 
         * get 1D vector position */
        int get_pos(int x, int y);

        // get value of map in position x, y
        int8_t get_cell_value(int x, int y);
        // get value of map in vector position pos
        int8_t get_cell_value(int pos);

        // get map metadata
        nav_msgs::MapMetaData get_info();

        std::vector<std::vector<int>> find_frontiers();
        /* - true case cell in given position is free and 
         * at least one of it's adjacent cells are unexplored 
         * - false otherwise.*/
        bool is_frontier(int pos);
        /* - true if cell's value is -1 or 
         * between free (FREE) and  occupied (OCCUPIED) thresholds. 
         * - false otherwise */
        bool is_unexplored(int x, int y);

        /* get every frontier cell adjacent to cell at pos 
           considering pos is the upmost and leftmost cell in the frontier */
        std::vector<int> extract_frontier(int pos);
        std::vector<int> adjacent_pos(int pos);
        int find_centroid(std::vector<int> frontier);
};

#endif