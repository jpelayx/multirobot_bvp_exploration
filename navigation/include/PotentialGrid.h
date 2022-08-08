#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <mutex>
#include <iostream>

#include "Robot.h"

#define OCC_TRESH 70
#define FREE_TRESH 30

#define NAVIGATOR_EXPAND_OBSTACLES_NUM 3

enum OccType {UNEXPLORED, OCCUPIED, OCCUPIED_EXP, FREE, OBJECTIVE};
enum FrontType {FRONTIER, MARKED_FRONTIER, NDA};

class Cell
{
    public:
        double potential;
        OccType occupation;
        FrontType frontier;

        Cell(int);
        void update(int);
        int show();
};

class Window
{
    public:
        Window(int x_min=0, int x_max=0, int y_min=0, int y_max=0)
            : x0{x_min}, xf{x_max}, y0{y_min}, yf{y_max} 
        {}
        int x0, 
            xf,
            y0,
            yf;
};

class PotentialGrid
{
    public:
        PotentialGrid();
        PotentialGrid(ros::NodeHandle*, std::string name_space);

        void initialize(ros::NodeHandle* nh, std::string ns);

        /* updates obstacles and resets objectives in area around robot_pos 
         * area is a rectangle determined by points 
         * (robot_pos.x - radius, robot_pos.y - radius) and (robot_pos.x + radius, robot_pos.y + radius)
         * if radius == -1 then the entire map area will be updated
         * 
         * area updated will become grid's active window (e.g. area where potential field will be calculated)
         */
        void update(const nav_msgs::OccupancyGrid::ConstPtr &m, geometry_msgs::Transform robot_pos, float radius = -1);
        
        // updates potential in active window 
        void update_potential();

        // set goals to frontiers' centroids in active window and resets old goals
        void set_local_goal();
        
        // set goal to point p and resets old goals in active window
        void set_goal(geometry_msgs::Point p);

        // returns normalized gradient descent in point t.translation
        std::vector<double> normalized_gradient(geometry_msgs::Transform t);
        


    private:
        ros::Publisher potential_pub;
        ros::Publisher vector_pub;
        ros::Publisher vector_field_pub;

        void publish_potential_field();
        void publish_vector(std::vector<double>, geometry_msgs::Transform);
        void publish_vector_field();
        
        std::vector<std::vector<Cell*>> grid;
        Window active_area;
        
        // metadata
        geometry_msgs::Point map0;
        int width, height;
        double resolution;

        std::vector<int> objectives;
        bool objectives_set;

        // ros params 
        int param_pub_pot,
            param_pub_gradient_vec,
            param_pub_vec_field;
        float param_potential_convergence_tol;

        std::mutex grid_mtx;

        // namespace
        std::string ns; 

        /* functions for converting from world coordinates to grid indexes 
         * and vice-versa                                                  */

        int grid_x(geometry_msgs::Transform);
        int grid_y(geometry_msgs::Transform);
        int grid_x(geometry_msgs::Point);
        int grid_y(geometry_msgs::Point);
        double world_x(int x); 
        double world_y(int y); 

        // expand obstacles by NAVIGATOR_EXPAND_OBSTACLES_NUM
        void expand_obstacles();
        /* true if cell is free and at least onde adjacent cell is unexplored 
         * false otherwise                                                    */ 
        bool is_frontier(int x,int y);

        bool near_occupied(int x,int y);

        std::vector<double> normalized_gradient(int x, int y);

        std::mutex update_mtx;
        bool update_var;
        void set_updated();
        void reset_updated();
        bool has_updated();
};