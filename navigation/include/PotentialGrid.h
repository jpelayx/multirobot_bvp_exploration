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

enum OccType {UNEXPLORED, OCCUPIED, OCCUPIED_EXP, FREE};
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

class PotentialGrid
{
    public:
        PotentialGrid(ros::NodeHandle*, std::string name_space);
        void get_map(const nav_msgs::OccupancyGrid::ConstPtr&);
        void get_objective(const geometry_msgs::PointStamped::ConstPtr&);

        void update_potential(int x_min,int x_max,int y_min,int y_max);

        // set goals to frontiers inside window
        void set_goal(int x_min,int x_max,int y_min,int y_max);
        
        // set goal to point p
        void set_goal(geometry_msgs::Point p);
        void expand_obstacles(int x_min,int x_max,int y_min,int y_max);
        bool is_frontier(int,int);
        bool near_occupied(int,int);

        int grid_x(geometry_msgs::Transform);
        int grid_y(geometry_msgs::Transform);
        int grid_x(geometry_msgs::Point);
        int grid_y(geometry_msgs::Point);
        double world_x(int x); 
        double world_y(int y); 

        std::vector<double> normalized_gradient(int x,int y);
        
        void followPotential();

        void publish_potential_field(nav_msgs::MapMetaData);
        void publish_vector(std::vector<double>, geometry_msgs::Transform);
        void publish_path(geometry_msgs::Transform); 
        void publish_vector_field();

        int width, height;
        double resolution;
        bool initialized;

    private:
        ros::Subscriber map_sub;
        ros::Subscriber objective_sub;
        ros::Publisher potential_pub;
        ros::Publisher vector_pub;
        ros::Publisher path_pub;
        ros::Publisher vector_field_pub;

        nav_msgs::Path path;

        std::vector<std::vector<Cell*> > grid;
        
        geometry_msgs::Point map0;

        Robot* robot;

        geometry_msgs::Point objective;
        bool objective_is_set;

        int param_pub_pot,
            param_pub_gradient_vec,
            param_pub_vec_field,
            param_pub_path;
        float param_window_radius,
               param_potential_convergence_tol;

        std::mutex grid_mtx;

        std::string ns; // namespace

};