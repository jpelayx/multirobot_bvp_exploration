#ifndef _NAVIGATOR_H_
#define _NAVIGATOR_H_

#include "Robot.h"
#include "PotentialGrid.h"

class Navigator
{
    public:
    Navigator(ros::NodeHandle *nh, std::string ns);

    // global map callback
    void get_map(const nav_msgs::OccupancyGrid::ConstPtr&);

    // local map callback
    void get_local_map(const nav_msgs::OccupancyGrid::ConstPtr&);
    
    // objective callback 
    void get_objective(const geometry_msgs::PointStamped::ConstPtr&);

    void run();

    private:
    PotentialGrid grid;
    Robot robot;

    ros::Subscriber map_sub, 
                    local_map_sub,
                    objective_sub;
    
    ros::NodeHandle *node_handle;
    std::string name_space;

    float param_window_radius;

    bool is_merged,
         is_objective_set;

};
 

#endif // _NAVIGATOR_H_