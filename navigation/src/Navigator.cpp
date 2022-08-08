#include "Navigator.h"

Navigator::Navigator(ros::NodeHandle *nh, std::string ns)
{
    node_handle = nh;
    name_space = ns;

    // namespace format can't start with '/' (because of tf2) 
    // and should end with '/'
    if(*name_space.begin() == '/')
        name_space.erase(name_space.begin());
    if(*name_space.end()   != '/')
        name_space = name_space + "/";

    param_window_radius = 3.5;
    // nh->getParam("window_radius", param_window_radius);

    ROS_INFO("Starting navigation node for robot %s", name_space.c_str());

    robot.initialize(nh, ns);
    grid.initialize(nh, ns);

    geometry_msgs::Transform transform; 
    if(robot.get_transform(transform) == -1)
    {
        // map not merged to global map, using local navigation
        is_merged = false;
        local_map_sub = node_handle->subscribe("map", 1, &Navigator::get_local_map, this);
    }
    else 
    {
        // map merged to global map, using global navigation
        is_merged = true;
        map_sub = node_handle->subscribe("/map", 1, &Navigator::get_map, this);
    }
    is_objective_set = false;
    objective_sub = node_handle->subscribe("objective", 1, &Navigator::get_objective, this);

}

void Navigator::get_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    geometry_msgs::Transform t;
    robot.get_transform(t);
    if (!is_merged) // first time getting global map 
    {
        grid.update(map, t, -1); // update entire area
        is_merged = true;
    }
    else 
        grid.update(map, t, param_window_radius);
}


void Navigator::get_local_map(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    geometry_msgs::Transform transform;
    if (robot.get_transform(transform) == 0)  // has merged with global map
    {
        map_sub = node_handle->subscribe("/map", 1, &Navigator::get_map, this);
        local_map_sub.shutdown();
        is_objective_set = false;
        ROS_INFO("%s has joined global map.", name_space.c_str());
    }    
    else 
    {
        geometry_msgs::Transform t;
        robot.get_local_transform(t);
        grid.update(map, t, param_window_radius);
        grid.set_local_goal();
        is_objective_set = true;
    }
}

void Navigator::get_objective(const geometry_msgs::PointStamped::ConstPtr &objective)
{
    grid.set_goal(objective->point);
    is_objective_set = true;
}

void Navigator::run()
{
    ros::Rate loop_rate(15);

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Transform transform;
        if (is_merged)
            robot.get_transform(transform); // global navigation
        else  
            robot.get_local_transform(transform); // local navigation
        std::vector<double> gradient = grid.normalized_gradient(transform);
        robot.follow(gradient);
    }
}