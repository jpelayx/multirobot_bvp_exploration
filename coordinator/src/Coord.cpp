#include "Coord.hpp"


Coord::Coord(ros::NodeHandle n)
{
    nh = n;
    tf_listener = new tf2_ros::TransformListener(tf_buffer);
}

void Coord::add_maps(std::vector<std::string> name_spaces)
{
    // subscribe to global map 
    global_map_sub = nh.subscribe("/map", 1, &Coord::global_map_callback, this);
    map_list = name_spaces;
}

void Coord::global_map_callback(const nav_msgs::OccupancyGrid& m)
{
    map_mtx.lock();
    map = m;
    map_mtx.unlock();
    update_merged_maps();
}

void Coord::update_merged_maps()
{
    if (merged_maps.size() == map_list.size())
        return; // all maps merged
    for (auto ns = map_list.begin(); ns != map_list.end(); ns++)
    {
        // if able to find transform from global map to local map 
        // local map has been merged
        bool in_global_map = true;
        try {
            geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(*ns + "/map", "map", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            in_global_map = false;
            ROS_INFO("no tf to %s", (*ns + "/map").c_str());
        }
        if (in_global_map)
            merged_maps.push_back(*ns);
    }
}

void Coord::run()
{
    ros::spin();
}