#include "Coord.hpp"


Coord::Coord(ros::NodeHandle n)
{
    nh = n;
    tf_listener = new tf2_ros::TransformListener(tf_buffer);
    map_updated = false;

    std::string nss;
    n.getParam("map_list", nss);

    std::vector<std::string> names;
    int i = 0;
    int j = nss.find(' ');
    while(j!=nss.npos)
    {
        std::string ns = nss.substr(i,j-1);
        names.push_back(ns);
        i = j;
        j = nss.find(' ');
    }
    add_maps(names);

}

void Coord::add_maps(std::vector<std::string> nss)
{
    // subscribe to global map 
    global_map_sub = nh.subscribe("/map", 1, &Coord::global_map_callback, this);
    for (auto it = nss.begin(); it != nss.end(); it++)
    {
        struct CoordTarget* tgt = new CoordTarget();
        tgt->is_merged = false;
        tgt->ns = *it;
        tgt->pub = nh.advertise<geometry_msgs::Point>(*it + "/objective", 1);
        map_list.push_back(tgt);
    }
    merged_maps_count = 0;
}

void Coord::global_map_callback(const nav_msgs::OccupancyGrid& m)
{
    map_mtx.lock();
    map = m;
    map_mtx.unlock();
    update_merged_maps();
    map_updated = true;
}

void Coord::update_merged_maps()
{
    if (merged_maps_count == map_list.size())
        return; // all maps merged
    for (auto it = map_list.begin(); it != map_list.end(); it++)
    {
        // if able to find transform from global map to local map 
        // local map has been merged
        CoordTarget* t = *it;
        if(!t->is_merged)
        {
            try {
                geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(t->ns + "/map", "map", ros::Time(0));
                t->is_merged = true;
                merged_maps_count++;
            }
            catch (tf2::TransformException &ex) {
                ROS_INFO("no tf to %s", (t->ns + "/map").c_str());
            }
        }
    }
}

void Coord::publish_objectives()
{
    for (auto it = map_list.begin(); it != map_list.end(); it++)
    {
        CoordTarget *t = *it;
        if(t->is_merged){
            // t->pub.publish(t->objective);
            ROS_INFO("got %s", (t->ns).c_str());
        }
    }
}

std::vector<geometry_msgs::Point> Coord::find_frontiers()
{
    std::vector<geometry_msgs::Point> v;
    return v;
}

void Coord::assign_frontiers(std::vector<geometry_msgs::Point> frontiers)
{
    return;
}

void Coord::run()
{
    while(ros::ok())
    {
        ros::spinOnce();
        if(map_updated)
        {
            map_updated = false;
            // std::vector<geometry_msgs::Point> frontiers = find_frontiers();
            // assign_frontiers(frontiers);
            publish_objectives();
        }

    }
}