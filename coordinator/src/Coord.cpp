#include "Coord.hpp"

std::vector<std::string> parse_names(std::string n)
{
    std::vector<std::string> ns;
    int p0 = 0;
    int pf = n.find(' ', p0);
    while (pf != n.npos)
    {
        std::string name = n.substr(p0, pf-p0);
        ns.push_back(name);
        p0 = pf + 1;
        pf = n.find(' ', p0);
    }
    ns.push_back(n.substr(p0));
    return ns;
} 

Coord::Coord(ros::NodeHandle n)
{
    nh = n;
    tf_listener = new tf2_ros::TransformListener(tf_buffer);
    map_updated = false;

    std::string nss;
    n.getParam("map_list", nss);
    std::vector<std::string> names = parse_names(nss);
    add_maps(names);

}

void Coord::add_maps(std::vector<std::string> nss)
{
    // subscribe to global map 
    global_map_sub = nh.subscribe("/map", 1, &Coord::global_map_callback, this);
    ROS_INFO("coordinating :");
    for (auto it = nss.begin(); it != nss.end(); it++)
    {
        struct CoordTarget* tgt = new CoordTarget();
        tgt->is_merged = false;
        tgt->ns = *it;
        tgt->pub = nh.advertise<geometry_msgs::Point>(*it + "/objective", 1);
        map_list.push_back(tgt);
        ROS_INFO("               %s", tgt->ns.c_str());
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
            bool merged = true;
            try {
                geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(t->ns + "/map", "map", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_INFO("no tf to %s", (t->ns + "/map").c_str());
                merged = false;
            }
            if (merged)
            {
                t->is_merged = true;
                merged_maps_count++;
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
    map_mtx.lock();
    for (int y=0; y<map.info.height; y++) 
        for (int x=0; x<map.info.width; x++)
        {

        }

    map_mtx.unlock();
    return;
}

geometry_msgs::Point Coord::find_centroid(std::vector<geometry_msgs::Point> frontier)
{
    return;
}

void Coord::assign_frontiers(std::vector<geometry_msgs::Point> frontiers)
{
    return;
}

void Coord::run()
{
    ros::Rate rate(10);
    std::cout << "aaaaaaaaaaa CU" << std::endl;
    while(ros::ok())
    {
        ros::spinOnce();
        if(map_updated)
        {
            map_updated = false;
            ROS_INFO("map updated");
            // std::vector<geometry_msgs::Point> frontiers = find_frontiers();
            // assign_frontiers(frontiers);
            publish_objectives();
        }
        rate.sleep();

    }
}