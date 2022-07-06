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
    global_map = new Map(nh, "/map");

    std::string nss;
    n.getParam("map_list", nss);
    std::vector<std::string> names = parse_names(nss);
    add_maps(names);

}

void Coord::add_maps(std::vector<std::string> nss)
{
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

void Coord::assign_frontiers(std::vector<int> frontiers)
{
    // for every map's position
    // get shortest euclidian distance to a frontier
    // [TODO] deal with conflicts

    for (auto tgt = map_list.begin(); tgt != map_list.end(); tgt++)
    {
        if ((*tgt)->is_merged)
        {
            geometry_msgs::Point tgt_p = get_position(*tgt);
            ROS_INFO("%s position: %f, %f", (*tgt)->ns.c_str(), tgt_p.x, tgt_p.y);
            std::vector<int> closest_frontiers = find_closest_frontiers(tgt_p, frontiers, merged_maps_count);
            (*tgt)->objective = global_map->get_point(closest_frontiers[0]);
            ROS_INFO("assigned position (%f, %f) to robot %s.", (*tgt)->objective.x, (*tgt)->objective.y, (*tgt)->ns.c_str());
        }
    }

}

geometry_msgs::Point Coord::get_position(CoordTarget* t)
{
    geometry_msgs::Point p;
    if(!t->is_merged)
        return p;
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = tf_buffer.lookupTransform(t->ns + "/map", "map", ros::Time(0));
    }
    catch(const tf2::TransformException& e)
    {
        return p;
    }
    p.x = tf.transform.translation.x;
    p.y = tf.transform.translation.y;
    p.z = tf.transform.translation.z;    
    return p;
}

std::vector<int> Coord::find_closest_frontiers(geometry_msgs::Point p, std::vector<int> fs, int n)
{
    int p_pos = global_map->get_pos(p);
    std::sort(fs.begin(), fs.end(), 
              [p_pos, this](int i, int j)
              {return this->global_map->get_euclidian_distance(p_pos, i) < this->global_map->get_euclidian_distance(p_pos, j);});
    return std::vector<int>(fs.begin(), fs.begin() + n);
}


void Coord::run()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        if(global_map->updated())
        {
            global_map->reset_updated();
            ROS_INFO("map updated");
            update_merged_maps();
            std::vector<int> frontiers = global_map->get_frontiers_centroids();
            ROS_INFO("computed frontiers");
            assign_frontiers(frontiers);
            ROS_INFO("frontiers assigned");
            publish_objectives();
            ROS_INFO("objectives published");
        }
        rate.sleep();

    }
}