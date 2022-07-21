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
            t->pub.publish(t->objective);
            ROS_INFO("got %s", (t->ns).c_str());        
        }
    }
}

void Coord::assign_frontiers(std::vector<int> frontiers)
{
    // 1. computes frontier vector for each robot, ordered by distance
    std::vector<std::vector<int>> close_frontiers;
    std::vector<int> assigned_frontiers;
    for (auto m = map_list.begin(); m != map_list.end(); m++)
    {
        if((*m)->is_merged)
        {
            geometry_msgs::Point m_p = get_position(*m);
            close_frontiers.push_back(find_closest_frontiers(m_p, frontiers, merged_maps_count));
        }
        else 
        {
            std::vector<int> empty_v;
            close_frontiers.push_back(empty_v);
        }
        assigned_frontiers.push_back(-1);
    }

    // 2. assigns frontiers, dealing w/ conflicts
    int assigned_maps = 0;
    int has_conflicts = 0;
    while (!(assigned_maps == merged_maps_count && has_conflicts == 0))
    {
        for (int i=0; i<map_list.size(); i++)
        {
            std::cout << "mapa " << i << std::endl;
            if(!map_list[i]->is_merged)
                break;
            if(assigned_frontiers[i] == -1)
            {
                assigned_frontiers[i] = 0;
            }
            int conflict = find_conflict(i, close_frontiers, assigned_frontiers);
            if(conflict != -1)
            {
                has_conflicts++;
                std::cout << "found conflict " << std::endl;
                // in this case, more agents than frontiers, so we split the frontier where the 
                // inevitable conflict occurs and send each agent to the closest half.
                if( assigned_maps == frontiers.size())
                {
                    std::cout << "splitting frontier " << std::endl;
                    int conflict_frontier = close_frontiers[i][assigned_frontiers[i]];
                    std::vector<int> split_f = global_map->split_frontier(conflict_frontier, i, conflict);
                    if (global_map->get_euclidian_distance(get_position(map_list[i]), split_f[0]) <
                        global_map->get_euclidian_distance(get_position(map_list[conflict]), split_f[0]))
                    {
                        close_frontiers[i].push_back(split_f[0]);
                        assigned_frontiers[i] = close_frontiers[i].size() - 1;
                        close_frontiers[conflict].push_back(split_f[1]);
                        assigned_frontiers[conflict] = close_frontiers[conflict].size() - 1;
                    }
                    else
                    {
                        close_frontiers[i].push_back(split_f[1]);
                        assigned_frontiers[i] = close_frontiers[i].size() - 1;
                        close_frontiers[conflict].push_back(split_f[0]);
                        assigned_frontiers[conflict] = close_frontiers[conflict].size() - 1;
                    }
                    has_conflicts--; // conflict solved
                }
                // reassigns agent most distant to frontier
                else 
                {
                    std::cout << "reassigning most distant" << std::endl;
                    int conflict_frontier = close_frontiers[i][assigned_frontiers[i]];
                    int worse_m;
                    if (global_map->get_euclidian_distance(get_position(map_list[i]), conflict_frontier) <
                        global_map->get_euclidian_distance(get_position(map_list[conflict]), conflict_frontier))
                        worse_m = conflict;   
                    else
                        worse_m = i;
                    assigned_frontiers[worse_m]++;
                    std::cout << "number " << worse_m << " reassigned" << std::endl;

                    // if no new conflict is found, resolved
                    if(find_conflict(worse_m, close_frontiers, assigned_frontiers) == -1)
                        has_conflicts--;
                    else
                     std::cout << "conflict not yet resolved " << std::endl; 
                    // else, conflict move to other cells, to be resolved in the next iteration or
                    // further down the loop.
                }
            }
            assigned_maps++;
        }
    } 

    for (int i=0; i<map_list.size(); i++)
    {
        geometry_msgs::PointStamped p;
        p.point = global_map->get_point(close_frontiers[i][assigned_frontiers[i]]);
        p.header.frame_id = "/map";
        p.header.stamp = ros::Time(0);
        map_list[i]->objective =  p;
        std::cout << "map " << i << "goes to (" << p.point.x << ", " << p.point.y << ")" << std::endl; 
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
    if (n > fs.size())
        return std::vector<int>(fs.begin(), fs.begin() + n);
    return fs;
}

int Coord::find_conflict(int i, std::vector<std::vector<int>> frontiers, std::vector<int> assigned_frontiers)
{
    int f = assigned_frontiers[i];
    for(int j=0; j<assigned_frontiers.size(); j++)
        if(i != j && assigned_frontiers[i] == assigned_frontiers[j])
            return j;
    return -1;
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