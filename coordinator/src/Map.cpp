#include "Map.hpp"

Map::Map(ros::NodeHandle nh, std::string topic)
{
    map_sub = nh.subscribe("/map", 1, &Map::map_callback, this);
    map_updated = false;
}


void Map::map_callback(const nav_msgs::OccupancyGrid& m)
{
    map_mtx.lock();
    map = m;
    if (map_labels.empty() || map_labels.size() != map.data.size())
    {
        map_labels.resize(map.data.size());
        for(int i=0; i<map_labels.size(); i++)
            map_labels[i] = NOT_VISITED;
    }
    map_mtx.unlock();
    map_updated = true;
}

bool Map::updated()
{
    return map_updated;
}

void Map::reset_updated()
{
    map_updated = false;
}

std::vector<geometry_msgs::Point> Map::get_frontiers_centroids()
{
    std::vector<std::vector<geometry_msgs::Point>> frontiers = find_frontiers();
    std::vector<geometry_msgs::Point> centroids;
    for (auto f = frontiers.begin(); f != frontiers.end(); f++)
    {
        centroids.push_back(find_centroid(*f));
    }
    return centroids;
}

std::vector<std::vector<geometry_msgs::Point>> Map::find_frontiers()
{
    std::vector<std::vector<geometry_msgs::Point>> frontiers;
    map_mtx.lock();
    for (int i = 0; i < map.data.size(); i++)
    {
        if(is_frontier(i) && map_labels[i] == NOT_VISITED)
        {
            std::vector<geometry_msgs::Point> f = extract_frontier(i);
            frontiers.push_back(f);
        }
    }
    map_mtx.unlock();
}