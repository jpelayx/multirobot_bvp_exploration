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
        map_labels.resize(map.data.size());
    for(int i=0; i<map_labels.size(); i++)
        map_labels[i] = NOT_VISITED;
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

float Map::get_euclidian_distance(geometry_msgs::Point p, int j)
{
    return get_euclidian_distance(get_pos(p), j);
}

float Map::get_euclidian_distance(int i, int j)
{
    return sqrt(pow(get_x(i)-get_x(j),2) + pow(get_y(i)-get_y(j),2));
}


std::vector<int> Map::get_frontiers_centroids()
{
    ROS_INFO("finding frontiers");
    std::vector<std::vector<int>> frontiers = find_frontiers();
    ROS_INFO("found %d frontiers. computing centroids", (int)frontiers.size());
    std::vector<int> centroids;
    for (auto f = frontiers.begin(); f != frontiers.end(); f++)
    {
        centroids.push_back(find_centroid(*f));
    }
    ROS_INFO("found centroids");
    return centroids;
}


std::vector<int> Map::split_frontier(int f)
{
    std::vector<int> whole_frontier = extract_frontier(f);
    std::vector<int> a, b;

}

int Map::find_centroid(std::vector<int> f)
{
    // compute centroid 
    int x_sum = 0, y_sum = 0;
    for (auto it = f.begin(); it != f.end(); it++ )
    {
        x_sum += get_x(*it);
        y_sum += get_y(*it);
    }
    int cx = x_sum/f.size(), cy = y_sum/f.size();
    ROS_INFO("centroid: %d, %d", cx, cy);

    // find closest (euclidian distance) point in frontier
    int closest = f[0];
    float distance = sqrt(pow(get_x(f[0])-cx, 2) + pow(get_y(f[0])-cy,2));
    for (auto it = f.begin()+1; it != f.end(); it++)
    {
        float new_distance = sqrt(pow(get_x(*it)-cx, 2) + pow(get_y(*it)-cy,2));
        if (new_distance < distance)
        {
            distance = new_distance;
            closest = *it;
        }
    }
    ROS_INFO("closest = %d, %d", get_x(closest), get_y(closest));
    return closest;
}

std::vector<std::vector<int>> Map::find_frontiers()
{
    std::vector<std::vector<int>> frontiers;
    for (int i = 0; i < map.data.size(); i++)
    {
        if(is_frontier(i) && map_labels[i] == NOT_VISITED)
        {
            std::vector<int> f = extract_frontier(i);
            if (f.size() >= MIN_FRONTIER_SIZE)
                frontiers.push_back(f);
        }
    }
    return frontiers;
}

std::vector<int> Map::extract_frontier(int pos)
{
    std::vector<int> frontier, candidates;
    candidates.push_back(pos);
    while (!candidates.empty())
    {
        int c = candidates[candidates.size()-1];
        candidates.pop_back();

        if(is_frontier(c) && map_labels[c] == NOT_VISITED)
        {
            frontier.push_back(c);
            std::vector<int> a = adjacent_pos(c);
            candidates.insert(candidates.end(), a.begin(), a.end());
        }
        map_labels[c] = VISITED;
    }
    ROS_INFO("got %d cells", (int)frontier.size());
    return frontier;   
}

std::vector<int> Map::adjacent_pos(int pos)
{
    std::vector<int> a;
    a.push_back(get_pos(get_x(pos)-1, get_y(pos)-1));
    a.push_back(get_pos(get_x(pos)-1, get_y(pos)+1));
    a.push_back(get_pos(get_x(pos)-1, get_y(pos)  ));
    a.push_back(get_pos(get_x(pos)+1, get_y(pos)-1));
    a.push_back(get_pos(get_x(pos)+1, get_y(pos)+1));
    a.push_back(get_pos(get_x(pos)+1, get_y(pos)  ));
    a.push_back(get_pos(get_x(pos)  , get_y(pos)-1));
    a.push_back(get_pos(get_x(pos)  , get_y(pos)+1));
    return a;
}

bool Map::is_frontier(int pos)
{
    int x = get_x(pos);
    int y = get_y(pos);

    // a frontier is a free cell that is adjacent to an unexplored cell
    int8_t cell = get_cell_value(x,y);
    if (!(cell <= 30 && cell >= 0)) // is not a free cell
        return false;
    int f = get_info().width - 1;
    if ((x > 0)          && is_unexplored(x-1, y  )) return true;
    if ((x > 0 && y > 0) && is_unexplored(x-1, y-1)) return true; 
    if ((x > 0 && y < f) && is_unexplored(x-1, y+1)) return true; 
    if ((x < f)          && is_unexplored(x+1, y  )) return true;
    if ((x < f && y > 0) && is_unexplored(x+1, y-1)) return true; 
    if ((x < f && y < f) && is_unexplored(x+1, y+1)) return true; 
    if ((y > 0)          && is_unexplored(x  , y-1)) return true; 
    if ((y < f)          && is_unexplored(x  , y+1)) return true; 
    return false;
}

bool Map::is_unexplored(int x, int y)
{
    int8_t cell = get_cell_value(x, y);
    return cell == -1 || (cell > FREE && cell < OCCUPIED);
}

int Map::get_x(int pos)
{
    nav_msgs::MapMetaData info = get_info();
    return pos%info.width;
}

int Map::get_y(int pos)
{
    nav_msgs::MapMetaData info = get_info();
    return (int) floor(pos/info.width);
}


int Map::get_pos(int x, int y)
{
    nav_msgs::MapMetaData info = get_info();
    return y*info.width + x;
}

int Map::get_pos(geometry_msgs::Point p)
{
    nav_msgs::MapMetaData info = get_info();
    int x = (p.x - info.origin.position.x - info.resolution/2) / info.resolution;
    int y = (p.y - info.origin.position.y - info.resolution/2) / info.resolution;
    return get_pos(x, y);
}

geometry_msgs::Point Map::get_point(int pos)
{
    nav_msgs::MapMetaData info = get_info();
    geometry_msgs::Point p;
    p.x = get_x(pos) * info.resolution + info.origin.position.x + info.resolution/2;
    p.y = get_y(pos) * info.resolution + info.origin.position.y + info.resolution/2;
    p.z = info.origin.position.z + info.resolution/2;
    return p;
}

nav_msgs::MapMetaData Map::get_info()
{
    map_mtx.lock();
    nav_msgs::MapMetaData info = map.info;
    map_mtx.unlock();
    return info; 
}

int8_t Map::get_cell_value(int x, int y)
{
    return get_cell_value(get_pos(x, y));
}

int8_t Map::get_cell_value(int pos)
{
    int8_t value;
    map_mtx.lock();
    value = map.data[pos];
    map_mtx.unlock();
    return value;
}
