#include "PotentialGrid.h"
#include <tf2/utils.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>
#include <queue>

PotentialGrid::PotentialGrid(ros::NodeHandle *n, std::string name_space)
{
    initialized = 0;
    // n->getParam("pub_potential", param_pub_pot);
    // n->getParam("pub_gradient", param_pub_gradient_vec);
    // n->getParam("pub_path", param_pub_path);
    // n->getParam("pub_vec_field", param_pub_vec_field);
    // n->getParam("window_radius", param_window_radius);
    // n->getParam("conv_tol", param_potential_convergence_tol);
	param_pub_pot = 1;
	param_pub_gradient_vec = 1;
	param_pub_path = 1;
	param_pub_vec_field = 1;
	param_window_radius = 3.5;
	param_potential_convergence_tol = 1.0E-7;

    std::cout << "WINDOW RADIUS: " << param_window_radius << std::endl;
    std::cout << "CONV TOL: " << param_potential_convergence_tol << std::endl;

    std::cout << "namespace: " << name_space << std::endl;
    ns="";
     if(name_space != "/"){
        ns = name_space+"/";
        ns.erase(ns.begin());
    }
    std::cout << "ns: " << ns << std::endl;
    map_sub = n->subscribe("map", 1, &PotentialGrid::get_map, this);
    objective_sub = n->subscribe("objective", 1, &PotentialGrid::get_objective, this);
    objective_is_set = false;

    if(param_pub_pot){
        ROS_INFO("publishing to /potential_field");
        potential_pub = n->advertise<nav_msgs::OccupancyGrid>("potential_field",1);
    }
    if(param_pub_gradient_vec){
        ROS_INFO("publishing to /gradient");
        vector_pub = n->advertise<visualization_msgs::Marker>("gradient",1);
    }
    if(param_pub_path){
        path_pub = n->advertise<nav_msgs::Path>("path",1);
        path.header.frame_id = "map";
        path.header.seq = 0;
    }
    vector_field_pub = n->advertise<visualization_msgs::Marker>("vector_field", 1);

    robot = new Robot(n, ns);
}

void PotentialGrid::get_map(const nav_msgs::OccupancyGrid::ConstPtr& map){
    grid_mtx.lock();
    // update meta info
    height = map->info.height;
    width  = map->info.width;
    resolution = map->info.resolution;
    map0 = map->info.origin.position;
    grid_mtx.unlock();

    int map_size = height * width;
    int x0=0, xf=height, y0=0, yf=width;

    ROS_INFO("updating info");

    geometry_msgs::Transform current_pos;
    if(robot->get_transform(current_pos) == -1){
        return;
    }    
    // ROS_INFO("current position = %f, %f\n height, width, resolution = %d, %d, %f", current_pos.transform.translation.x, current_pos.transform.translation.y, height, width, resolution);
    // ROS_INFO("position on grid = %d, %d", current_grid_x, current_grid_y);

    grid_mtx.lock();
    if(!initialized){
        //new grid
        // ROS_INFO("new grid");
        grid.clear();
        grid.resize(width);

        for(int i=0; i<width; i++){
            for(int j=0; j<height; j++){
                Cell *c = new Cell (map->data[i+j*width]);
                grid[i].push_back(c);
            }
        }
        initialized = true;
    }
    else{
        //update activation window
        std::cout << "window radius: " << param_window_radius << " resolution: " << resolution << std::endl;
        x0 = grid_x(current_pos) - (int)(param_window_radius/resolution);
        std::cout << "x0 = " << x0;
        if(x0<0)
            x0=0;
        xf = grid_x(current_pos) + (int)(param_window_radius/resolution);
        std::cout << ", xf = " << xf;
        if(xf>width)
            xf=width;
        y0 = grid_y(current_pos) - (int)(param_window_radius/resolution);
        std::cout << ", y0 = " << y0;
        if(y0<0)
            y0=0;
        yf = grid_y(current_pos) + (int)(param_window_radius/resolution);
        std::cout << ", yf = " << yf << std::endl;
        if(yf>height)
            yf=height;

        for(int i=x0; i<xf; i++){
            for(int j=y0; j<yf; j++){
                grid[i][j]->update(map->data[i+j*width]);
            }
        }
    }
    grid_mtx.unlock();

    ROS_INFO("updating potential (%d to %d, %d to %d", x0, xf, y0, yf);
    update_potential(x0, xf, y0, yf);

    if(param_pub_vec_field)
        publish_vector_field();
   
    if(param_pub_pot)
        publish_potential_field(map->info);
}

void PotentialGrid::get_objective(const geometry_msgs::PointStamped::ConstPtr& p)
{
    objective = p->point; 
    objective_is_set = true;
}

int PotentialGrid::grid_x(geometry_msgs::Transform pos){
    return (int) ((pos.translation.x - map0.x)/resolution);
}

int PotentialGrid::grid_y(geometry_msgs::Transform pos){
    return (int) ((pos.translation.y - map0.y)/resolution);
}

int PotentialGrid::grid_x(geometry_msgs::Point pos){
    return (int) ((pos.x - map0.x)/resolution);
}

int PotentialGrid::grid_y(geometry_msgs::Point pos){
    return (int) ((pos.y - map0.y)/resolution);
}

double PotentialGrid::world_x(int x){
    return (x * resolution) + map0.x + resolution/2;
}

double PotentialGrid::world_y(int y){
    return (y * resolution) + map0.y + resolution/2;
}

void PotentialGrid::update_potential(int x, int x_max, int y, int y_max){
    expand_obstacles(x,x_max,y,y_max);
    if(!objective_is_set)
        set_goal(x,x_max,y,y_max);
    else
        set_goal(objective);

    double error = 100, old;
    int iterations = 0;
    grid_mtx.lock();
    while(error > param_potential_convergence_tol){
        error = 0;
        for(int i=x; i<x_max; i++){
            for (int j=y; j<y_max; j++){
                if (grid.at(i).at(j)->occupation == FREE){
                    old = grid.at(i).at(j)->potential;
                    grid.at(i).at(j)->potential = (grid.at(i-1).at(j)->potential +
                                                   grid.at(i+1).at(j)->potential +
                                                   grid.at(i).at(j-1)->potential +
                                                   grid.at(i).at(j+1)->potential)/4;
                    error += pow(old - grid.at(i).at(j)->potential, 2);
                }
            }
        }
        iterations++;
    }
    grid_mtx.unlock();
    ROS_INFO("potential converged with %d iterations", iterations);
}

void PotentialGrid::set_goal(geometry_msgs::Point p)
{
    int x = grid_x(p);
    int y = grid_y(p);

    int radius = 3;
    int x0 = x - radius >= 0? x - radius  : 0;
    int xf = x + radius < width? x + radius  : width - 1;
    int y0 = y - radius >= 0? y - radius  : 0;
    int yf = y + radius < height? y + radius  : height - 1;

    grid_mtx.lock();
    for(int i = x0; i <= xf; i++)
        for(int j = y0; j <= yf; j++)
            if(grid[i][j]->occupation == FREE)
                grid[i][j]->potential = 0.0;
    grid_mtx.unlock();
}

void PotentialGrid::set_goal(int x, int x_max, int y, int y_max){
    bool frontier_found = false;
    ROS_INFO("setting up frontiers");
    for(int i=x; i<x_max; i++){
        for(int j=y; j<y_max; j++){
            if(is_frontier(i,j)){
                if(!frontier_found)
                    frontier_found = true;
                grid[i][j]->frontier  = FRONTIER;
            }     
        }
    }
    if(!frontier_found)
        return;
    ROS_INFO("frontiers found");
    
    std::vector<int> frontier_centers;
    
    for(int i=x; i<x_max; i++)
        for(int j=x; j<y_max; j++)
            if(grid[i][j]->frontier == FRONTIER){
                std::vector<int> frontier;
                geometry_msgs::Point center;
                center.x = 0.0;                 
                center.y = 0.0; 
                center.z = 0.0; 
                grid[i][j]->frontier = MARKED_FRONTIER;

                int size = 0;
                std::queue<int> q;
                q.push(i);
                q.push(j);

                while(!q.empty()){
                    size ++;
                    int x_front = q.front();
                    q.pop();
                    int y_front = q.front();
                    q.pop();

                    center.x += x_front; 
                    center.y += y_front;                     
                    frontier.push_back(x_front);
                    frontier.push_back(y_front);

                    for(int x_adj=x_front-1; x_adj<= x_front+1; x_adj++)
                        for(int y_adj=y_front-1; y_adj<=y_front+1; y_adj++)
                            if(grid[x_adj][y_adj]->frontier == FRONTIER){
                                grid[x_adj][y_adj]->frontier = MARKED_FRONTIER;
                                q.push(x_adj);
                                q.push(y_adj);
                            }
                        
                }

                center.x = center.x/size;
                center.y = center.y/size;

                float min_dist = FLT_MAX;
                int closest_x, closest_y;
                for(int n=0; n<frontier.size(); n+=2){
                    float dist = sqrt(pow(center.x-frontier[n], 2) + pow(center.y-frontier[n+1], 2));
                    if(dist < min_dist){
                        closest_x = frontier[n];
                        closest_y = frontier[n+1];
                        min_dist = dist;
                    }
                }
                frontier_centers.push_back(closest_x);
                frontier_centers.push_back(closest_y);
            }
    for(int n=0; n<frontier_centers.size(); n+=2){
        ROS_INFO("center: %d, %d == %f, %f", frontier_centers[n], frontier_centers[n+1], world_x(frontier_centers[n]), world_y(frontier_centers[n+1]));
        grid[frontier_centers[n]][frontier_centers[n+1]]->potential = 0.0;
    }
    return;     
}
        
void PotentialGrid::expand_obstacles(int x, int x_max, int y, int y_max){
    int rad = 3;
    for(int i=x; i<x_max; i++)
        for(int j=y; j<y_max; j++)
        {
            if(grid[i][j]->occupation == OCCUPIED){
                for(int inear = i-rad; inear<=i+rad; inear++)
                    for(int jnear=j-rad; jnear<=j+rad; jnear++){
                        if(grid[inear][jnear]->occupation == FREE){
                            grid[inear][jnear]->occupation = OCCUPIED_EXP;
                            grid[inear][jnear]->potential  = 1.0;
                        }
                    }
            }
        }
}

bool PotentialGrid::is_frontier(int i, int j){
    if(grid[i][j]->occupation != UNEXPLORED)
        return false;
    if(i>0)
    if(grid[i-1][j]->occupation == FREE)
        return true;
    if(i<width-1)
        if(grid[i+1][j]->occupation == FREE)
            return true;
    if(j>0)
        if(grid[i][j-1]->occupation == FREE)
            return true;
    if(j<height-1)
        if(grid[i][j+1]->occupation == FREE)
            return true;
    return false;
}

bool PotentialGrid::near_occupied(int i, int j){
    int rad = 3;
    if(grid[i][j]->occupation != FREE)
        return false;
    for(int x = i-rad; x <= i+rad; x++)
        for(int y = j-rad; y <= j+rad; y++)
            if(grid[x][y]->occupation == OCCUPIED)
                return true;
    return false;    
}

std::vector<double> PotentialGrid::normalized_gradient(int x, int y){
    std::vector<double> v(2);
    // grid_mtx.lock();
    v[0] = (grid[x-1][y]->potential - grid[x+1][y]->potential)/2;
    v[1] = (grid[x][y-1]->potential - grid[x][y+1]->potential)/2;
    // grid_mtx.unlock();

    if(v[0] == 0 && v[1] == 0)
        return v;

    double size_v = sqrt(pow(v[0],2) + pow(v[1],2));
    v[0] = (v[0]/size_v);
    v[1] = (v[1]/size_v);
    return v;
}

void PotentialGrid::followPotential(){
    geometry_msgs::Transform pos;
    if(robot->get_transform(pos)==-1)
        return;
   
    grid_mtx.lock();
    int x = grid_x(pos);
    int y = grid_y(pos);
    std::vector<double> gradient = normalized_gradient(x, y);
    grid_mtx.unlock();
    // ROS_INFO("gradient (%f, %f)", gradient[0], gradient[1]);
    if(param_pub_gradient_vec)
        publish_vector(gradient, pos);
    if(param_pub_path)
        publish_path(pos);
    
    robot->follow(gradient);

}

void PotentialGrid::publish_potential_field(nav_msgs::MapMetaData info){
    nav_msgs::OccupancyGrid pf;
    pf.info = info;
    pf.header.frame_id = "map";
    pf.header.seq = 1;
    for(int i=0; i<width; i++)
        for(int j=0; j<height; j++){
            pf.data.push_back(grid[j][i]->show());
        }
    potential_pub.publish(pf);
}

void PotentialGrid::publish_vector(std::vector<double> v, geometry_msgs::Transform rob){
    uint32_t arrow = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "tb_grad";
    mk.id = 0;
    mk.type = arrow;
    mk.action = visualization_msgs::Marker::ADD;
   
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0; 
    mk.pose.orientation.z = 0.0; 
    mk.pose.orientation.w = 1.0;  
    mk.pose.position.x = 0.0;      
    mk.pose.position.y = 0.0; 
    mk.pose.position.z =   0.0; 
   
    mk.points.resize(2);
    mk.points[0].x = rob.translation.x;
    mk.points[0].y = rob.translation.y;
    mk.points[0].z = rob.translation.z;
    mk.points[1].x = mk.points[0].x + v[0]*0.25;
    mk.points[1].y = mk.points[0].y + v[1]*0.25;
    mk.points[1].z = mk.points[0].z; 

    mk.color.r = 0.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.scale.x =0.02;
    mk.scale.y =0.02;
    mk.scale.z =0.10;

    mk.lifetime = ros::Duration();
    vector_pub.publish(mk);
}

void PotentialGrid::publish_vector_field(){
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = ros::Time();
    m.ns = "vector_field";
    m.id = 0;
    m.type   = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = resolution*0.10;

    // m.color.r = 1.0f;
    // m.color.g = 0.0f;
    // m.color.b = 1.0f;
    // m.color.a = 1.0f;

    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.pose.position.x = -10;
    m.pose.position.y = -10;
    m.pose.position.z = 0;

    for(int i = 0; i < grid.size(); i+=2){
        for(int j = 0; j < grid[j].size(); j+=2){
            if(grid[i][j]->occupation == FREE){
                std::vector<double> g = normalized_gradient(i, j);
                geometry_msgs::Point *p0 = new geometry_msgs::Point;
                geometry_msgs::Point *pf = new geometry_msgs::Point;
                p0->x = resolution*i + resolution/2;
                p0->y = resolution*j + resolution/2;
                p0->z = 0.0;

                pf->x = p0->x + (resolution*1.0)*g[0];
                pf->y = p0->y + (resolution*1.0)*g[1];
                pf->z = 0.0;
                
                std_msgs::ColorRGBA *c0 =new std_msgs::ColorRGBA;
                std_msgs::ColorRGBA *cf =new std_msgs::ColorRGBA;
                c0->r = 1.0f;
                c0->g = 0.0f;
                c0->b = 0.0f;
                c0->a = 1.0f;
                cf->r = 1.0f;
                cf->g = 0.0f;
                cf->b = 1.0f;
                cf->a = 1.0f;                

                if(p0->x > pf->x || p0->y > pf->y){
                    m.points.push_back(*pf);
                    m.points.push_back(*p0);
                    m.colors.push_back(*cf);      
                    m.colors.push_back(*c0);  
                }
                else{
                    m.points.push_back(*p0);
                    m.points.push_back(*pf);
                    m.colors.push_back(*c0);      
                    m.colors.push_back(*cf);      
                }
            }
        }
    }
    vector_field_pub.publish(m);
}

void PotentialGrid::publish_path(geometry_msgs::Transform current_pos){
    geometry_msgs::PoseStamped * p = new geometry_msgs::PoseStamped;
    p->header.frame_id = "map";
    p->header.stamp = ros::Time(0);
    p->pose.orientation = current_pos.rotation;
    p->pose.position.x = current_pos.translation.x;
    p->pose.position.y = current_pos.translation.y;
    p->pose.position.z = current_pos.translation.z;

    path.poses.push_back(*p);
    path_pub.publish(path);
}


Cell::Cell(int v){
    if(v >= OCC_TRESH){
        potential  = 1.0;
        occupation = OCCUPIED;
    }
    else{
        if(v >= 0 && v<= FREE_TRESH){
            occupation = FREE;
        }
        else{
            occupation = UNEXPLORED;
        }
        potential = 0.5;
    }
    frontier = NDA;
}

void Cell::update(int v){
    if(v >= OCC_TRESH){
        potential  = 1.0;
        occupation = OCCUPIED;
    }
    else{
        if(v >= 0 && v<= FREE_TRESH){
            occupation = FREE;
        }
        else{
            occupation = UNEXPLORED;
        }
    }
}
 
int Cell::show(){
    if(occupation == UNEXPLORED)
        return -1;
    return (int) (potential * 100);
}