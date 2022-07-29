#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <mutex>
#include <iostream>


class Robot
{
    public: 
        Robot(ros::NodeHandle *nh, std::string ns);

        /* set objective vector to follow
         *   objective_vector: 2D objective vector  */
        void follow(std::vector<double> objective_vector);
        
        /* puts in transform the tf from robot to global map  
         * if successfull, returns 0 
         * else, return -1                                 */ 
        int get_transform(geometry_msgs::Transform &transform);

    private:
        ros::Publisher cmd_vel_pub;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener *tf_listener;

        std::string name;

        double normalize_angle(double a);

};

#endif