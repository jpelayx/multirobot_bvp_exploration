#include "Robot.h"
#include <iostream>

Robot::Robot()
{
    return;
}

Robot::Robot(ros::NodeHandle *nh, std::string ns)
{
    initialize(nh, ns);
}

void Robot::initialize(ros::NodeHandle *nh, std::string ns)
{
    name.assign(ns);
    tf_listener = new tf2_ros::TransformListener (tf_buffer);
    cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // first transform will probably always fail, but next ones should be fine.
    geometry_msgs::TransformStamped current_pos;
    try{
        current_pos = tf_buffer.lookupTransform(name+"map", name+"base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch(tf2::TransformException &ex){
       // ignores failed attempt as expected
    }
    try{
        current_pos = tf_buffer.lookupTransform("map", name+"base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch(tf2::TransformException &ex){
       // ignores failed attempt as expected
    }
}

void Robot::follow(std::vector<double> objective)
{
    if (objective.size() != 2)
    {
        ROS_WARN("Objective vector does not have 2 dimensions.");
        return;
    }

    std::cout << "oi\n";
    
    geometry_msgs::Twist vel;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    if(objective[0] == 0.0 && objective[1] == 0.0)
    {
        cmd_vel_pub.publish(vel);
        ROS_INFO("zero grad");
        return;
    }

    geometry_msgs::Transform pos;
    if ( get_transform(pos) != 0 )
    {
        cmd_vel_pub.publish(vel);
        return;
    }

    ROS_INFO("BOSTA");

    tf2::Quaternion rotation;
    tf2::fromMsg(pos.rotation, rotation);
    double robot_angle = tf2::getYaw(rotation);  // [0, 2PI]
    
    double objective_angle = atan2(objective[1], objective[0]);
    if(isnan(objective_angle)) // 90 or 270 degree tangent
    {
        if(objective[1] > 0)
            objective_angle = M_PI_2;
        else 
            objective_angle = -M_PI_2;
    }
    
    double phi = objective_angle - robot_angle;
    phi = normalize_angle(phi);

    if(phi > M_2_PI)
    {
        vel.angular.z = 0.5;
    }
    else if (phi < -M_PI_2)
    {
        vel.angular.z = -0.5;
    }
    else{
        vel.angular.z = (phi/M_PI_2)*0.5;
    }
    vel.linear.x = 0.1;
    cmd_vel_pub.publish(vel);
    ROS_INFO("got vel long");
}

double Robot::normalize_angle(double a)
{
    while(a > M_PI)
        a -= M_PI*2;
    while(a < -M_PI)
        a += M_PI*2;  
    return a;
}

int Robot::get_transform(geometry_msgs::Transform &transform)
{
    geometry_msgs::TransformStamped current_pos;
    try{
            current_pos = tf_buffer.lookupTransform("map", name+"base_link", ros::Time(0), ros::Duration(3.0));
            transform = current_pos.transform;
            return 0;
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return -1;
    }
}

int Robot::get_local_transform(geometry_msgs::Transform &transform)
{
    geometry_msgs::TransformStamped current_pos;
    try{
            current_pos = tf_buffer.lookupTransform(name+"map", name+"base_link", ros::Time(0), ros::Duration(3.0));
            transform = current_pos.transform;
            return 0;
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return -1;
    }
}
