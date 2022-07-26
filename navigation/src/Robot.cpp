#include "Robot.h"

Robot::Robot(ros::NodeHandle *nh, std::string ns)
{
    name = ns;
    tf_listener = new tf2_ros::TransformListener (tf_buffer);
    cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void Robot::follow(std::vector<double> objective)
{
    if (objective.size() != 2)
    {
        ROS_WARN("Objective vector does not have 2 dimensions.");
        return;
    }
    
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
        return;
    }

    geometry_msgs::Transform pos;
    if ( get_transform(pos) != 0 )
    {
        cmd_vel_pub.publish(vel);
        return;
    }

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
            std::string n = name;
            n.erase(0,1);
            current_pos = tf_buffer.lookupTransform(n+"map", name+"base_link", ros::Time(0));
            transform = current_pos.transform;
            return 0;
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return -1;
    }
}