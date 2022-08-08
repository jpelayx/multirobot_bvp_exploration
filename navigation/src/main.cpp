#include "Navigator.h"

#define WINDOW_RADIUS 3.7
#define CONV_ERROR 0.05
#define PUBLISH 1

int main (int argc, char **argv){
    ros::init(argc, argv, "potential");
    ros::NodeHandle n;

    ros::Rate loop_rate(15);

    std::string ns = ros::this_node::getNamespace();

    Navigator nav(&n, ns);
    nav.run();
    return 0;
}