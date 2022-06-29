#include "Coord.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "coordinator");
    ros::NodeHandle node;
    Coord c(node);
    // std::vector<std::string> nss {std::string("tb3_0"), std::string("tb3_1")};
    // c.add_maps(nss);
    c.run();

    return 0;
}
