#include "conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <ufo/map/occupancy_map.h>

class ufomapNode : public rclcpp::Node
{
public:
    ufomapNode() : Node("ufomap_node") {}

    // TODO: use conversions.h to output ufomap somehow
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the UfoMap node
    rclcpp::spin(std::make_shared<ufomapNode>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
