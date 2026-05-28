#include "ros2_compat.h"
#include "distance_lookup.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("table_dist");

    std::string filename = node->declare_parameter<std::string>("table_file", "distances.txt");

    DistanceLookup distanceLookup;
    distanceLookup.init(filename);

    for (int pixel = 0; pixel <= distanceLookup.getMaxPixel(); ++pixel) {
        ROS_INFO("间隔像素: %d, 距离: %f", pixel, distanceLookup.getDistance(pixel));
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
