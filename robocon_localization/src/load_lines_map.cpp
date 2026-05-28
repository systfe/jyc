#include "ros2_compat.h"
#include "lines_map.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("load_lines_map");

    std::string imagePath = node->declare_parameter<std::string>("lines_file", "lines.jpg");

    LinesMap lines_map;
    lines_map.loadImage(imagePath);
    cv::Mat whiteImage = lines_map.getLinesMap();
    cv::Mat blueImage = lines_map.getBlueMap();
    cv::Mat redImage = lines_map.getRedMap();

    // 获取用户主目录路径
    const char* homeDir = getenv("HOME");
    if (homeDir != nullptr) {
        std::string savePath(homeDir);
        // 保存为无损PNG格式
        cv::imwrite(savePath + "/white_lines.png", whiteImage);
        cv::imwrite(savePath + "/blue_lines.png", blueImage);
        cv::imwrite(savePath + "/red_lines.png", redImage);
    }

    cv::imshow("lines", whiteImage);
    cv::imshow("blue map", blueImage);
    cv::imshow("red map", redImage);
    cv::waitKey(0);
    rclcpp::shutdown();
    return 0;
}
