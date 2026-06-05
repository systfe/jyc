#include "ros2_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "distance_lookup.h"
#include "lines_map.h"

using namespace cv;

static DistanceLookup distanceLookup;
static LinesMap lines_map;
static float counter_x = 0;
static float counter_y = 0;
static float counter_yaw = 0;
static cv::Mat field_image;
static cv::Mat monitor_image;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
{
    static int count = 0;
    count++;
    if(count < 1)
        return;
    else
        count = 0;
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;
    Mat image_show = image_raw.clone();
    Mat image_lines = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_corrected = image_lines.clone();

    // 1. HSV空间的彩色区域检测。新场地是白底加彩色区域，不能再把白底当场线。
    Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);

    Mat image_white;
    inRange(image_hsv, Scalar(0, 35, 30), Scalar(180, 255, 255), image_white);
    morphologyEx(image_white, image_white, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));

    int center_x = image_hsv.cols / 2;
    int center_y = image_hsv.rows / 2;

    // 用射线扫描彩色边缘图像
    for (int angle = 0; angle < 360; angle += 1) {
        double rad = angle * CV_PI / 180.0;
        unsigned char last_pixel = 0;
        for (int length = 0; length < std::max(image_hsv.cols, image_hsv.rows); length++) {
            int x = static_cast<int>(center_x + length * cos(rad));
            int y = static_cast<int>(center_y + length * sin(rad));
            
            // 别超出图像范围
            if (x >= 0 && x < image_white.cols && y >= 0 && y < image_white.rows) {
                // 检测是否在彩色区域边缘
                unsigned char pixel = image_white.at<uchar>(y, x);
                if (length > 0 && last_pixel != pixel && (last_pixel == 255 || pixel == 255))
                {
                    // 在 image_show 中画紫色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 0, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 0, 255), 1);
                    // 在 image_lines 绘制未矫正的场线点
                    cv::circle(image_lines, Point(x,y), 2, cv::Scalar(255, 255, 255), -1);

                    // 畸变矫正
                    double dist = distanceLookup.getDistance(length)*20;
                    if(dist > 0 && dist < 150)
                    {
                        // 根据 dist 计算像素点位置
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));

                        // 在 image_corrected 中绘制已经矫正的场线点
                        // cv::circle(image_corrected, Point(x2,y2), 2, cv::Scalar(255, 255, 255), -1);
                        if (x2 >= 0 && x2 < image_corrected.cols && y2 >= 0 && y2 < image_corrected.rows) {
                            image_corrected.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel = pixel;
            }
        }
    }

    // 读取场线模板
    cv::Mat image_lines_map = lines_map.getLinesMap();

    // 将 image_corrected 顺时针旋转90度
    cv::Mat rotated_image;
    cv::rotate(image_corrected, rotated_image, cv::ROTATE_90_CLOCKWISE);

    // 计算旋转后图像的中心点
    cv::Point2f img_center(rotated_image.cols/2.0f, rotated_image.rows/2.0f);
    
    // 添加缩放比例变量
    const float scale_factor = 2.5f;
    // 计算横向偏移量，使两个图像中心对齐
    float x_offset = (image_lines_map.cols - rotated_image.cols) / 2.0f;
    // 计算纵向偏移量
    float y_offset = (image_lines_map.rows - rotated_image.rows) / 2.0f;

    // 从旋转后的图像中提取彩色边缘点
    std::vector<cv::Point2f> white_points;
    for(int y = 0; y < rotated_image.rows; y++) {
        for(int x = 0; x < rotated_image.cols; x++) {
            if(rotated_image.at<uchar>(y,x) == 255) {
                // 计算相对于中心点的偏移，并加上横向和纵向偏移量
                float scaled_x = img_center.x + (x - img_center.x) * scale_factor + x_offset;
                float scaled_y = img_center.y + (y - img_center.y) * scale_factor + y_offset;
                white_points.push_back(cv::Point2f(scaled_x, scaled_y));
            }
        }
    }

    // 在一定范围内搜索最佳匹配
    int max_sum = 0;
    float best_dx = 0, best_dy = 0;
    float best_angle = 0;
    cv::Point2f center(img_center.x + x_offset, img_center.y + y_offset);

    // 搜索范围：平移±1像素，旋转±1度
    for(int dx = -1; dx <= 1; dx++) {
        for(int dy = -1; dy <= 1; dy++) {
            for(float angle = -1; angle <= 1; angle += 1) {
                // 加入累计的旋转角度
                float rad = (angle + counter_yaw) * CV_PI / 180.0;
                int sum = 0;

                // 对每个彩色边缘点进行变换并检查匹配度
                for(const auto& point : white_points) {
                    // 计算旋转后的点
                    float x = point.x - center.x;
                    float y = point.y - center.y;
                    // 加入累计的平移量
                    float rotated_x = x * cos(rad) - y * sin(rad) + center.x + dx + counter_x;
                    float rotated_y = x * sin(rad) + y * cos(rad) + center.y + dy + counter_y;

                    // 检查点是否在地图范围内
                    if(rotated_x >= 0 && rotated_x < image_lines_map.cols &&
                       rotated_y >= 0 && rotated_y < image_lines_map.rows) {
                        // 累加匹配度
                        sum += image_lines_map.at<uchar>(rotated_y, rotated_x);
                    }
                }

                // 更新最佳匹
                if(sum > max_sum) {
                    max_sum = sum;
                    best_dx = dx;
                    best_dy = dy;
                    best_angle = angle;
                    counter_x += dx;
                    counter_y += dy;
                    counter_yaw += angle;
                }
            }
        }
    }

    // 使用最佳匹配参数绘制结果
    // cv::Mat match_result = image_lines_map.clone();
    // cv::cvtColor(match_result, match_result, cv::COLOR_GRAY2BGR);
    cv::Mat match_result = lines_map.getRawImage();
    
    // 添加ROS_INFO打印最佳匹配参数
    // printf("匹配参数 - dx: %.2f, dy: %.2f, angle: %.2f  sum= %d\n", best_dx, best_dy, best_angle, max_sum );
    printf("counter_x: %.2f, counter_y: %.2f, counter_yaw: %.2f\n", counter_x, counter_y, counter_yaw);
    float rad = (best_angle + counter_yaw) * CV_PI / 180.0;
    for(const auto& point : white_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + best_dx + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + best_dy + counter_y;

        if(rotated_x >= 0 && rotated_x < match_result.cols &&
           rotated_y >= 0 && rotated_y < match_result.rows) {
            cv::circle(match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,0,255), -1);
        }
    }

    // 创建显示用的图像副本
    field_image.copyTo(monitor_image(cv::Rect(0, 50, field_image.cols, field_image.rows)));
    
    // 清除并更新信息区域
    cv::Mat info_area = monitor_image(cv::Rect(0, 0, monitor_image.cols, 50));
    info_area.setTo(cv::Scalar(255, 255, 255));  // 清除之前的文本
    
    std::string coordinates = cv::format("X: %.2f  Y: %.2f  Yaw: %.2f", counter_x, counter_y, counter_yaw);
    cv::putText(monitor_image, coordinates, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    
    // 绘制机器人图标
    // 计算field_image的中心点
    cv::Point2f bottom_center(field_image.cols/2.0f, field_image.rows/2.0f+50);
    
    // 计算机器人在图像中的位置
    float display_scale_x = static_cast<float>(field_image.cols) / image_lines_map.cols;
    float display_scale_y = static_cast<float>(field_image.rows) / image_lines_map.rows;
    float robot_x = bottom_center.x + counter_x * display_scale_x;
    float robot_y = bottom_center.y + counter_y * display_scale_y;
    cv::Point robot_pos(robot_x, robot_y);
    
    // 绘制机器人主体（紫色填充的圆形，黑色轮廓）
    int robot_radius = 15;
    cv::circle(monitor_image, robot_pos, robot_radius, cv::Scalar(0,0,0), 2);  // 黑色轮廓
    cv::circle(monitor_image, robot_pos, robot_radius-2, cv::Scalar(255,0,255), -1);  // 紫色填充
    
    // 绘制朝向线段
    float direction_length = 20.0f;
    float direction_rad = counter_yaw * CV_PI / 180.0;
    cv::Point direction_end(
        robot_x + direction_length * cos(direction_rad),
        robot_y + direction_length * sin(direction_rad)
    );
    cv::line(monitor_image, robot_pos, direction_end, cv::Scalar(0,0,0), 2);

    // 显示
    cv::imshow("result", image_show);
    // cv::imshow("white", image_white);
    // cv::imshow("lines", image_lines);
    // cv::imshow("image_corrected", image_corrected);
    cv::imshow("match_result", match_result);
    cv::imshow("field_image", monitor_image);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("localization");

    std::string table_file = node->declare_parameter<std::string>("table_file", "distances.txt");
    distanceLookup.init(table_file);

    std::string lines_file = node->declare_parameter<std::string>("lines_file", "lines.jpg");
    lines_map.loadImage(lines_file);

    std::string field_file = node->declare_parameter<std::string>("field_file", "field_bg.png");
    cv::Mat resultImage = cv::imread(field_file);
    if(resultImage.empty()) {
        ROS_ERROR("读取场地背景图失败: %s", field_file.c_str());
        return -1;
    }
    
    // 初始化field_image为彩色图像，尺寸为resultImage的一半
    cv::resize(resultImage, field_image, cv::Size(resultImage.cols/2, resultImage.rows/2));
    
    // 初始化monitor_image，大小与field_image相同
    monitor_image = cv::Mat(field_image.rows + 50, field_image.cols, CV_8UC3);

    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/omni_camera/image_raw", 1, imageCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
