#include "ros2_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include "distance_lookup.h"
#include "lines_map.h"
#include "lines_matcher.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;

static DistanceLookup distanceLookup;
static LinesMap lines_map;
static float counter_x = 0;
static float counter_y = 0;
static float counter_yaw = 0;
static float odom_x = 0;
static float odom_y = 0;
static float odom_yaw = 0;
static bool odom_ready = false;
static cv::Mat field_image;
static cv::Mat monitor_image;
static cv::Mat image_lines_all;
static cv::Mat image_lines_map;
static cv::Mat image_red_map;
static cv::Mat image_blue_map;
static constexpr float FIELD_SIZE_METERS = 3.0f;

static float pixelsPerMeterX()
{
    return image_lines_map.empty() ? 1.0f : image_lines_map.cols / FIELD_SIZE_METERS;
}

static float pixelsPerMeterY()
{
    return image_lines_map.empty() ? 1.0f : image_lines_map.rows / FIELD_SIZE_METERS;
}

static void syncPoseFromOdom()
{
    if (!odom_ready) {
        return;
    }

    counter_x = odom_x * pixelsPerMeterX();
    counter_y = -odom_y * pixelsPerMeterY();
    counter_yaw = -odom_yaw;
}

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
{
    syncPoseFromOdom();

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;
    Mat image_show = image_raw.clone();
    Mat image_scope = image_raw.clone();
    image_scope = Scalar(0, 0, 0);  // 将背景设置为黑色
    Mat image_lines = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_corrected = image_lines.clone();
    Mat image_red = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_blue = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_sideline_red = image_lines.clone();
    Mat image_sideline_blue = image_lines.clone();

    // 1. HSV空间的彩色区域检测。新场地是白底加彩色区域，不能再把白底当场线。
    Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);

    Mat mask_red1, mask_red2;
    // 场地图红色 RGB(250,74,59)，OpenCV HSV 约为 (2,195,250)。
    inRange(image_hsv, Scalar(0, 80, 80), Scalar(12, 255, 255), mask_red1);
    inRange(image_hsv, Scalar(170, 80, 80), Scalar(180, 255, 255), mask_red2);
    image_red = mask_red1 | mask_red2;  // 合并两个红色范围

    // 场地图浅蓝 RGB(92,169,221)，OpenCV HSV 约为 (102,149,221)。
    // 深色紫/蓝边 Hue 接近但更暗且饱和度更高，所以用亮度和饱和度排除它。
    inRange(image_hsv, Scalar(96, 70, 170), Scalar(110, 200, 255), image_blue);

    // 场地图洋红 RGB(255,25,255)，OpenCV HSV 约为 (150,230,255)。
    Mat image_magenta;
    inRange(image_hsv, Scalar(130, 45, 60), Scalar(170, 255, 255), image_magenta);

    Mat image_white = image_magenta;
    morphologyEx(image_white, image_white, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));

    int center_x = image_hsv.cols / 2;
    int center_y = image_hsv.rows / 2;
    const int self_mask_radius = std::min(image_hsv.cols, image_hsv.rows) / 8;
    circle(image_white, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_red, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_blue, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    const double min_feature_distance = 0.25;
    const double max_feature_distance = 4.50;
    const double rotated_cols = image_corrected.rows;
    const double rotated_rows = image_corrected.cols;
    const double camera_to_map_scale = std::min(
        static_cast<double>(image_lines_map.cols) / rotated_cols,
        static_cast<double>(image_lines_map.rows) / rotated_rows);
    const double projection_scale_correction = 1.00;
    const double map_pixels_per_meter = pixelsPerMeterX() * projection_scale_correction;
    cv::Point2f map_center(image_lines_map.cols / 2.0f, image_lines_map.rows / 2.0f);
    std::vector<cv::Point2f> projected_white_points;
    std::vector<cv::Point2f> projected_red_points;
    std::vector<cv::Point2f> projected_blue_points;
    auto addProjectedPoint = [&](std::vector<cv::Point2f>& points, double distance_m, double rad) {
        cv::Point2f point(
            map_center.x - distance_m * map_pixels_per_meter * std::sin(rad),
            map_center.y + distance_m * map_pixels_per_meter * std::cos(rad));
        if (point.x >= 0 && point.x < image_lines_map.cols &&
            point.y >= 0 && point.y < image_lines_map.rows) {
            points.push_back(point);
        }
    };

    // 用射线扫描彩色边缘、红色和蓝色图像
    for (int angle = 0; angle < 360; angle += 4) {
        double rad = angle * CV_PI / 180.0;
        unsigned char last_pixel_white = 0;
        unsigned char last_pixel_red = 0;
        unsigned char last_pixel_blue = 0;
        
        for (int length = 0; length < std::max(image_hsv.cols, image_hsv.rows); length++) {
            int x = static_cast<int>(center_x + length * cos(rad));
            int y = static_cast<int>(center_y + length * sin(rad));
            
            if (x >= 0 && x < image_white.cols && y >= 0 && y < image_white.rows) {
                // 检测彩色区域边缘
                unsigned char pixel_white = image_white.at<uchar>(y, x);
                if (length > 0 && last_pixel_white != pixel_white && (last_pixel_white == 255 || pixel_white == 255))
                {
                    // 在 image_show 中画紫色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 0, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 0, 255), 1);
                    cv::circle(image_lines, Point(x,y), 2, cv::Scalar(255, 255, 255), -1);

                    // 畸变矫正
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_white_points, distance_m, rad);
                        double dist = distance_m * map_pixels_per_meter / camera_to_map_scale;
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        if (x2 >= 0 && x2 < image_corrected.cols && y2 >= 0 && y2 < image_corrected.rows) {
                            image_corrected.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel_white = pixel_white;

                // 检测红线边缘
                unsigned char pixel_red = image_red.at<uchar>(y, x);
                if (last_pixel_red != 255 && pixel_red == 255)  // 修改这里的条件
                {
                    // 在 image_show 中画黄色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(0, 255, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(0, 255, 255), 1);

                    // 畸变矫正
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_red_points, distance_m, rad);
                        double dist = distance_m * map_pixels_per_meter / camera_to_map_scale;
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        if (x2 >= 0 && x2 < image_sideline_red.cols && y2 >= 0 && y2 < image_sideline_red.rows) {
                            image_sideline_red.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel_red = pixel_red;

                // 检测蓝线边缘
                unsigned char pixel_blue = image_blue.at<uchar>(y, x);
                if (last_pixel_blue != 255 && pixel_blue == 255)
                {
                    // 在 image_show 中画浅蓝色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 255, 0), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 255, 0), 1);

                    // 畸变矫正
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_blue_points, distance_m, rad);
                        double dist = distance_m * map_pixels_per_meter / camera_to_map_scale;
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        if (x2 >= 0 && x2 < image_sideline_blue.cols && y2 >= 0 && y2 < image_sideline_blue.rows) {
                            image_sideline_blue.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel_blue = pixel_blue;
            }
        }
    }

    // 将 image_corrected 顺时针旋转90度
    cv::Mat rotated_image;
    cv::rotate(image_corrected, rotated_image, cv::ROTATE_90_CLOCKWISE);

    // 计算旋转后图像的中心点
    cv::Point2f img_center(rotated_image.cols/2.0f, rotated_image.rows/2.0f);
    
    // 从原始射线检测结果直接投影到地图坐标，避免中间图像尺寸裁剪远距离特征。
    std::vector<cv::Point2f> white_points = projected_white_points;
    std::vector<cv::Point2f> red_points = projected_red_points;
    std::vector<cv::Point2f> blue_points = projected_blue_points;

    // 用于计算平均位置的变量
    cv::Point2f red_avg(0.0f, 0.0f);
    cv::Point2f blue_avg(0.0f, 0.0f);

    // 旋转红线和蓝线图像
    cv::Mat rotated_red, rotated_blue;
    cv::rotate(image_sideline_red, rotated_red, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(image_sideline_blue, rotated_blue, cv::ROTATE_90_CLOCKWISE);

    static int debug_frame_count = 0;
    if (++debug_frame_count % 30 == 0) {
        printf(
            "detected points before match: white=%zu red=%zu blue=%zu\n",
            white_points.size(),
            red_points.size(),
            blue_points.size());
    }

    // [1]先使用红蓝边线进行初次匹配
    const bool has_sidelines = !red_points.empty() && !blue_points.empty();
    if (has_sidelines && !odom_ready) {
        red_avg = LinesMatcher::calculateLinePointsAverage(red_points, image_lines_map.size());
        blue_avg = LinesMatcher::calculateLinePointsAverage(blue_points, image_lines_map.size());
        
        // 计算从red_avg指向blue_avg的角度
        float dx = blue_avg.x - red_avg.x;
        float dy = blue_avg.y - red_avg.y;
        float angle = atan2(dy, dx) * 180 / CV_PI;
        
        // 计算红点和蓝点的连线的中心点坐标
        cv::Point2f line_center((red_avg.x + blue_avg.x) / 2, (red_avg.y + blue_avg.y) / 2);
        counter_yaw = -90 - angle;
    }
    
    cv::Point2f center = map_center;

    if (has_sidelines && !odom_ready) {
        int last_max_sum = 0;
        while (true) {
            MatchResult match_result = LinesMatcher::findMatch(red_points, blue_points, 
                                                                image_red_map, image_blue_map,
                                                                center, counter_x, counter_y, counter_yaw);
            
            // 如果本次匹配结果没有改善，则退出循环
            if (match_result.max_sum <= last_max_sum) {
                break;
            }
            
            // 更新计数器和上一次的最大匹配值
            counter_x += match_result.best_dx;
            counter_y += match_result.best_dy;
            counter_yaw += match_result.best_angle;
            last_max_sum = match_result.max_sum;
        }
    }

    float rad = (counter_yaw) * CV_PI / 180.0;
    
    if (has_sidelines && !odom_ready) {
        // 初始化范围统计变量
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        // 先统计红点范围
        for(const auto& point : red_points) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
            
            min_x = std::min(min_x, rotated_x);
            max_x = std::max(max_x, rotated_x);
            min_y = std::min(min_y, rotated_y);
            max_y = std::max(max_y, rotated_y);
        }

        // 再加上统计蓝点范围
        for(const auto& point : blue_points) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
            
            min_x = std::min(min_x, rotated_x);
            max_x = std::max(max_x, rotated_x);
            min_y = std::min(min_y, rotated_y);
            max_y = std::max(max_y, rotated_y);
        }

        // 利用红蓝边界，剔除边界外的白点
        std::vector<cv::Point2f> filtered_white_points;
        for(const auto& point : white_points) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
            
            // 只保留在范围内的点
            if (rotated_x >= min_x && rotated_x <= max_x && 
                rotated_y >= min_y && rotated_y <= max_y) {
                filtered_white_points.push_back(point);
            }
        }
        // 用过滤后的点替换原来的white_points
        white_points = filtered_white_points;
    }

    // [2] 使用白点进行场线匹配
    if (!odom_ready) {
        int last_white_sum = 0;
        while (true) {
            MatchResult white_match = LinesMatcher::refineMatchWithWhitePoints(
                white_points,
                image_lines_map,
                center,
                counter_x,
                counter_y,
                counter_yaw
            );
            
            // 如果本次匹配结果没有改善，则退出循环
            if (white_match.max_sum <= last_white_sum) {
                break;
            }
            
            // 更新计数器和上一次的最大匹配值
            counter_x += white_match.best_dx;
            counter_y += white_match.best_dy;
            counter_yaw += white_match.best_angle;
            last_white_sum = white_match.max_sum;
        }
    }

    // 显示匹配效果
    cv::Mat image_match_result = image_lines_all.clone();
    // cv::Mat image_match_result = image_lines_map.clone();
    // cv::Mat image_match_result = image_red_map.clone();
    // cv::Mat image_match_result = cv::Mat::zeros(image_red_map.size(), CV_8UC1);
    // for(int y = 0; y < image_match_result.rows; y++) {
    //     for(int x = 0; x < image_match_result.cols; x++) {
    //         image_match_result.at<uchar>(y,x) = std::max(image_red_map.at<uchar>(y,x), 
    //                                               image_blue_map.at<uchar>(y,x));
    //     }
    // }
    // cv::cvtColor(image_match_result, image_match_result, cv::COLOR_GRAY2BGR);
    
    // printf("counter_x: %.2f, counter_y: %.2f, counter_yaw: %.2f\n", counter_x, counter_y, counter_yaw);

    // 绘制白点
    for(const auto& point : white_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,255,0), -1);
        }
    }

    // 绘制红点
    for(const auto& point : red_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,0,255), -1);
        }
    }

    // 绘制蓝点
    for(const auto& point : blue_points) {
        float x = point.x - center.x;
        float y = point.y - center.y;
        float rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
        float rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(255,0,0), -1);
        }
    }
    
    // 将各种线条标记到 image_scope 上
    // for(int y = 0; y < image_scope.rows; y++) {
    //     for(int x = 0; x < image_scope.cols; x++) {
    //         // if(image_corrected.at<uchar>(y,x) == 255) {
    //         //     // 白色点表示彩色边缘
    //         //     circle(image_scope, Point(x,y), 2, Scalar(255,255,255), -1);
    //         // }
    //         if(image_sideline_red.at<uchar>(y,x) == 255) {
    //             // 红色点表示红线
    //             circle(image_scope, Point(x,y), 2, Scalar(0,0,255), -1);
    //         }
    //         if(image_sideline_blue.at<uchar>(y,x) == 255) {
    //             // 蓝色点表示蓝线
    //             circle(image_scope, Point(x,y), 2, Scalar(255,0,0), -1);
    //         }
    //     }
    // }

    // 在image_scope上绘制筛选后的white_points
    // for(const auto& point : white_points) {
    //     if(point.x >= 0 && point.x < image_scope.cols &&
    //        point.y >= 0 && point.y < image_scope.rows) {
    //         int x = -(point.x - x_offset - img_center.x)/scale_factor_x + img_center.x;
    //         int y = (point.y - y_offset - img_center.y)/scale_factor_y + img_center.y;
    //         circle(image_scope, Point(y, x), 2, Scalar(255,255,255), -1);
    //     }
    // }

    // 创建显示用的图像副本
    field_image.copyTo(monitor_image(cv::Rect(0, 50, field_image.cols, field_image.rows)));
    
    // 清除并更新信息区域
    cv::Mat info_area = monitor_image(cv::Rect(0, 0, monitor_image.cols, 50));
    info_area.setTo(cv::Scalar(255, 255, 255));  // 清除之前的文本
    
    std::string coordinates = cv::format(
        "X: %.2fm  Y: %.2fm  Yaw: %.1f",
        counter_x / pixelsPerMeterX(),
        -counter_y / pixelsPerMeterY(),
        -counter_yaw);
    cv::putText(monitor_image, coordinates, cv::Point(0, 30), 
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
    // cv::imshow("scope", image_scope);
    cv::imshow("match_result", image_match_result);
    cv::imshow("定位", monitor_image);
    cv::waitKey(1);
}

// 添加重定位回调函数
void relocCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    counter_x = msg->pose.position.x * pixelsPerMeterX();
    counter_y = -msg->pose.position.y * pixelsPerMeterY();
    
    // 从四元数转换为欧拉角(yaw)
    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    counter_yaw = -yaw * 180.0 / M_PI;  // 转换为图像坐标系角度
    ROS_WARN("重新定位: ( %.2f,  %.2f) yaw: %.2f", counter_x, counter_y, counter_yaw);
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_yaw = yaw * 180.0 / M_PI;
    odom_ready = true;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loc_sidelines");

    std::string table_file = node->declare_parameter<std::string>("table_file", "distances.txt");
    distanceLookup.init(table_file);

    std::string lines_file = node->declare_parameter<std::string>("lines_file", "lines.jpg");
    image_lines_all = cv::imread(lines_file);


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

    
    // 读取参数
    std::string lines_map_path = node->declare_parameter<std::string>("lines_map_file", "");
    std::string red_map_path = node->declare_parameter<std::string>("red_map_file", "");
    std::string blue_map_path = node->declare_parameter<std::string>("blue_map_file", "");
    // 读取图像为灰度图
    image_lines_map = cv::imread(lines_map_path, cv::IMREAD_GRAYSCALE);
    image_red_map = cv::imread(red_map_path, cv::IMREAD_GRAYSCALE);
    image_blue_map = cv::imread(blue_map_path, cv::IMREAD_GRAYSCALE);

    // 检查图像是否成功加载
    if(image_lines_map.empty() || image_red_map.empty() || image_blue_map.empty()) {
        ROS_ERROR("场线模板读取失败！");
        return -1;
    }

    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/omni_camera/image_raw", 1, imageCallback);
    // 添加重定位话题订阅
    auto reloc_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/reloc_pose", 1, relocCallback);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odomCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
