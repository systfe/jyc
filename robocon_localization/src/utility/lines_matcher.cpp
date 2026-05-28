#include "lines_matcher.h"

MatchResult LinesMatcher::findMatch(const std::vector<cv::Point2f>& red_points, 
                                     const std::vector<cv::Point2f>& blue_points,
                                     const cv::Mat& image_red_map,
                                     const cv::Mat& image_blue_map,
                                     const cv::Point2f& center,
                                     float counter_x,
                                     float counter_y,
                                     float counter_yaw) {
    // ... 原findSidelineMatch函数的实现 ...
    MatchResult result = {0, 0, 0, 0};
    
    for(int dx = -1; dx <= 1; dx++) {
        for(int dy = -1; dy <= 1; dy++) {
            for(float angle = -1; angle <= 1; angle += 1) {
                float rad = (angle + counter_yaw) * CV_PI / 180.0;
                int sum = 0;

                for(const auto& point : red_points) {
                    float x = point.x - center.x;
                    float y = point.y - center.y;
                    float rotated_x = x * cos(rad) - y * sin(rad) + center.x + dx + counter_x;
                    float rotated_y = x * sin(rad) + y * cos(rad) + center.y + dy + counter_y;

                    if(rotated_x >= 0 && rotated_x < image_red_map.cols &&
                       rotated_y >= 0 && rotated_y < image_red_map.rows) {
                        sum += image_red_map.at<uchar>(rotated_y, rotated_x);
                    }
                }

                for(const auto& point : blue_points) {
                    float x = point.x - center.x;
                    float y = point.y - center.y;
                    float rotated_x = x * cos(rad) - y * sin(rad) + center.x + dx + counter_x;
                    float rotated_y = x * sin(rad) + y * cos(rad) + center.y + dy + counter_y;

                    if(rotated_x >= 0 && rotated_x < image_blue_map.cols &&
                       rotated_y >= 0 && rotated_y < image_blue_map.rows) {
                        sum += image_blue_map.at<uchar>(rotated_y, rotated_x);
                    }
                }

                if(sum > result.max_sum) {
                    result.max_sum = sum;
                    result.best_dx = dx;
                    result.best_dy = dy;
                    result.best_angle = angle;
                }
            }
        }
    }
    return result;
}

cv::Point2f LinesMatcher::calculateLinePointsAverage(const std::vector<cv::Point2f>& points, 
                                                      const cv::Size& imageSize) {
    // ... 原calculateLinePointsAverage函数的实现 ...
    cv::Point2f avg_point(0, 0);
    if (points.empty()) {
        return avg_point;
    }

    cv::Mat points_image = cv::Mat::zeros(imageSize, CV_8UC1);
    for (const auto& point : points) {
        if (point.x >= 0 && point.x < points_image.cols &&
            point.y >= 0 && point.y < points_image.rows) {
            points_image.at<uchar>(point.y, point.x) = 255;
        }
    }

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(points_image, lines, 1, CV_PI/180, 10);

    if (!lines.empty()) {
        float best_rho = lines[0][0];
        float best_theta = lines[0][1];
        int max_support = 0;

        for (const auto& line : lines) {
            float rho = line[0];
            float theta = line[1];
            int support = 0;
            
            for (const auto& point : points) {
                float dist = std::abs(point.x * std::cos(theta) + 
                                    point.y * std::sin(theta) - rho);
                if (dist < 5.0) {
                    support++;
                }
            }

            if (support > max_support) {
                max_support = support;
                best_rho = rho;
                best_theta = theta;
            }
        }

        int point_count = 0;
        for (const auto& point : points) {
            float dist = std::abs(point.x * std::cos(best_theta) + 
                                point.y * std::sin(best_theta) - best_rho);
            if (dist < 5.0) {
                avg_point.x += point.x;
                avg_point.y += point.y;
                point_count++;
            }
        }
        
        if (point_count > 0) {
            avg_point.x /= point_count;
            avg_point.y /= point_count;
        }
    } else {
        for (const auto& point : points) {
            avg_point.x += point.x;
            avg_point.y += point.y;
        }
        avg_point.x /= points.size();
        avg_point.y /= points.size();
    }

    return avg_point;
} 

// 添加新的函数声明
MatchResult LinesMatcher::refineMatchWithWhitePoints(
    const std::vector<cv::Point2f>& white_points,
    const cv::Mat& image_lines_map,
    const cv::Point2f& center,
    float current_x,
    float current_y,
    float current_yaw
) {
    MatchResult result = {0, 0, 0, 0};
    int max_sum = 0;

    for(int dx = -1; dx <= 1; dx++) {
        for(int dy = -1; dy <= 1; dy++) {
            for(float angle = -1; angle <= 1; angle += 1) {
                float rad = (angle + current_yaw) * CV_PI / 180.0;
                int sum = 0;

                for(const auto& point : white_points) {
                    float x = point.x - center.x;
                    float y = point.y - center.y;
                    float rotated_x = x * cos(rad) - y * sin(rad) + center.x + dx + current_x;
                    float rotated_y = x * sin(rad) + y * cos(rad) + center.y + dy + current_y;

                    if(rotated_x >= 0 && rotated_x < image_lines_map.cols &&
                       rotated_y >= 0 && rotated_y < image_lines_map.rows) {
                        sum += image_lines_map.at<uchar>(rotated_y, rotated_x);
                    }
                }

                if(sum > max_sum) {
                    max_sum = sum;
                    result.best_dx = dx;
                    result.best_dy = dy;
                    result.best_angle = angle;
                    result.max_sum = sum;
                }
            }
        }
    }
    return result;
}
