#ifndef LINES_MATCHER_H
#define LINES_MATCHER_H

#include <opencv2/opencv.hpp>
#include <vector>

struct MatchResult {
    int max_sum;
    float best_dx;
    float best_dy;
    float best_angle;
};

class LinesMatcher {
public:
    static MatchResult findMatch(const std::vector<cv::Point2f>& red_points, 
                               const std::vector<cv::Point2f>& blue_points,
                               const cv::Mat& image_red_map,
                               const cv::Mat& image_blue_map,
                               const cv::Point2f& center,
                               float counter_x,
                               float counter_y,
                               float counter_yaw);

    static cv::Point2f calculateLinePointsAverage(const std::vector<cv::Point2f>& points, 
                                                const cv::Size& imageSize);

    static MatchResult refineMatchWithWhitePoints(
        const std::vector<cv::Point2f>& white_points,
        const cv::Mat& image_lines_map,
        const cv::Point2f& center,
        float current_x,
        float current_y,
        float current_yaw
    );
};

#endif // LINES_MATCHER_H 