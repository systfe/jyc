#ifndef LINES_MAP_H
#define LINES_MAP_H

#include <opencv2/opencv.hpp>
#include <string>

class LinesMap {
public:
    LinesMap();

    void loadImage(const std::string& imagePath);
    cv::Mat getLinesMap() const;
    cv::Mat getRawImage() const;
    cv::Mat getBlueMap() const;
    cv::Mat getRedMap() const;

private:
    cv::Mat createGradientMask(int size);
    void generateGradientImage();
    void generateBlueGradientImage();
    void generateRedGradientImage();

    cv::Mat lines_image;
    cv::Mat grayImage;
    cv::Mat gradientImage;
    cv::Mat blueIMap;
    cv::Mat redIMap;
};

#endif // LINES_MAP_H 