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
    cv::Mat createColorMask() const;
    cv::Mat createMagentaMask() const;
    cv::Mat createPurpleMask() const;
    cv::Mat createFilledSafetyZoneMaskFromPurple(const cv::Mat& purpleMask) const;
    cv::Mat createRedMask() const;
    cv::Mat createBlueMask() const;
    cv::Mat createEdgeMask(const cv::Mat& mask) const;
    cv::Mat createDirectionalGradientFromMask(const cv::Mat& mask, int innerRadius, int outerRadius) const;
    cv::Mat createFilledGradientFromMask(const cv::Mat& mask, const cv::Mat& blockedMask, int outerRadius) const;
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
