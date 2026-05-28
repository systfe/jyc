#include "distance_lookup.h"

DistanceLookup::DistanceLookup() {
    
}

void DistanceLookup::init(const std::string& filename)
{
    data_ = readDistancePixelFile(filename);
    if (data_.empty()) {
        ROS_ERROR("打不开距离标定文件: %s", filename.c_str());
    } else {
        maxPixel_ = std::max_element(data_.begin(), data_.end(), [](const DistancePixel& a, const DistancePixel& b) {
            return a.pixel < b.pixel;
        })->pixel;
        lookupArray_ = createLookupArray(data_, maxPixel_);
    }
}

double DistanceLookup::getDistance(int pixel){
    if (pixel < 0 || pixel > maxPixel_) {
        return -1;
    }
    return lookupArray_[pixel];
}

int DistanceLookup::getMaxPixel(){
    return maxPixel_;
}

std::vector<DistancePixel> DistanceLookup::readDistancePixelFile(const std::string& filename) {
    std::vector<DistancePixel> data;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        DistancePixel dp;
        char colon;
        if (iss >> dp.distance >> colon >> dp.pixel) {
            data.push_back(dp);
        }
    }
    return data;
}

double DistanceLookup::interpolate(double x0, double y0, double x1, double y1, double x) {
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

std::vector<double> DistanceLookup::createLookupArray(const std::vector<DistancePixel>& data, int maxPixel) {
    std::vector<double> lookupArray(maxPixel + 1, 0.0);
    for (size_t i = 0; i < data.size() - 1; ++i) {
        for (int pixel = data[i].pixel; pixel <= data[i + 1].pixel; ++pixel) {
            lookupArray[pixel] = interpolate(data[i].pixel, data[i].distance, data[i + 1].pixel, data[i + 1].distance, pixel);
        }
    }
    return lookupArray;
}