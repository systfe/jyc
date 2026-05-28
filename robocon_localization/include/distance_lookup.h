#ifndef DISTANCE_LOOKUP_H
#define DISTANCE_LOOKUP_H

#include "ros2_compat.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

struct DistancePixel {
        double distance;
        int pixel;
    };

class DistanceLookup {
public:
    DistanceLookup();
    void init(const std::string& filename);
    double getDistance(int pixel);
    int getMaxPixel();

private:
    

    std::vector<DistancePixel> readDistancePixelFile(const std::string& filename);
    double interpolate(double x0, double y0, double x1, double y1, double x);
    std::vector<double> createLookupArray(const std::vector<DistancePixel>& data, int maxPixel);

    std::vector<DistancePixel> data_;
    std::vector<double> lookupArray_;
    int maxPixel_;
};

#endif // DISTANCE_LOOKUP_H