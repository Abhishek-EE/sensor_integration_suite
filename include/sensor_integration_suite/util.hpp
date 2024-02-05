#pragma once

#include <iomanip>
#include <string>
#include <cmath>

// Simple thread-safe logger
void logger(const std::string msg);

struct LidarPoint {
    float angle;        ///< point angle in degrees
    float distance;     ///< point distance in meters
    int confidence;     ///< confidence of reading (generally >200 is OK)
    int timestamp;      ///< timestamp in milliseconds (resets after 65536 ms)
    float x; ///cartesian coordinate 
    float y; /// cartesian coordinate

    LidarPoint(float angle, float dist, int conf=-1, int ts=-1) :
        angle(angle), distance(dist), confidence(conf), timestamp(ts) {
            float angleRadians = angle* M_PI / 180.0;
            x = distance*cos(angleRadians);
            y = distance*sin(angleRadians);
        }
};

///
/// For printing LidarPoint structures to C++ streams.
///
std::ostream& operator<<(std::ostream& os, LidarPoint& p);