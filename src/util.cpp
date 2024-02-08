#include "sensor_integration_suite/util.hpp"
#include <iostream>
#include <cstdlib> // for getenv
#include <string>
#include <fstream>
#include <iostream>

void logger(const std::string& msg) {
    const char* rosLogDir = std::getenv("ROS_LOG_DIR");
    const char* rosHome = std::getenv("ROS_HOME");
    std::string logDirectory;

    if (rosLogDir != nullptr) {
        logDirectory = rosLogDir;
    } else if (rosHome != nullptr) {
        logDirectory = rosHome;
        logDirectory += "/log"; // ROS_HOME/log is the default logging directory if ROS_LOG_DIR is not set
    } else {
        logDirectory = std::getenv("HOME");
        logDirectory += "/.ros/log"; // Default to ~/.ros/log if neither ROS_LOG_DIR nor ROS_HOME are set
    }

    // Append your log file name to the directory path
    std::string logFilePath = logDirectory + "/lidar_log.txt";

    // Proceed with logging as before
    static std::ofstream logFile(logFilePath, std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open log file: " << logFilePath << std::endl;
        return;
    }
    logFile << msg << std::endl;
    logFile.flush();
}

std::ostream& operator<<(std::ostream& os, LidarPoint& p)
{
    return os << std::fixed << std::setprecision(3) << R"({"angle": )" 
            << p.angle << R"(, "distance": )" << p.distance << R"(, "confidence": )" 
            << p.confidence << R"(, "timestamp": )" << p.timestamp << "}";
}