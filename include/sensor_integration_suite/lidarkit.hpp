#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <memory>

#include "sensor_integration_suite/util.hpp"

const size_t PACKET_LEN = 47;
const size_t NUM_POINTS = 12;

#ifdef __APPLE__
const std::string DEFAULT_DEVICE_URI = "/dev/tty.usbserial-0001";
#else
const std::string DEFAULT_DEVICE_URI = "/dev/ttyUSB0";
#endif

class LidarKit {
private:
    int fd;
    std::string dev_uri;
    bool debug_mode;
    std::atomic<bool> is_running;
    std::unique_ptr<std::thread> dev_thread;
    std::vector<LidarPoint> points;
    std::mutex points_mtx;

    void open_device();
    void close_device();
    void reset_device();
    void thread_loop();
    std::vector<LidarPoint> parse_packet_to_points(const std::vector<uint8_t>& packet);
    float calculate_angle_for_point(size_t pointIndex, size_t totalPoints);
    int extract_timestamp(const std::vector<uint8_t>& packet);
public:

    ///
    /// Creates an state and configuration structure for the lidar device.
    /// @param dev_uri the path of the serial-connected lidar device (e.g. "/dev/ttyUSB0")
    ///
    LidarKit(std::string dev_uri, bool debug_mode = true);
    // LidarKit(std::string dev_uri);
    void set_dev_uri(std::string uri){
        dev_uri=uri;
        this->open_device();
        if (this->fd == -1) throw std::exception();}

    void set_debug_mode(bool debug_mode){
        this->debug_mode = debug_mode;
    };

    LidarKit();

    ///
    /// Safely destroys the state structrure.
    ///
    ~LidarKit();

    ///
    /// Signals the system to begin recording data from the lidar device.
    /// @return returns true if the state was previously stopped, and the device could be started, otherwise returns false
    ///
    bool start();

    ///
    /// Signals the system to finish recording data from the lidar device.
    ///
    void stop();

    ///
    /// Transfers the list of recorded points (safely and efficiently).
    /// @return a vector of LidarPoint structures for each recorded point
    ///
    std::vector<LidarPoint> get_points();
};
