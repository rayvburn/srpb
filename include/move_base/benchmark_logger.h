#pragma once

#include <ros/ros.h>

#include <ctime>
#include <iomanip>
#include <sstream>

class BenchmarkLogger {
public:
    virtual void init(ros::NodeHandle& nh) {
        //path to save the recorded data
        nh.param("log_filename", log_filename_, std::string("log.txt"));

        auto log_path_base = log_filename_.substr(0, log_filename_.find_last_of(EXTENSION_SEPARATOR));
        auto log_extension = log_filename_.substr(log_filename_.find_last_of(EXTENSION_SEPARATOR) + 1);

        log_filename_ = log_path_base + "_" + BenchmarkLogger::timeToString() + EXTENSION_SEPARATOR + log_extension;
    }

    /**
     * @brief Transforms current timestamp to the formatted string
     * @url https://stackoverflow.com/a/16358111
     */
    static std::string timeToString() {
        auto time = std::time(nullptr);
        auto tm = *std::localtime(&time);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        return oss.str();
    }

protected:
    BenchmarkLogger() = default;

    void start(FILE** log_file, std::string log_filename) {
        //open the file to record the navigation data
        *log_file = fopen(log_filename.c_str(), "w+");
        if (*log_file == NULL) {
            ROS_ERROR("Failed to open the log file `%s` for benchmarking `move_base`", log_filename.c_str());
            return;
        }
    }

    void finish(FILE** log_file) {
        if (*log_file == NULL) {
            ROS_ERROR("Failed to close the log file for benchmarking `move_base`");
            return;
        }
        // close the file
        fclose(*log_file);
    }

    static constexpr auto EXTENSION_SEPARATOR = ".";
    std::string log_filename_;
};
