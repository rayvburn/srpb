#pragma once

#include <ros/ros.h>

class BenchmarkLogger {
public:
    virtual void init(ros::NodeHandle& nh) {
        //path to save the recorded data
        nh.param("log_filename", log_filename_, std::string("log.txt"));
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

    std::string log_filename_;
};
