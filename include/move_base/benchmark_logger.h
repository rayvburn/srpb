#pragma once

#include <ros/ros.h>

class BenchmarkLogger {
public:
    virtual void init(ros::NodeHandle& nh) {
        //path to save the recorded data
        nh.param("log_filename", log_filename_, std::string("log.txt"));
    }

    void start() {
        //open the file to record the navigation data
        log_file_ = fopen(log_filename_.c_str(), "w+");
        if (log_file_ == NULL) {
            ROS_ERROR("Failed to open the log file for benchmarking `move_base`");
            return;
        }
    }

    void finish() {
        //close the file
        fclose(log_file_);
    }

protected:
    BenchmarkLogger() = default;

    FILE* log_file_;
    std::string log_filename_;
};
