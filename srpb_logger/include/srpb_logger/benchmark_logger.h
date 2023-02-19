#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ctime>
#include <iomanip>
#include <sstream>

namespace srpb {
namespace logger {

class BenchmarkLogger {
public:
    virtual void init(ros::NodeHandle& nh) {
        //path to save the recorded data
        nh.param("log_filename", log_filename_, std::string("log.txt"));
        // ID of the frame that poses will be expressed in
        nh.param("log_frame_id", target_frame_, target_frame_);

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

    /**
     * @brief Transform poses to the frame given by @ref target_frame_
     */
    geometry_msgs::PoseWithCovarianceStamped transformPose(const geometry_msgs::PoseWithCovarianceStamped& pose_in) {
        geometry_msgs::PoseWithCovarianceStamped pose_out;
        try {
            tf_buffer_.transform(pose_in, pose_out, target_frame_);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Failure %s\n", ex.what());
        }
        return pose_out;
    }

    /**
     * @brief Transform poses to the frame given by @ref target_frame_
     */
    geometry_msgs::PoseWithCovarianceStamped transformPose(const geometry_msgs::PoseWithCovariance& pose_in, const std::string& frame_in) {
        geometry_msgs::PoseWithCovarianceStamped pose_in_stamped;
        std::string frame_source = frame_in;
        // warning: argument source_frame in tf2 frame_ids cannot start with a '/'
        if (frame_source.at(0) == '/') {
            frame_source.erase(frame_source.begin());
        }
        pose_in_stamped.header.frame_id = frame_source;
        pose_in_stamped.pose = pose_in;
        return transformPose(pose_in_stamped);
    }

    /**
     * @brief Transform poses to the frame given by @ref target_frame_
     */
    geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& pose_in) {
        geometry_msgs::PoseWithCovarianceStamped pose_in_cov;
        pose_in_cov.header = pose_in.header;
        pose_in_cov.pose.pose = pose_in.pose;
        // will be ignored in output anyway
        pose_in_cov.pose.covariance.fill(0.0);
        geometry_msgs::PoseWithCovarianceStamped pose_out = transformPose(pose_in_cov);
        geometry_msgs::PoseStamped output;
        output.header = pose_out.header;
        output.pose = pose_out.pose.pose;
        return output;
    }

protected:
    BenchmarkLogger(): target_frame_("odom"), tf_listener_(tf_buffer_) {}

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

    // For transforming poses to a common frame
    std::string target_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

} // namespace logger
} // namespace srpb
