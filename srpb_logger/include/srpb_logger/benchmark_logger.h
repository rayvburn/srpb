#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace srpb {
namespace logger {

class BenchmarkLogger {
public:
    virtual void init(ros::NodeHandle& nh) {
        //path to save the recorded data
        nh.param("srpb/log_filename", log_filename_, std::string("log.txt"));
        // ID of the frame that poses will be expressed in
        nh.param("srpb/log_frame_id", target_frame_, target_frame_);
        // append the timestamp to filename
        log_filename_ = BenchmarkLogger::appendToFilename(log_filename_, BenchmarkLogger::timeToString());
    }

    /**
     * @brief Returns the part number of the log
     */
    int getLogPart() const {
        return part_num_;
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
     * @brief Appends @ref suffix to the @ref filename and returns new instance of string
     */
    static std::string appendToFilename(const std::string& filename, const std::string& suffix) {
        auto path_base = filename.substr(0, filename.find_last_of(EXTENSION_SEPARATOR));
        auto extension = filename.substr(filename.find_last_of(EXTENSION_SEPARATOR) + 1);
        return path_base + "_" + suffix + EXTENSION_SEPARATOR + extension;
    }

    /**
     * @brief Transform poses to the frame given by @ref target_frame_
     *
     * @note calling simply:
     *   tf_buffer_.transform(pose_in, pose_out, target_frame_);
     * does not force time source to use so, e.g., for goal poses received from external tool it may happen
     * that wrong time source will be used and transform is not found (wall vs sim time difference).
     */
    geometry_msgs::PoseWithCovarianceStamped transformPose(const geometry_msgs::PoseWithCovarianceStamped& pose_in) {
        // transforming might not be necessary
        if (pose_in.header.frame_id == target_frame_) {
            return pose_in;
        }
        auto transform_stamped = getTransform(pose_in.header.frame_id);
        geometry_msgs::PoseWithCovarianceStamped pose_out;
        tf2::doTransform(pose_in, pose_out, transform_stamped);
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

    /// @brief Retrieves transform from the @ref source_frame to the @ref target_frame_
    geometry_msgs::TransformStamped getTransform(const std::string& source_frame) {
        return getTransform(source_frame, target_frame_);
    }

    /// @brief Retrieves transform from the @ref source_frame to the @ref target_frame
    geometry_msgs::TransformStamped getTransform(const std::string& source_frame, const std::string& target_frame) {
        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Failure %s\n", ex.what());
        }
        return tf;
    }

protected:
    BenchmarkLogger(): part_num_(1), target_frame_("map"), tf_listener_(tf_buffer_) {}

    void start(std::fstream& log_file, const std::string& log_filename) {
        //open the file to record the navigation data
        log_file.open(log_filename, std::ios::out);
        if (log_file.is_open()) {
            return;
        }
        throw std::runtime_error(
            "Failed to open the log file " + log_filename + " for benchmarking `move_base`"
        );
    }

    void finish(std::fstream& log_file) {
        // close the file
        log_file.close();
        if (!log_file.is_open()) {
            return;
        }
        throw std::runtime_error("Failed to close the log file for benchmarking `move_base`");
    }

    void startNextPart(std::fstream& log_file, const std::string& log_filename) {
        auto suffix = std::string("part") + std::to_string(part_num_);
        auto log_filename_mod = BenchmarkLogger::appendToFilename(log_filename, suffix);
        start(log_file, log_filename_mod);
    }

    void incrementLogPartCounter() {
        part_num_++;
    }

    static constexpr auto EXTENSION_SEPARATOR = ".";
    std::string log_filename_;
    // Useful when a single log consists of multiple files; incremented in each @ref incrementLogPartCounter
    int part_num_;

    // For transforming poses to a common frame
    std::string target_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

} // namespace logger
} // namespace srpb
