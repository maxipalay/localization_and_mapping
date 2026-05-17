#pragma once

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

namespace visual_inertial_localization::test
{

inline builtin_interfaces::msg::Time stampFromSeconds(double seconds)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(seconds);
    stamp.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(stamp.sec)) * 1e9);
    return stamp;
}

inline int64_t stampNs(double seconds)
{
    return static_cast<int64_t>(seconds * 1e9);
}

inline std::filesystem::path writeTempTagMap(const std::string &yaml_contents)
{
    static std::atomic<uint64_t> counter{0};
    const auto unique_id =
        std::to_string(::getpid()) + "_" +
        std::to_string(counter.fetch_add(1)) + "_" +
        std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
    const auto path = std::filesystem::temp_directory_path() /
                      ("visual_inertial_localization_tag_map_" + unique_id + ".yaml");
    std::ofstream out(path);
    out << yaml_contents;
    out.close();
    return path;
}

class TempTagMapFile
{
public:
    explicit TempTagMapFile(const std::string &yaml_contents)
        : path_(writeTempTagMap(yaml_contents))
    {
    }

    ~TempTagMapFile()
    {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }

    const std::filesystem::path &path() const noexcept
    {
        return path_;
    }

private:
    std::filesystem::path path_;
};

inline geometry_msgs::msg::TransformStamped makeTransform(
    const std::string &parent_frame,
    const std::string &child_frame,
    const Eigen::Vector3d &translation,
    const Eigen::Quaterniond &rotation = Eigen::Quaterniond::Identity())
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = translation.x();
    tf.transform.translation.y = translation.y();
    tf.transform.translation.z = translation.z();
    tf.transform.rotation.x = rotation.x();
    tf.transform.rotation.y = rotation.y();
    tf.transform.rotation.z = rotation.z();
    tf.transform.rotation.w = rotation.w();
    return tf;
}

inline void setStaticTransform(
    tf2_ros::Buffer &tf_buffer,
    const std::string &parent_frame,
    const std::string &child_frame,
    const Eigen::Vector3d &translation,
    const Eigen::Quaterniond &rotation = Eigen::Quaterniond::Identity())
{
    auto tf = makeTransform(parent_frame, child_frame, translation, rotation);
    tf_buffer.setTransform(tf, "localization_test", true);
}

inline apriltag_msgs::msg::AprilTagDetectionArray makeDetectionArray(
    double stamp_seconds,
    double decision_margin,
    int hamming = 0,
    int tag_id = 1,
    const std::string &family = "tag36h11")
{
    apriltag_msgs::msg::AprilTagDetectionArray msg;
    msg.header.stamp = stampFromSeconds(stamp_seconds);

    apriltag_msgs::msg::AprilTagDetection detection;
    detection.family = family;
    detection.id = tag_id;
    detection.hamming = hamming;
    detection.decision_margin = static_cast<float>(decision_margin);
    detection.goodness = 1.0F;
    msg.detections.push_back(detection);
    return msg;
}

inline tf2_ros::Buffer makeTfBuffer()
{
    return tf2_ros::Buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
}

inline std::string singleTagMapYaml(
    int tag_id = 1,
    const Eigen::Vector3d &translation = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond &rotation = Eigen::Quaterniond::Identity())
{
    std::ostringstream yaml;
    yaml << "tags:\n";
    yaml << "  - id: " << tag_id << "\n";
    yaml << "    position: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]\n";
    yaml << "    orientation_xyzw: ["
         << rotation.x() << ", " << rotation.y() << ", "
         << rotation.z() << ", " << rotation.w() << "]\n";
    return yaml.str();
}

} // namespace visual_inertial_localization::test
