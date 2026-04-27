#include <visual_inertial_localization/localization.hpp>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <utility>

namespace visual_inertial_localization
{

LocalizationModule::LocalizationModule(LocalizationConfig config)
    : config_(std::move(config))
{
}

LocalizationLoadReport LocalizationModule::loadTagMap()
{
    mapped_tags_.clear();

    LocalizationLoadReport report;
    report.frame_override_count = config_.tag_frame_overrides.size();

    if (config_.tag_map_path.empty())
    {
        report.message = "localization_tag_map_path is empty";
        return report;
    }

    try
    {
        const YAML::Node root = YAML::LoadFile(config_.tag_map_path);
        const YAML::Node tags = root["tags"];
        if (!tags || !tags.IsSequence())
        {
            report.message = "tag map is missing a 'tags' sequence";
            return report;
        }

        for (const auto &tag_node : tags)
        {
            if (!tag_node["id"] || !tag_node["position"] || !tag_node["orientation_xyzw"])
            {
                continue;
            }

            const auto position = tag_node["position"];
            const auto orientation = tag_node["orientation_xyzw"];
            if (!position.IsSequence() || position.size() != 3 ||
                !orientation.IsSequence() || orientation.size() != 4)
            {
                continue;
            }

            Eigen::Quaterniond q(
                orientation[3].as<double>(),
                orientation[0].as<double>(),
                orientation[1].as<double>(),
                orientation[2].as<double>());
            if (q.norm() <= 1e-9)
            {
                continue;
            }
            q.normalize();

            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() = q.toRotationMatrix();
            T.translation() = Eigen::Vector3d(
                position[0].as<double>(),
                position[1].as<double>(),
                position[2].as<double>());
            mapped_tags_[tag_node["id"].as<int>()] = T;
        }
    }
    catch (const std::exception &e)
    {
        report.message = e.what();
        mapped_tags_.clear();
        return report;
    }

    report.ok = !mapped_tags_.empty();
    report.mapped_tag_count = mapped_tags_.size();
    report.message = report.ok ? "loaded" : "tag map did not contain any valid tags";
    return report;
}

const LocalizationConfig &LocalizationModule::config() const noexcept
{
    return config_;
}

const std::unordered_map<int, Eigen::Isometry3d> &LocalizationModule::mappedTags() const noexcept
{
    return mapped_tags_;
}

} // namespace visual_inertial_localization
