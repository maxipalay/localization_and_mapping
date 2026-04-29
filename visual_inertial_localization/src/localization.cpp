#include <visual_inertial_localization/localization.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <tf2/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <exception>
#include <cmath>
#include <unordered_set>
#include <utility>

namespace visual_inertial_localization
{

namespace
{

Eigen::Isometry3d transformMsgToIso(const geometry_msgs::msg::Transform &t)
{
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    q.normalize();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(
        t.translation.x,
        t.translation.y,
        t.translation.z);
    return T;
}

double rotationDistanceRad(const Eigen::Matrix3d &R_a, const Eigen::Matrix3d &R_b)
{
    Eigen::Quaterniond q_rel(R_a.transpose() * R_b);
    q_rel.normalize();
    const double w = std::clamp(std::abs(q_rel.w()), 0.0, 1.0);
    return 2.0 * std::acos(w);
}

double poseTranslationDistance(const Eigen::Isometry3d &T_a, const Eigen::Isometry3d &T_b)
{
    return (T_a.translation() - T_b.translation()).norm();
}

double poseRotationDistanceRad(const Eigen::Isometry3d &T_a, const Eigen::Isometry3d &T_b)
{
    return rotationDistanceRad(T_a.linear(), T_b.linear());
}

} // namespace

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

TagIngestReport LocalizationModule::ingestDetections(
    const apriltag_msgs::msg::AprilTagDetectionArray &msg,
    const std::string &body_frame_id,
    const std::string &odom_frame_id,
    tf2_ros::Buffer &tf_buffer) const
{
    TagIngestReport report;
    report.total_detections = msg.detections.size();
    if (mapped_tags_.empty())
    {
        return report;
    }

    const int64_t stamp_ns =
        (static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL) +
        static_cast<int64_t>(msg.header.stamp.nanosec);
    const auto lookup_time = rclcpp::Time(msg.header.stamp);
    const auto lookup_timeout =
        rclcpp::Duration::from_seconds(config_.tag_tf_lookup_timeout_ms / 1000.0);
    std::vector<BufferedTagObservation> accepted_observations;
    accepted_observations.reserve(msg.detections.size());

    {
        std::lock_guard<std::mutex> lk(tag_obs_mtx_);
        for (const auto &detection : msg.detections)
        {
            if (mapped_tags_.find(static_cast<int>(detection.id)) == mapped_tags_.end())
            {
                ++report.skipped_unmapped;
                continue;
            }
            if (detection.hamming > config_.max_tag_hamming)
            {
                ++report.skipped_hamming;
                continue;
            }
            if (detection.decision_margin < config_.min_tag_decision_margin)
            {
                ++report.skipped_margin;
                continue;
            }

            try
            {
                const auto tf = tf_buffer.lookupTransform(
                    body_frame_id,
                    tagFrameName_(detection),
                    lookup_time,
                    lookup_timeout);
                const Eigen::Isometry3d T_BT = transformMsgToIso(tf.transform);
                const double range_m = T_BT.translation().norm();
                if (config_.max_tag_range_m > 0.0 &&
                    std::isfinite(range_m) &&
                    range_m > config_.max_tag_range_m)
                {
                    ++report.skipped_range;
                    continue;
                }
                if (isObliqueTagObservation_(T_BT))
                {
                    ++report.skipped_oblique;
                    continue;
                }

                BufferedTagObservation observation;
                observation.stamp_ns = stamp_ns;
                observation.tag_id = static_cast<int>(detection.id);
                observation.family = detection.family;
                observation.T_BT = T_BT;
                observation.decision_margin = detection.decision_margin;
                observation.goodness = detection.goodness;
                observation.hamming = detection.hamming;
                accepted_observations.push_back(observation);
                tag_observation_buffer_.push_back(std::move(observation));
                ++report.accepted;
            }
            catch (const tf2::TransformException &)
            {
                ++report.skipped_tf_lookup;
            }
        }

        pruneBufferedTagObservationsLocked_(stamp_ns);
        report.buffered = tag_observation_buffer_.size();
    }

    if (!accepted_observations.empty())
    {
        try
        {
            const auto tf_odom_body = tf_buffer.lookupTransform(
                odom_frame_id,
                body_frame_id,
                lookup_time,
                lookup_timeout);
            const Eigen::Isometry3d T_OB = transformMsgToIso(tf_odom_body.transform);
            const auto hypotheses = buildPoseHypotheses_(accepted_observations);
            const auto cluster_indices = dominantHypothesisCluster_(hypotheses);
            if (!cluster_indices.empty())
            {
                const size_t best_index = bestHypothesisIndex_(hypotheses, cluster_indices);
                const Eigen::Isometry3d T_MO = hypotheses[best_index].T_MB * T_OB.inverse();
                updateStableCorrection_(
                    stamp_ns,
                    T_MO,
                    hypotheses[best_index].score,
                    cluster_indices.size());
            }
        }
        catch (const tf2::TransformException &)
        {
        }
    }

    return report;
}

std::vector<BufferedTagObservation> LocalizationModule::recentObservationsForStamp(const rclcpp::Time &stamp) const
{
    std::vector<BufferedTagObservation> out;
    if (config_.tag_max_age_s <= 0.0)
    {
        return out;
    }

    const int64_t stamp_ns = stamp.nanoseconds();
    const int64_t max_age_ns = static_cast<int64_t>(config_.tag_max_age_s * 1e9);

    std::lock_guard<std::mutex> lk(tag_obs_mtx_);
    out.reserve(tag_observation_buffer_.size());
    for (const auto &observation : tag_observation_buffer_)
    {
        if (observation.stamp_ns > stamp_ns)
        {
            continue;
        }
        if ((stamp_ns - observation.stamp_ns) > max_age_ns)
        {
            continue;
        }
        out.push_back(observation);
    }
    return out;
}

std::optional<BootstrapEstimate> LocalizationModule::estimateBootstrap(
    const rclcpp::Time &stamp,
    const Eigen::Isometry3d &T_OB) const
{
    const auto stable_correction = estimateStableCorrection(stamp);
    if (stable_correction.has_value())
    {
        BootstrapEstimate estimate;
        estimate.T_MO = stable_correction->T_MO;
        estimate.support_count = stable_correction->support_count;
        estimate.score = stable_correction->score;
        return estimate;
    }

    const auto observations = recentObservationsForStamp(stamp);
    if (observations.empty())
    {
        return std::nullopt;
    }

    struct CorrectionHypothesis
    {
        Eigen::Isometry3d T_MO = Eigen::Isometry3d::Identity();
        double score{0.0};
    };

    std::vector<CorrectionHypothesis> hypotheses;
    hypotheses.reserve(observations.size());
    for (const auto &observation : observations)
    {
        const auto mapped_it = mapped_tags_.find(observation.tag_id);
        if (mapped_it == mapped_tags_.end())
        {
            continue;
        }

        CorrectionHypothesis h;
        h.T_MO = mapped_it->second * observation.T_BT.inverse() * T_OB.inverse();
        h.score = observation.decision_margin;
        hypotheses.push_back(std::move(h));
    }

    if (hypotheses.empty())
    {
        return std::nullopt;
    }

    const double rot_threshold_rad = config_.cluster_rotation_deg * std::acos(-1.0) / 180.0;
    size_t best_index = 0;
    size_t best_support = 0;
    double best_score = -1.0;
    for (size_t i = 0; i < hypotheses.size(); ++i)
    {
        size_t support = 0;
        double score = 0.0;
        for (size_t j = 0; j < hypotheses.size(); ++j)
        {
            const double trans_error =
                (hypotheses[i].T_MO.translation() - hypotheses[j].T_MO.translation()).norm();
            const double rot_error =
                rotationDistanceRad(hypotheses[i].T_MO.linear(), hypotheses[j].T_MO.linear());
            if (trans_error <= config_.cluster_translation_m &&
                rot_error <= rot_threshold_rad)
            {
                ++support;
                score += hypotheses[j].score;
            }
        }

        if (support > best_support || (support == best_support && score > best_score))
        {
            best_index = i;
            best_support = support;
            best_score = score;
        }
    }

    if (best_support < config_.bootstrap_min_inliers)
    {
        return std::nullopt;
    }

    BootstrapEstimate estimate;
    estimate.T_MO = hypotheses[best_index].T_MO;
    estimate.support_count = best_support;
    estimate.score = best_score;
    return estimate;
}

std::optional<PosePriorEstimate> LocalizationModule::estimatePosePrior(const rclcpp::Time &stamp) const
{
    const auto estimates = estimatePosePriors(stamp);
    if (estimates.empty())
    {
        return std::nullopt;
    }
    return estimates.front();
}

std::vector<PosePriorEstimate> LocalizationModule::estimatePosePriors(const rclcpp::Time &stamp) const
{
    const auto observations = recentObservationsForStamp(stamp);
    const auto hypotheses = buildPoseHypotheses_(observations);
    if (hypotheses.empty())
    {
        return {};
    }

    const auto cluster_indices = dominantHypothesisCluster_(hypotheses);
    if (cluster_indices.size() < config_.bootstrap_min_inliers)
    {
        return {};
    }

    std::vector<size_t> ordered_indices = cluster_indices;
    std::sort(
        ordered_indices.begin(),
        ordered_indices.end(),
        [&hypotheses](size_t lhs, size_t rhs)
        {
            return hypotheses[lhs].score > hypotheses[rhs].score;
        });

    std::unordered_set<int> used_tag_ids;
    std::vector<PosePriorEstimate> estimates;
    estimates.reserve(ordered_indices.size());
    for (const size_t idx : ordered_indices)
    {
        const auto &hypothesis = hypotheses[idx];
        if (!used_tag_ids.insert(hypothesis.tag_id).second)
        {
            continue;
        }

        PosePriorEstimate estimate;
        estimate.T_MB = hypothesis.T_MB;
        estimate.support_count = cluster_indices.size();
        estimate.score = hypothesis.score;
        estimates.push_back(std::move(estimate));
    }

    return estimates;
}

std::optional<StableCorrectionEstimate> LocalizationModule::estimateStableCorrection(const rclcpp::Time &stamp) const
{
    std::lock_guard<std::mutex> lk(stable_correction_mtx_);
    pruneTemporalCorrectionHypothesesLocked_(stamp.nanoseconds());
    return computeStableCorrectionLocked_();
}

size_t LocalizationModule::bufferedObservationCount() const noexcept
{
    std::lock_guard<std::mutex> lk(tag_obs_mtx_);
    return tag_observation_buffer_.size();
}

const LocalizationConfig &LocalizationModule::config() const noexcept
{
    return config_;
}

const std::unordered_map<int, Eigen::Isometry3d> &LocalizationModule::mappedTags() const noexcept
{
    return mapped_tags_;
}

std::string LocalizationModule::tagFrameName_(const apriltag_msgs::msg::AprilTagDetection &detection) const
{
    const auto override_it = config_.tag_frame_overrides.find(static_cast<int>(detection.id));
    if (override_it != config_.tag_frame_overrides.end())
    {
        return override_it->second;
    }
    return detection.family + ":" + std::to_string(detection.id);
}

bool LocalizationModule::isObliqueTagObservation_(const Eigen::Isometry3d &T_BT) const
{
    if (config_.max_tag_oblique_angle_deg <= 0.0)
    {
        return false;
    }

    const Eigen::Vector3d body_p_tag = T_BT.translation();
    const double range_m = body_p_tag.norm();
    if (!std::isfinite(range_m) || range_m <= 1e-6)
    {
        return false;
    }

    const double pi = std::acos(-1.0);
    const double max_angle_rad = config_.max_tag_oblique_angle_deg * pi / 180.0;
    const double min_abs_cosine = std::cos(std::clamp(max_angle_rad, 0.0, 0.5 * pi));
    const Eigen::Vector3d tag_normal_body = T_BT.linear() * Eigen::Vector3d::UnitZ();
    const double normal_norm = tag_normal_body.norm();
    if (!std::isfinite(normal_norm) || normal_norm <= 1e-6)
    {
        return false;
    }

    const double abs_cosine =
        std::abs(tag_normal_body.dot(-body_p_tag) / (normal_norm * range_m));
    return std::isfinite(abs_cosine) && abs_cosine < min_abs_cosine;
}

void LocalizationModule::pruneBufferedTagObservationsLocked_(int64_t newest_stamp_ns) const
{
    if (config_.tag_buffer_age_s <= 0.0)
    {
        tag_observation_buffer_.clear();
        return;
    }

    const int64_t max_age_ns = static_cast<int64_t>(config_.tag_buffer_age_s * 1e9);
    while (!tag_observation_buffer_.empty())
    {
        const auto &front = tag_observation_buffer_.front();
        if ((newest_stamp_ns - front.stamp_ns) <= max_age_ns)
        {
            break;
        }
        tag_observation_buffer_.pop_front();
    }
}

std::vector<LocalizationModule::PoseHypothesis> LocalizationModule::buildPoseHypotheses_(
    const std::vector<BufferedTagObservation> &observations) const
{
    std::vector<PoseHypothesis> hypotheses;
    hypotheses.reserve(observations.size());
    for (const auto &observation : observations)
    {
        const auto mapped_it = mapped_tags_.find(observation.tag_id);
        if (mapped_it == mapped_tags_.end())
        {
            continue;
        }

        PoseHypothesis h;
        h.tag_id = observation.tag_id;
        h.stamp_ns = observation.stamp_ns;
        h.T_MB = mapped_it->second * observation.T_BT.inverse();
        h.score = observation.decision_margin + 0.1 * observation.goodness;
        h.decision_margin = observation.decision_margin;
        hypotheses.push_back(std::move(h));
    }
    return hypotheses;
}

std::vector<size_t> LocalizationModule::dominantHypothesisCluster_(
    const std::vector<PoseHypothesis> &hypotheses) const
{
    std::vector<size_t> best_cluster;
    const double rot_threshold_rad = config_.cluster_rotation_deg * std::acos(-1.0) / 180.0;
    double best_score = -1.0;

    for (size_t i = 0; i < hypotheses.size(); ++i)
    {
        std::vector<size_t> cluster;
        double cluster_score = 0.0;
        for (size_t j = 0; j < hypotheses.size(); ++j)
        {
            if (poseTranslationDistance(hypotheses[i].T_MB, hypotheses[j].T_MB) >
                config_.cluster_translation_m)
            {
                continue;
            }
            if (poseRotationDistanceRad(hypotheses[i].T_MB, hypotheses[j].T_MB) >
                rot_threshold_rad)
            {
                continue;
            }
            cluster.push_back(j);
            cluster_score += hypotheses[j].score;
        }

        if (cluster.size() > best_cluster.size() ||
            (cluster.size() == best_cluster.size() && cluster_score > best_score))
        {
            best_score = cluster_score;
            best_cluster = std::move(cluster);
        }
    }

    return best_cluster;
}

size_t LocalizationModule::bestHypothesisIndex_(
    const std::vector<PoseHypothesis> &hypotheses,
    const std::vector<size_t> &cluster_indices) const
{
    size_t best_index = cluster_indices.front();
    for (const size_t idx : cluster_indices)
    {
        if (hypotheses[idx].score > hypotheses[best_index].score)
        {
            best_index = idx;
        }
    }
    return best_index;
}

void LocalizationModule::pruneTemporalCorrectionHypothesesLocked_(int64_t newest_stamp_ns) const
{
    if (config_.stable_hypothesis_age_s <= 0.0)
    {
        stable_correction_buffer_.clear();
        return;
    }

    const int64_t max_age_ns = static_cast<int64_t>(config_.stable_hypothesis_age_s * 1e9);
    while (!stable_correction_buffer_.empty())
    {
        const auto &front = stable_correction_buffer_.front();
        if ((newest_stamp_ns - front.stamp_ns) <= max_age_ns)
        {
            break;
        }
        stable_correction_buffer_.pop_front();
    }
}

std::optional<StableCorrectionEstimate> LocalizationModule::computeStableCorrectionLocked_() const
{
    if (stable_correction_buffer_.size() < config_.stable_min_frames)
    {
        return std::nullopt;
    }

    const double rot_threshold_rad = config_.stable_rotation_deg * std::acos(-1.0) / 180.0;
    std::vector<size_t> best_cluster;
    double best_score = -1.0;
    for (size_t i = 0; i < stable_correction_buffer_.size(); ++i)
    {
        std::vector<size_t> cluster;
        double cluster_score = 0.0;
        for (size_t j = 0; j < stable_correction_buffer_.size(); ++j)
        {
            if (poseTranslationDistance(
                    stable_correction_buffer_[i].T_MO,
                    stable_correction_buffer_[j].T_MO) > config_.stable_translation_m)
            {
                continue;
            }
            if (poseRotationDistanceRad(
                    stable_correction_buffer_[i].T_MO,
                    stable_correction_buffer_[j].T_MO) > rot_threshold_rad)
            {
                continue;
            }
            cluster.push_back(j);
            cluster_score += stable_correction_buffer_[j].score;
        }

        if (cluster.size() > best_cluster.size() ||
            (cluster.size() == best_cluster.size() && cluster_score > best_score))
        {
            best_score = cluster_score;
            best_cluster = std::move(cluster);
        }
    }

    if (best_cluster.size() < config_.stable_min_frames)
    {
        return std::nullopt;
    }

    size_t best_index = best_cluster.front();
    size_t max_support_count = 0;
    for (const size_t idx : best_cluster)
    {
        if (stable_correction_buffer_[idx].score > stable_correction_buffer_[best_index].score)
        {
            best_index = idx;
        }
        max_support_count = std::max(max_support_count, stable_correction_buffer_[idx].support_count);
    }

    StableCorrectionEstimate out;
    out.T_MO = stable_correction_buffer_[best_index].T_MO;
    out.score = stable_correction_buffer_[best_index].score;
    out.frame_support = best_cluster.size();
    out.support_count = max_support_count;
    return out;
}

std::optional<StableCorrectionEstimate> LocalizationModule::updateStableCorrection_(
    int64_t stamp_ns,
    const Eigen::Isometry3d &T_MO,
    double score,
    size_t support_count) const
{
    std::lock_guard<std::mutex> lk(stable_correction_mtx_);

    TemporalCorrectionHypothesis hypothesis;
    hypothesis.stamp_ns = stamp_ns;
    hypothesis.T_MO = T_MO;
    hypothesis.score = score;
    hypothesis.support_count = support_count;

    if (!stable_correction_buffer_.empty() &&
        stable_correction_buffer_.back().stamp_ns == stamp_ns)
    {
        stable_correction_buffer_.back() = std::move(hypothesis);
    }
    else
    {
        stable_correction_buffer_.push_back(std::move(hypothesis));
    }

    pruneTemporalCorrectionHypothesesLocked_(stamp_ns);
    return computeStableCorrectionLocked_();
}

} // namespace visual_inertial_localization
