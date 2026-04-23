#include "offline_global_graph/exporter.hpp"

#include <fstream>
#include <map>
#include <stdexcept>

namespace offline_global_graph
{

namespace
{

void writePoseCsv(
  std::ostream &os,
  const gtsam::Pose3 &pose)
{
  const gtsam::Point3 t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  os
    << t.x() << ","
    << t.y() << ","
    << t.z() << ","
    << q.x() << ","
    << q.y() << ","
    << q.z() << ","
    << q.w();
}

void writePoseYaml(
  std::ostream &os,
  const gtsam::Pose3 &pose,
  const std::string &indent)
{
  const gtsam::Point3 t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  os << indent << "position: [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
  os << indent << "orientation_xyzw: ["
     << q.x() << ", "
     << q.y() << ", "
     << q.z() << ", "
     << q.w() << "]\n";
}

}  // namespace

void writeOptimizationOutputs(
  const SessionData &session,
  const OptimizationResult &result,
  const std::filesystem::path &output_dir)
{
  std::filesystem::create_directories(output_dir);

  const auto keyframes_path = output_dir / "optimized_keyframes.csv";
  std::ofstream keyframes_os(keyframes_path, std::ios::out | std::ios::trunc);
  if (!keyframes_os.is_open()) {
    throw std::runtime_error("failed to open output file: " + keyframes_path.string());
  }

  keyframes_os
    << "kf_id,stamp_ns,opt_body_px,opt_body_py,opt_body_pz,opt_body_qx,opt_body_qy,opt_body_qz,opt_body_qw\n";
  for (size_t i = 0; i < result.optimized_keyframes.size(); ++i) {
    const auto &entry = result.optimized_keyframes[i];
    keyframes_os << entry.first << "," << session.keyframes[i].stamp_ns << ",";
    writePoseCsv(keyframes_os, entry.second);
    keyframes_os << "\n";
  }

  std::map<int, size_t> observation_counts;
  for (const auto &observation : session.tag_observations) {
    ++observation_counts[observation.tag_id];
  }

  const auto tags_path = output_dir / "optimized_tags.yaml";
  std::ofstream tags_os(tags_path, std::ios::out | std::ios::trunc);
  if (!tags_os.is_open()) {
    throw std::runtime_error("failed to open output file: " + tags_path.string());
  }

  tags_os << "tags:\n";
  for (const auto &entry : result.optimized_tags) {
    tags_os << "  - id: " << entry.first << "\n";
    tags_os << "    observation_count: " << observation_counts[entry.first] << "\n";
    writePoseYaml(tags_os, entry.second, "    ");
  }

  const auto summary_path = output_dir / "optimization_summary.yaml";
  std::ofstream summary_os(summary_path, std::ios::out | std::ios::trunc);
  if (!summary_os.is_open()) {
    throw std::runtime_error("failed to open output file: " + summary_path.string());
  }

  summary_os << "session_dir: \"" << session.session_dir.string() << "\"\n";
  summary_os << "session_name: \"" << session.session_name << "\"\n";
  summary_os << "body_frame_id: \"" << session.body_frame_id << "\"\n";
  summary_os << "keyframe_count: " << session.keyframes.size() << "\n";
  summary_os << "tag_observation_count: " << session.tag_observations.size() << "\n";
  summary_os << "skipped_missing_tf_tag_observations: "
             << session.skipped_missing_tf_tag_observations << "\n";
  summary_os << "skipped_unavailable_tag_observations: "
             << session.skipped_unavailable_tag_observations << "\n";
  summary_os << "skipped_non_body_tag_observations: "
             << session.skipped_non_body_tag_observations << "\n";
  summary_os << "between_factor_count: " << result.between_factor_count << "\n";
  summary_os << "tag_observation_factor_count: " << result.tag_observation_factor_count << "\n";
  summary_os << "tag_observation_skipped_distance_count: "
             << result.tag_observation_skipped_distance_count << "\n";
  summary_os << "tag_observation_skipped_oblique_count: "
             << result.tag_observation_skipped_oblique_count << "\n";
  summary_os << "prior_factor_count: " << result.prior_factor_count << "\n";
  summary_os << "initial_error: " << result.initial_error << "\n";
  summary_os << "final_error: " << result.final_error << "\n";
  summary_os << "anchor_strategy: \"" << result.anchor_strategy << "\"\n";
}

}  // namespace offline_global_graph
