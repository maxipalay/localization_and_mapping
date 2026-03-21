#include "offline_global_graph/optimizer.hpp"
#include "offline_global_graph/session_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct DebugCliConfig
{
  std::filesystem::path session_dir;
  offline_global_graph::OptimizerConfig optimizer_config;
  std::optional<uint64_t> kf_id;
  std::optional<int> tag_id;
  std::filesystem::path csv_output_path;
  std::vector<int> only_tag_ids;
  std::vector<int> exclude_tag_ids;
  size_t max_rows{20};
  bool sort_descending{true};
};

struct ResidualRow
{
  uint64_t kf_id{0};
  int tag_id{0};
  std::string parent_frame_id;
  std::string child_frame_id;
  std::string detection_frame_id;
  int64_t lookup_stamp_ns{0};
  gtsam::Pose3 logged_body_T_tag;
  gtsam::Pose3 predicted_body_T_tag;
  gtsam::Pose3 residual;
  double translation_error_m{0.0};
  double rotation_error_deg{0.0};
};

void printUsage()
{
  std::cerr
    << "Usage: offline_global_graph_debug_cli --session-dir PATH\n"
    << "       [--kf-id INTEGER] [--tag-id INTEGER] [--max-rows INTEGER]\n"
    << "       [--only-tag-id INTEGER] [--exclude-tag-id INTEGER]\n"
    << "       [--csv-output PATH]\n"
    << "       [--sort ascending|descending]\n"
    << "       [--between-translation-sigma FLOAT] [--between-rotation-sigma FLOAT]\n"
    << "       [--tag-translation-sigma FLOAT] [--tag-rotation-sigma FLOAT]\n"
    << "       [--soft-prior-translation-sigma FLOAT] [--soft-prior-rotation-sigma FLOAT]\n"
    << "       [--anchor-translation-sigma FLOAT] [--anchor-rotation-sigma FLOAT]\n"
    << "       [--robust-huber-k FLOAT]\n";
}

std::string requireValue(int argc, char **argv, int &index, const char *flag)
{
  if ((index + 1) >= argc) {
    throw std::runtime_error(std::string("missing value for ") + flag);
  }
  ++index;
  return argv[index];
}

double parseCliDouble(const std::string &value, const char *flag)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse numeric value for ") + flag);
  }
}

uint64_t parseCliUint64(const std::string &value, const char *flag)
{
  try {
    return static_cast<uint64_t>(std::stoull(value));
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse integer value for ") + flag);
  }
}

int parseCliInt(const std::string &value, const char *flag)
{
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("failed to parse integer value for ") + flag);
  }
}

double rotationAngleDeg(const gtsam::Rot3 &rotation)
{
  constexpr double kRadToDeg = 57.2957795130823208768;
  const auto q = rotation.toQuaternion();
  const double w = std::clamp(std::abs(q.w()), 0.0, 1.0);
  return 2.0 * std::acos(w) * kRadToDeg;
}

std::string poseSummary(const gtsam::Pose3 &pose)
{
  const auto t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "p=[" << t.x() << ", " << t.y() << ", " << t.z() << "] "
      << "q_xyzw=[" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]";
  return oss.str();
}

bool tagAllowed(
  int tag_id,
  const std::vector<int> &only_tag_ids,
  const std::vector<int> &exclude_tag_ids)
{
  if (!only_tag_ids.empty() &&
      std::find(only_tag_ids.begin(), only_tag_ids.end(), tag_id) == only_tag_ids.end()) {
    return false;
  }

  return std::find(exclude_tag_ids.begin(), exclude_tag_ids.end(), tag_id) == exclude_tag_ids.end();
}

offline_global_graph::SessionData filterSessionByTags(
  const offline_global_graph::SessionData &session,
  const std::vector<int> &only_tag_ids,
  const std::vector<int> &exclude_tag_ids)
{
  if (only_tag_ids.empty() && exclude_tag_ids.empty()) {
    return session;
  }

  offline_global_graph::SessionData filtered = session;
  filtered.tag_observations.clear();
  filtered.tag_priors.clear();

  for (const auto &observation : session.tag_observations) {
    if (tagAllowed(observation.tag_id, only_tag_ids, exclude_tag_ids)) {
      filtered.tag_observations.push_back(observation);
    }
  }

  for (const auto &prior : session.tag_priors) {
    if (tagAllowed(prior.tag_id, only_tag_ids, exclude_tag_ids)) {
      filtered.tag_priors.push_back(prior);
    }
  }

  return filtered;
}

void writePoseCsv(std::ostream &os, const gtsam::Pose3 &pose)
{
  const auto t = pose.translation();
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

void writeResidualCsv(
  const std::filesystem::path &path,
  const std::vector<ResidualRow> &rows)
{
  std::ofstream os(path, std::ios::out | std::ios::trunc);
  if (!os.is_open()) {
    throw std::runtime_error("failed to open residual CSV output: " + path.string());
  }

  os
    << "kf_id,tag_id,parent_frame_id,child_frame_id,detection_frame_id,lookup_stamp_ns,"
    << "translation_error_m,rotation_error_deg,"
    << "logged_px,logged_py,logged_pz,logged_qx,logged_qy,logged_qz,logged_qw,"
    << "predicted_px,predicted_py,predicted_pz,predicted_qx,predicted_qy,predicted_qz,predicted_qw,"
    << "residual_px,residual_py,residual_pz,residual_qx,residual_qy,residual_qz,residual_qw\n";

  for (const auto &row : rows) {
    os
      << row.kf_id << ","
      << row.tag_id << ","
      << row.parent_frame_id << ","
      << row.child_frame_id << ","
      << row.detection_frame_id << ","
      << row.lookup_stamp_ns << ","
      << row.translation_error_m << ","
      << row.rotation_error_deg << ",";
    writePoseCsv(os, row.logged_body_T_tag);
    os << ",";
    writePoseCsv(os, row.predicted_body_T_tag);
    os << ",";
    writePoseCsv(os, row.residual);
    os << "\n";
  }
}

DebugCliConfig parseArgs(int argc, char **argv)
{
  DebugCliConfig cfg;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--session-dir") {
      cfg.session_dir = requireValue(argc, argv, i, "--session-dir");
    } else if (arg == "--kf-id") {
      cfg.kf_id = parseCliUint64(requireValue(argc, argv, i, "--kf-id"), "--kf-id");
    } else if (arg == "--tag-id") {
      cfg.tag_id = parseCliInt(requireValue(argc, argv, i, "--tag-id"), "--tag-id");
    } else if (arg == "--only-tag-id") {
      cfg.only_tag_ids.push_back(
        parseCliInt(requireValue(argc, argv, i, "--only-tag-id"), "--only-tag-id"));
    } else if (arg == "--exclude-tag-id") {
      cfg.exclude_tag_ids.push_back(
        parseCliInt(requireValue(argc, argv, i, "--exclude-tag-id"), "--exclude-tag-id"));
    } else if (arg == "--csv-output") {
      cfg.csv_output_path = requireValue(argc, argv, i, "--csv-output");
    } else if (arg == "--max-rows") {
      cfg.max_rows = static_cast<size_t>(parseCliUint64(requireValue(argc, argv, i, "--max-rows"), "--max-rows"));
    } else if (arg == "--sort") {
      const auto value = requireValue(argc, argv, i, "--sort");
      if (value == "descending" || value == "desc") {
        cfg.sort_descending = true;
      } else if (value == "ascending" || value == "asc") {
        cfg.sort_descending = false;
      } else {
        throw std::runtime_error("invalid value for --sort: " + value);
      }
    } else if (arg == "--between-translation-sigma") {
      cfg.optimizer_config.between_translation_sigma_m =
        parseCliDouble(requireValue(argc, argv, i, "--between-translation-sigma"), "--between-translation-sigma");
    } else if (arg == "--between-rotation-sigma") {
      cfg.optimizer_config.between_rotation_sigma_rad =
        parseCliDouble(requireValue(argc, argv, i, "--between-rotation-sigma"), "--between-rotation-sigma");
    } else if (arg == "--tag-translation-sigma") {
      cfg.optimizer_config.tag_translation_sigma_m =
        parseCliDouble(requireValue(argc, argv, i, "--tag-translation-sigma"), "--tag-translation-sigma");
    } else if (arg == "--tag-rotation-sigma") {
      cfg.optimizer_config.tag_rotation_sigma_rad =
        parseCliDouble(requireValue(argc, argv, i, "--tag-rotation-sigma"), "--tag-rotation-sigma");
    } else if (arg == "--soft-prior-translation-sigma") {
      cfg.optimizer_config.soft_prior_translation_sigma_m =
        parseCliDouble(requireValue(argc, argv, i, "--soft-prior-translation-sigma"), "--soft-prior-translation-sigma");
    } else if (arg == "--soft-prior-rotation-sigma") {
      cfg.optimizer_config.soft_prior_rotation_sigma_rad =
        parseCliDouble(requireValue(argc, argv, i, "--soft-prior-rotation-sigma"), "--soft-prior-rotation-sigma");
    } else if (arg == "--anchor-translation-sigma") {
      cfg.optimizer_config.anchor_translation_sigma_m =
        parseCliDouble(requireValue(argc, argv, i, "--anchor-translation-sigma"), "--anchor-translation-sigma");
    } else if (arg == "--anchor-rotation-sigma") {
      cfg.optimizer_config.anchor_rotation_sigma_rad =
        parseCliDouble(requireValue(argc, argv, i, "--anchor-rotation-sigma"), "--anchor-rotation-sigma");
    } else if (arg == "--robust-huber-k") {
      cfg.optimizer_config.robust_huber_k =
        parseCliDouble(requireValue(argc, argv, i, "--robust-huber-k"), "--robust-huber-k");
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (cfg.session_dir.empty()) {
    throw std::runtime_error("missing required --session-dir");
  }

  return cfg;
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    const DebugCliConfig cfg = parseArgs(argc, argv);

    auto session = offline_global_graph::loadSession(cfg.session_dir);
    session = filterSessionByTags(session, cfg.only_tag_ids, cfg.exclude_tag_ids);
    const auto result = offline_global_graph::optimizeSession(session, cfg.optimizer_config);

    std::unordered_map<uint64_t, gtsam::Pose3> world_T_body_by_kf;
    for (const auto &entry : result.optimized_keyframes) {
      world_T_body_by_kf.emplace(entry.first, entry.second);
    }

    std::unordered_map<int, gtsam::Pose3> world_T_tag_by_id;
    for (const auto &entry : result.optimized_tags) {
      world_T_tag_by_id.emplace(entry.first, entry.second);
    }

    std::vector<ResidualRow> rows;
    rows.reserve(session.tag_observations.size());

    for (const auto &observation : session.tag_observations) {
      if (cfg.kf_id && observation.kf_id != *cfg.kf_id) {
        continue;
      }
      if (cfg.tag_id && observation.tag_id != *cfg.tag_id) {
        continue;
      }

      const auto kf_it = world_T_body_by_kf.find(observation.kf_id);
      const auto tag_it = world_T_tag_by_id.find(observation.tag_id);
      if (kf_it == world_T_body_by_kf.end() || tag_it == world_T_tag_by_id.end()) {
        continue;
      }

      ResidualRow row;
      row.kf_id = observation.kf_id;
      row.tag_id = observation.tag_id;
      row.parent_frame_id = observation.parent_frame_id;
      row.child_frame_id = observation.child_frame_id;
      row.detection_frame_id = observation.detection_frame_id;
      row.lookup_stamp_ns = observation.lookup_stamp_ns;
      row.logged_body_T_tag = observation.body_T_tag;
      row.predicted_body_T_tag = kf_it->second.inverse().compose(tag_it->second);
      row.residual = observation.body_T_tag.between(row.predicted_body_T_tag);
      row.translation_error_m = row.residual.translation().norm();
      row.rotation_error_deg = rotationAngleDeg(row.residual.rotation());
      rows.push_back(std::move(row));
    }

    std::sort(
      rows.begin(),
      rows.end(),
      [&cfg](const ResidualRow &lhs, const ResidualRow &rhs) {
        const bool less = lhs.translation_error_m < rhs.translation_error_m;
        return cfg.sort_descending ? !less : less;
      });

    double max_translation = 0.0;
    double max_rotation = 0.0;
    double sum_translation = 0.0;
    double sum_rotation = 0.0;
    for (const auto &row : rows) {
      max_translation = std::max(max_translation, row.translation_error_m);
      max_rotation = std::max(max_rotation, row.rotation_error_deg);
      sum_translation += row.translation_error_m;
      sum_rotation += row.rotation_error_deg;
    }

    std::cout
      << "Debugged session '" << session.session_name << "'\n"
      << "  observations_considered: " << rows.size() << "\n"
      << "  initial_error: " << result.initial_error << "\n"
      << "  final_error: " << result.final_error << "\n"
      << "  anchor_strategy: " << result.anchor_strategy << "\n";

    if (rows.empty()) {
      std::cout << "  no matching observations found\n";
      return 0;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "  mean_translation_error_m: " << (sum_translation / static_cast<double>(rows.size())) << "\n"
              << "  mean_rotation_error_deg: " << (sum_rotation / static_cast<double>(rows.size())) << "\n"
              << "  max_translation_error_m: " << max_translation << "\n"
              << "  max_rotation_error_deg: " << max_rotation << "\n";

    if (!cfg.csv_output_path.empty()) {
      writeResidualCsv(cfg.csv_output_path, rows);
      std::cout << "  csv_output: " << cfg.csv_output_path.string() << "\n";
    }

    const size_t rows_to_print = std::min(cfg.max_rows, rows.size());
    std::cout << "\nTop residuals:\n";
    for (size_t i = 0; i < rows_to_print; ++i) {
      const auto &row = rows[i];
      std::cout
        << "  kf_id=" << row.kf_id
        << " tag_id=" << row.tag_id
        << " trans_err_m=" << row.translation_error_m
        << " rot_err_deg=" << row.rotation_error_deg
        << " parent='" << row.parent_frame_id
        << "' child='" << row.child_frame_id
        << "' detection_frame='" << row.detection_frame_id
        << "' lookup_stamp_ns=" << row.lookup_stamp_ns << "\n"
        << "    logged:    " << poseSummary(row.logged_body_T_tag) << "\n"
        << "    predicted: " << poseSummary(row.predicted_body_T_tag) << "\n"
        << "    residual:  " << poseSummary(row.residual) << "\n";
    }

    return 0;
  } catch (const std::exception &ex) {
    std::cerr << "offline_global_graph_debug_cli: " << ex.what() << "\n";
    return 1;
  }
}
