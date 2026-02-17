#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <visual_inertial/msg/keyframe.hpp>

#include <visual_inertial_common/types.hpp>              
#include <visual_inertial_optimization/optimization.hpp> 
#include <visual_inertial_optimization/types.hpp>        

#include <opencv2/core.hpp>

#include <Eigen/Geometry>

#include <mutex>
#include <condition_variable>
#include <deque>
#include <thread>
#include <atomic>
#include <optional>
#include <memory>
#include <cmath>
#include <algorithm>

namespace {

static Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  q.normalize();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return T;
}

static geometry_msgs::msg::TransformStamped isoToTf(
    const Eigen::Isometry3d& T_parent_child,
    const rclcpp::Time& stamp,
    const std::string& parent_frame,
    const std::string& child_frame)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = T_parent_child.translation().x();
  tf.transform.translation.y = T_parent_child.translation().y();
  tf.transform.translation.z = T_parent_child.translation().z();

  Eigen::Quaterniond q(T_parent_child.linear());
  q.normalize();
  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();

  return tf;
}

static KeyframeEvent toKeyframeEvent(const visual_inertial::msg::Keyframe& msg) {
  KeyframeEvent ev;
  ev.kf_id = msg.kf_id;
  ev.t_s = rclcpp::Time(msg.header.stamp).seconds();

  // Frontend pose: World(odom) <- Camera(LEFT optical)
  ev.T_WC = poseMsgToIso(msg.pose_wc);

  const size_t n = msg.track_ids.size();
  // Basic sanity; if mismatched, drop (caller handles)
  if (msg.u_l.size() != n || msg.v_l.size() != n ||
      msg.u_r.size() != n || msg.v_r.size() != n ||
      msg.has_right.size() != n) {
    ev.ids.clear(); ev.pl.clear(); ev.pr.clear(); ev.has_r.clear();
    return ev;
  }

  ev.ids.resize(n);
  ev.pl.resize(n);
  ev.pr.resize(n);
  ev.has_r.resize(n);

  for (size_t i = 0; i < n; ++i) {
    ev.ids[i]   = msg.track_ids[i];
    ev.pl[i]    = cv::Point2f(msg.u_l[i], msg.v_l[i]);
    ev.pr[i]    = cv::Point2f(msg.u_r[i], msg.v_r[i]);
    ev.has_r[i] = msg.has_right[i];
  }

  return ev;
}

// Same strategy you already use in tracking node.
static CameraRig makeStereoModel(
    const sensor_msgs::msg::CameraInfo& L,
    const sensor_msgs::msg::CameraInfo& R)
{

  CameraRig rig;

  auto fetch = [](const sensor_msgs::msg::CameraInfo& info, int idx_p, int idx_k) -> double {
    return (info.p.size() == 12) ? info.p[idx_p] : info.k[idx_k];
  };

  rig.left.cam_id = 0;
  rig.left.K << fetch(L, 0, 0), 0.0,           fetch(L, 2, 2),
                0.0,           fetch(L, 5, 4), fetch(L, 6, 5),
                0.0,           0.0,           1.0;

  rig.right.cam_id = 1;
  rig.right.K << fetch(R, 0, 0), 0.0,           fetch(R, 2, 2),
                 0.0,           fetch(R, 5, 4), fetch(R, 6, 5),
                 0.0,           0.0,           1.0;

  // Baseline from right projection matrix: P = [fx 0 cx Tx*fx; ...]
  const double fx_r = rig.right.K(0, 0);
  double Tx = 0.0;
  if (R.p.size() == 12 && std::abs(fx_r) > 1e-9) {
    Tx = -R.p[3] / fx_r;
  }
  rig.baseline = std::abs(Tx);

  return rig;
}

} // namespace

class OptimizationNode final : public rclcpp::Node {
public:
  OptimizationNode()
  : rclcpp::Node("visual_inertial_optimization_node")
  {
    // -------- Parameters --------
    keyframe_topic_   = declare_parameter<std::string>("keyframe_topic", "keyframes");
    left_info_topic_  = declare_parameter<std::string>("left_info_topic", "oak/left/camera_info");
    right_info_topic_ = declare_parameter<std::string>("right_info_topic", "oak/right/camera_info");

    map_frame_id_  = declare_parameter<std::string>("map_frame_id", "map");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");

    // Backend config params
    cfg_.window_size = static_cast<size_t>(declare_parameter<int>("window_size", 8));
    cfg_.stereo_sigma_px = declare_parameter<double>("stereo_sigma_px", 1.0);

    cfg_.prior_rot_sigma_rad = declare_parameter<double>("prior_rot_sigma_rad", 5.0 * M_PI / 180.0);
    cfg_.prior_trans_sigma_m = declare_parameter<double>("prior_trans_sigma_m", 0.25);

    cfg_.use_vo_between = declare_parameter<bool>("use_vo_between", true);
    cfg_.between_rot_sigma_rad = declare_parameter<double>("between_rot_sigma_rad", 3.0 * M_PI / 180.0);
    cfg_.between_trans_sigma_m = declare_parameter<double>("between_trans_sigma_m", 0.10);

    cfg_.init_landmarks_from_stereo = declare_parameter<bool>("init_landmarks_from_stereo", true);
    cfg_.prune_unobserved_landmarks = declare_parameter<bool>("prune_unobserved_landmarks", true);

    max_queue_ = static_cast<size_t>(declare_parameter<int>("max_keyframe_queue", 30));

    // -------- TF broadcaster --------
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // -------- QoS --------
    // Keyframes are low-rate and important: reliable, small queue.
    auto kf_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    // CameraInfo often comes at image rate; reliable is fine.
    // If your CameraInfo is published with transient_local durability and you want "latched" behavior,
    // change to: .transient_local()
    auto info_qos = rclcpp::SensorDataQoS();

    // -------- Subscriptions: CameraInfo (Option A) --------
    left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      left_info_topic_, info_qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        left_info_ = *msg;
        maybeInitRigAndOptimizer_();
      });

    right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      right_info_topic_, info_qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        right_info_ = *msg;
        maybeInitRigAndOptimizer_();
      });

    // -------- Subscription: Keyframes --------
    kf_sub_ = create_subscription<visual_inertial::msg::Keyframe>(
      keyframe_topic_, kf_qos,
      [this](visual_inertial::msg::Keyframe::SharedPtr msg) {
        // Hot path: enqueue + return
        {
          std::lock_guard<std::mutex> lk(q_mtx_);
          kf_q_.push_back(*msg);
          while (kf_q_.size() > max_queue_) kf_q_.pop_front();
        }
        q_cv_.notify_one();
      });

    // -------- Worker thread --------
    stop_.store(false);
    worker_ = std::thread([this]() { workerLoop_(); });

    RCLCPP_INFO(get_logger(),
                "Optimization node started. Subscribing: KF=%s, L_info=%s, R_info=%s. Broadcasting TF %s->%s",
                keyframe_topic_.c_str(), left_info_topic_.c_str(), right_info_topic_.c_str(),
                map_frame_id_.c_str(), odom_frame_id_.c_str());
  }

  ~OptimizationNode() override {
    stop_.store(true);
    q_cv_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

private:
  void maybeInitRigAndOptimizer_() {
    // calib_mtx_ must be held by caller
    if (rig_ready_) return;
    if (!left_info_.has_value() || !right_info_.has_value()) return;

    const auto rig = makeStereoModel(*left_info_, *right_info_);
    if (!rig.valid()) {
      RCLCPP_WARN(get_logger(), "Stereo rig computed but invalid (baseline/intrinsics). Waiting...");
      return;
    }

    cfg_.rig = rig;

    // Create optimizer now that we have rig.
    {
      std::lock_guard<std::mutex> lk(opt_mtx_);
      optimizer_ = std::make_shared<Optimizer>(cfg_);
    }
    rig_ready_ = true;

    RCLCPP_INFO(get_logger(),
                "Stereo rig ready: fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.5f (m). Optimizer initialized.",
                rig.left.fx(), rig.left.fy(), rig.left.cx(), rig.left.cy(), rig.baseline);
  }

  void workerLoop_() {
    while (rclcpp::ok() && !stop_.load()) {
      visual_inertial::msg::Keyframe msg;

      // Wait for a keyframe
      {
        std::unique_lock<std::mutex> lk(q_mtx_);
        q_cv_.wait(lk, [&] { return stop_.load() || !kf_q_.empty(); });
        if (stop_.load()) break;

        msg = std::move(kf_q_.front());
        kf_q_.pop_front();
      }

      // If not calibrated yet, just keep draining slowly (or you can re-enqueue).
      if (!rig_ready_) {
        // Avoid spamming logs; only occasionally.
        static uint64_t warn_ctr = 0;
        if ((warn_ctr++ % 50) == 0) {
          RCLCPP_WARN(get_logger(), "Waiting for CameraInfo calibration; dropping keyframes for now...");
        }
        continue;
      }

      // Grab optimizer instance (thread-safe)
      std::shared_ptr<Optimizer> opt;
      {
        std::lock_guard<std::mutex> lk(opt_mtx_);
        opt = optimizer_;
      }
      if (!opt) continue;

      // Convert KF msg -> KeyframeEvent
      const auto ev = toKeyframeEvent(msg);
      if (ev.ids.empty()) continue; // conversion failed or empty KF

      // If you later add VO delta to your message, fill it here.
      std::optional<Eigen::Isometry3d> T_Ck_Ckm1 = std::nullopt;

      // Run backend update (heavy work off callback thread)
      const auto res = opt->push(ev, T_Ck_Ckm1);
      if (!res) continue;

      // Compute and broadcast map -> odom
      //
      // Treat frontend pose_wc as odom <- camera (drifty VO world)
      // Backend returns map <- camera (optimized)
      const Eigen::Isometry3d T_odom_C = poseMsgToIso(msg.pose_wc);
      const Eigen::Isometry3d T_map_C  = res->T_WC_opt;

      const Eigen::Isometry3d T_map_odom = T_map_C * T_odom_C.inverse();
      const auto tf = isoToTf(T_map_odom, msg.header.stamp, map_frame_id_, odom_frame_id_);
      tf_broadcaster_->sendTransform(tf);
    }
  }

private:
  // Topics / frames
  std::string keyframe_topic_;
  std::string left_info_topic_;
  std::string right_info_topic_;
  std::string map_frame_id_;
  std::string odom_frame_id_;

  // Subscriptions
  rclcpp::Subscription<visual_inertial::msg::Keyframe>::SharedPtr kf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Calibration state (Option A)
  std::mutex calib_mtx_;
  std::optional<sensor_msgs::msg::CameraInfo> left_info_;
  std::optional<sensor_msgs::msg::CameraInfo> right_info_;
  std::atomic<bool> rig_ready_{false};

  // Backend
  OptimizationConfig cfg_;
  std::mutex opt_mtx_;
  std::shared_ptr<Optimizer> optimizer_;

  // Queue / threading
  std::mutex q_mtx_;
  std::condition_variable q_cv_;
  std::deque<visual_inertial::msg::Keyframe> kf_q_;
  size_t max_queue_{30};

  std::thread worker_;
  std::atomic<bool> stop_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptimizationNode>());
  rclcpp::shutdown();
  return 0;
}
