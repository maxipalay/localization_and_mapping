#include <rclcpp/rclcpp.hpp>

#include <visual_inertial/msg/tracks.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/imgproc.hpp>

#include <deque>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <optional>

class TracksVizNode final : public rclcpp::Node
{
public:
  TracksVizNode()
  : rclcpp::Node("tracks_viz_node")
  {
    image_topic_  = declare_parameter<std::string>("image_topic", "oak/left/image_rect");
    tracks_topic_ = declare_parameter<std::string>("tracks_topic", "tracks");
    output_topic_ = declare_parameter<std::string>("output_topic", "tracks_overlay");

    // Input transport plugin: "raw", "compressed", or your plugin name (e.g. "ffmpeg")
    image_transport_in_ = declare_parameter<std::string>("image_transport_in", "compressed");

    // Match tolerance in ms between image stamp and tracks stamp
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 10.0);

    // Queue bounds
    max_images_ = static_cast<size_t>(declare_parameter<int>("max_images", 30));
    max_tracks_ = static_cast<size_t>(declare_parameter<int>("max_tracks", 60));

    // Draw params
    point_radius_px_ = declare_parameter<int>("point_radius_px", 2);
    point_thickness_ = declare_parameter<int>("point_thickness", -1); // -1 filled
    draw_antialias_  = declare_parameter<bool>("draw_antialias", true);

    // Optional publish rate limit (0 = publish every match)
    max_pub_hz_ = declare_parameter<double>("max_pub_hz", 0.0);

    overlay_pub_ = image_transport::create_publisher(this, output_topic_);

    // Tracks sub
    tracks_sub_ = create_subscription<visual_inertial::msg::Tracks>(
      tracks_topic_, rclcpp::SensorDataQoS(),
      [this](visual_inertial::msg::Tracks::SharedPtr msg)
      {
        onTracks_(std::move(msg));
      });

    // Image sub via image_transport (will decode compressed/ffmpeg into sensor_msgs/Image for you)
    image_sub_ = image_transport::create_subscription(
      this,
      image_topic_,
      std::bind(&TracksVizNode::onImage_, this, std::placeholders::_1),
      image_transport_in_,
      rclcpp::SensorDataQoS().get_rmw_qos_profile());

    RCLCPP_INFO(get_logger(),
      "TracksVizNode started. image=%s (%s), tracks=%s, output=%s, tol=%.1fms",
      image_topic_.c_str(), image_transport_in_.c_str(),
      tracks_topic_.c_str(), output_topic_.c_str(), sync_tolerance_ms_);

    // Helpful: confirm subscriptions exist (throttled prints in callbacks too)
  }

private:
  static double stampToSec(const builtin_interfaces::msg::Time& t)
  {
    return rclcpp::Time(t).seconds();
  }

  bool shouldPublishNow_(const rclcpp::Time& stamp)
  {
    if (max_pub_hz_ <= 0.0) return true;
    const double min_dt = 1.0 / std::max(1e-6, max_pub_hz_);
    if (!last_pub_stamp_) { last_pub_stamp_ = stamp; return true; }
    const double dt = (stamp - *last_pub_stamp_).seconds();
    if (dt >= min_dt) { last_pub_stamp_ = stamp; return true; }
    return false;
  }

  void onTracks_(visual_inertial::msg::Tracks::SharedPtr msg)
  {
    if (msg->u_l.size() != msg->v_l.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Tracks msg mismatch u_l/v_l sizes (%zu vs %zu) - dropping",
        msg->u_l.size(), msg->v_l.size());
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      tracks_q_.push_back(std::move(msg));
      while (tracks_q_.size() > max_tracks_) tracks_q_.pop_front();
    }

    // Try to match+publish immediately (tracks usually arrive after image)
    tryMatchAndPublish_();
  }

  void onImage_(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      images_q_.push_back(msg);
      while (images_q_.size() > max_images_) images_q_.pop_front();
    }

    tryMatchAndPublish_();
  }

  void tryMatchAndPublish_()
  {
    // We do matching inside a lock only long enough to pick a pair.
    // Then we render/publish outside the lock.
    sensor_msgs::msg::Image::ConstSharedPtr img_msg;
    visual_inertial::msg::Tracks::SharedPtr tracks_msg;

    const double tol_s = sync_tolerance_ms_ * 1e-3;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (images_q_.empty() || tracks_q_.empty())
        return;

      // Both queues are time-ordered by arrival; stamps should also be increasing.
      // Match oldest tracks to closest image (typical: image arrives first, then tracks).
      // Drop stale items that can never match.

      while (!images_q_.empty() && !tracks_q_.empty())
      {
        const double t_img = stampToSec(images_q_.front()->header.stamp);
        const double t_trk = stampToSec(tracks_q_.front()->header.stamp);

        // If image is too old relative to tracks, drop image
        if (t_img < t_trk - tol_s)
        {
          images_q_.pop_front();
          continue;
        }

        // If tracks are too old relative to image, drop tracks
        if (t_trk < t_img - tol_s)
        {
          tracks_q_.pop_front();
          continue;
        }

        // Now |t_img - t_trk| <= tol_s (match!)
        img_msg = images_q_.front();
        tracks_msg = tracks_q_.front();
        images_q_.pop_front();
        tracks_q_.pop_front();
        break;
      }
    }

    if (!img_msg || !tracks_msg)
      return;

    const rclcpp::Time stamp(img_msg->header.stamp);
    if (!shouldPublishNow_(stamp))
      return;

    // Convert and draw
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img_msg, img_msg->encoding);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge toCvShare failed: %s", e.what());
      return;
    }

    cv::Mat vis;
    if (cv_ptr->image.channels() == 1) {
      cv::cvtColor(cv_ptr->image, vis, cv::COLOR_GRAY2BGR);
    } else if (cv_ptr->image.channels() == 3) {
      vis = cv_ptr->image.clone();
    } else {
      cv::Mat tmp;
      cv_ptr->image.convertTo(tmp, CV_8U);
      cv::cvtColor(tmp, vis, cv::COLOR_GRAY2BGR);
    }

    const int line_type = draw_antialias_ ? cv::LINE_AA : cv::LINE_8;
    const size_t n = tracks_msg->u_l.size();

    for (size_t i = 0; i < n; ++i)
    {
      const float u = tracks_msg->u_l[i];
      const float v = tracks_msg->v_l[i];
      if (u < 0.f || v < 0.f || u >= (float)vis.cols || v >= (float)vis.rows) continue;
      cv::circle(vis, cv::Point2f(u, v), point_radius_px_, cv::Scalar(0, 255, 0),
                 point_thickness_, line_type);
    }

    cv_bridge::CvImage out;
    out.header = img_msg->header;
    out.encoding = "bgr8";
    out.image = std::move(vis);
    overlay_pub_.publish(out.toImageMsg());
  }

private:
  std::string image_topic_;
  std::string tracks_topic_;
  std::string output_topic_;
  std::string image_transport_in_;

  double sync_tolerance_ms_{10.0};
  size_t max_images_{30};
  size_t max_tracks_{60};

  int point_radius_px_{2};
  int point_thickness_{-1};
  bool draw_antialias_{true};

  double max_pub_hz_{0.0};
  std::optional<rclcpp::Time> last_pub_stamp_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<visual_inertial::msg::Tracks>::SharedPtr tracks_sub_;
  image_transport::Publisher overlay_pub_;

  std::mutex mtx_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_q_;
  std::deque<visual_inertial::msg::Tracks::SharedPtr> tracks_q_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TracksVizNode>());
  rclcpp::shutdown();
  return 0;
}