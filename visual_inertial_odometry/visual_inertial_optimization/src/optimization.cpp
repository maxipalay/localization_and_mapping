#include "visual_inertial_optimization/optimization.hpp"
#include "visual_inertial_optimization/optimizer.hpp"

bool OptimizationModule::ready() const noexcept
{
  std::lock_guard<std::mutex> lk(mtx_);
  return static_cast<bool>(optimizer_);
}

void OptimizationModule::setBodyCameraExtrinsic(const Eigen::Isometry3d &T_BC)
{
  std::lock_guard<std::mutex> lk(mtx_);
  config_.T_BC = T_BC;
  if (optimizer_)
  {
    optimizer_->setConfig(config_);
  }
}

bool OptimizationModule::initializeRig(const CameraRig &rig)
{
  if (!rig.valid())
  {
    return false;
  }

  std::lock_guard<std::mutex> lk(mtx_);
  config_.rig = rig;
  optimizer_ = std::make_shared<Optimizer>(config_);
  return true;
}

void OptimizationModule::reset()
{
  std::shared_ptr<Optimizer> optimizer;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    optimizer = optimizer_;
  }
  if (optimizer)
  {
    optimizer->reset();
  }
}

std::optional<OptimizationResult> OptimizationModule::push(
    const KeyframeEvent &kf,
    const std::optional<Eigen::Isometry3d> &T_Bkm1_Bk_meas,
    const std::vector<AbsolutePosePrior> &absolute_pose_priors,
    const std::optional<Eigen::Isometry3d> &T_WB_init_override,
    const std::optional<Eigen::Isometry3d> &T_WB_anchor_override)
{
  std::shared_ptr<Optimizer> optimizer;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    optimizer = optimizer_;
  }
  if (!optimizer)
  {
    return std::nullopt;
  }

  return optimizer->push(
      kf,
      T_Bkm1_Bk_meas,
      absolute_pose_priors,
      T_WB_init_override,
      T_WB_anchor_override);
}

std::vector<LandmarkEstimate> OptimizationModule::getLandmarks(size_t max_points) const
{
  std::shared_ptr<Optimizer> optimizer;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    optimizer = optimizer_;
  }
  if (!optimizer)
  {
    return {};
  }

  return optimizer->getLandmarks(max_points);
}
