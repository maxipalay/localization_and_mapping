#include "visual_inertial_frontend/preintegrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

// #if !defined(GTSAM_ENABLE_BOOST_SERIALIZATION) || !(GTSAM_ENABLE_BOOST_SERIALIZATION)
// #error "ImuPreintegrator requires GTSAM_ENABLE_BOOST_SERIALIZATION enabled in GTSAM."
// #endif

#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <gtsam/navigation/CombinedImuFactor.h> // PreintegrationCombinedParams, PreintegratedCombinedMeasurements
#include <gtsam/navigation/ImuBias.h>           // imuBias::ConstantBias
#include <gtsam/base/Matrix.h>                  // I_3x3, I_6x6
#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <iomanip>

namespace
{

    inline double pos_inf() { return std::numeric_limits<double>::infinity(); }
    inline double neg_inf() { return -std::numeric_limits<double>::infinity(); }

    inline gtsam::Vector3 toGtsam(const Eigen::Vector3d &v)
    {
        return gtsam::Vector3(v.x(), v.y(), v.z());
    }

    inline std::shared_ptr<gtsam::PreintegrationCombinedParams>
    makeParams(const ImuPreintegratorConfig &cfg)
    {
        // "U" frame: z-up, gravity points along -Z.
        auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(cfg.gravity_mps2);

        const double acc2 = cfg.accel_noise_density * cfg.accel_noise_density;
        const double gyr2 = cfg.gyro_noise_density * cfg.gyro_noise_density;
        const double accb2 = cfg.accel_bias_rw * cfg.accel_bias_rw;
        const double gyrb2 = cfg.gyro_bias_rw * cfg.gyro_bias_rw;

        // Tiny integration covariance (MVP)
        const double int2 = 1e-8 * 1e-8;

        p->setAccelerometerCovariance(acc2 * gtsam::I_3x3);
        p->setGyroscopeCovariance(gyr2 * gtsam::I_3x3);
        p->setIntegrationCovariance(int2 * gtsam::I_3x3);

        p->setBiasAccCovariance(accb2 * gtsam::I_3x3);
        p->setBiasOmegaCovariance(gyrb2 * gtsam::I_3x3);

        // Simplest assumption: measurements are already in the body frame.
        p->setBodyPSensor(gtsam::Pose3::Identity());

        return p;
    }

    // Adjust these accessors if your ImuSample fields differ.
    inline double &sample_time(ImuSample &s) { return s.t_s; }
    inline const double &sample_time(const ImuSample &s) { return s.t_s; }

    inline Eigen::Vector3d &sample_accel(ImuSample &s) { return s.accel; }
    inline const Eigen::Vector3d &sample_accel(const ImuSample &s) { return s.accel; }

    inline Eigen::Vector3d &sample_gyro(ImuSample &s) { return s.gyro; }
    inline const Eigen::Vector3d &sample_gyro(const ImuSample &s) { return s.gyro; }

} // namespace

// ---------------- ImuPreintegrator ----------------

ImuPreintegrator::ImuPreintegrator(const ImuPreintegratorConfig &cfg) : cfg_(cfg) {}

void ImuPreintegrator::reset()
{
    std::lock_guard<std::mutex> lk(mtx_);
    buf_.clear();
    bias_hat_ = ImuBias{};
}

void ImuPreintegrator::setConfig(const ImuPreintegratorConfig &cfg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cfg_ = cfg;
    pruneLocked_();
}

void ImuPreintegrator::push(double t_s,
                            const Eigen::Vector3d &accel_mps2,
                            const Eigen::Vector3d &gyro_rps)
{
    std::lock_guard<std::mutex> lk(mtx_);

    if (!std::isfinite(t_s))
        return;

    // First-pass assumption: timestamps are monotonic; drop out-of-order
    if (!buf_.empty() && t_s <= sample_time(buf_.back()))
        return;

    ImuSample s;
    sample_time(s) = t_s;
    sample_accel(s) = accel_mps2;
    sample_gyro(s) = gyro_rps;

    buf_.push_back(s);
    pruneLocked_();
}

size_t ImuPreintegrator::size() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return buf_.size();
}

double ImuPreintegrator::oldestTime() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (buf_.empty())
        return pos_inf();
    return sample_time(buf_.front());
}

double ImuPreintegrator::newestTime() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (buf_.empty())
        return neg_inf();
    return sample_time(buf_.back());
}

bool ImuPreintegrator::hasCoverage(double t_s) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (buf_.empty())
        return false;
    return sample_time(buf_.back()) >= t_s;
}

void ImuPreintegrator::setBias(const ImuBias &bias)
{
    std::lock_guard<std::mutex> lk(mtx_);
    bias_hat_ = bias;

    // Debug print
    // std::cout << std::fixed << std::setprecision(6)
    //           << "[preint] new bias accel=("
    //           << bias_hat_.accel.x() << ", "
    //           << bias_hat_.accel.y() << ", "
    //           << bias_hat_.accel.z() << ") gyro=("
    //           << bias_hat_.gyro.x() << ", "
    //           << bias_hat_.gyro.y() << ", "
    //           << bias_hat_.gyro.z() << ")\n"
    //           << std::endl;
}

ImuBias ImuPreintegrator::bias() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return bias_hat_;
}

void ImuPreintegrator::pruneLocked_()
{
    if (cfg_.max_buffer_s <= 0.0)
        return;
    if (buf_.empty())
        return;

    const double newest = sample_time(buf_.back());
    const double min_t = newest - cfg_.max_buffer_s;

    while (buf_.size() > 1 && sample_time(buf_.front()) < min_t)
    {
        buf_.pop_front();
    }
}

std::optional<ImuPreintegrator::Packet> ImuPreintegrator::buildAndConsume(
    uint64_t kf_id0, double t0_s,
    uint64_t kf_id1, double t1_s)
{
    if (!std::isfinite(t0_s) || !std::isfinite(t1_s) || !(t1_s > t0_s))
    {
        return std::nullopt;
    }

    // Copy out integration knots under lock, and consume buffer up to t1
    std::vector<ImuSample> knots;
    ImuBias bias_hat_local;

    {
        std::lock_guard<std::mutex> lk(mtx_);

        if (buf_.empty())
            return std::nullopt;
        if (sample_time(buf_.back()) < t1_s)
            return std::nullopt; // no coverage yet

        bias_hat_local = bias_hat_;

        // first sample with t > t0
        auto it_first = std::upper_bound(
            buf_.begin(), buf_.end(), t0_s,
            [](double t, const ImuSample &s)
            { return t < sample_time(s); });

        // first sample with t > t1
        auto it_end = std::upper_bound(
            buf_.begin(), buf_.end(), t1_s,
            [](double t, const ImuSample &s)
            { return t < sample_time(s); });

        if (it_first == buf_.end())
            return std::nullopt;
        if (sample_time(*it_first) > t1_s)
            return std::nullopt; // no samples in (t0,t1]

        // Measurement to use at t0: last sample <= t0 if available else first > t0
        ImuSample meas0 = *it_first;
        if (it_first != buf_.begin())
        {
            const auto it_prev = std::prev(it_first);
            if (sample_time(*it_prev) <= t0_s)
                meas0 = *it_prev;
        }

        // Build knot list:
        //  - synthetic sample at t0 using meas0
        //  - all real samples in (t0, t1]
        //  - synthetic endpoint at t1 if needed
        knots.reserve(static_cast<size_t>(std::distance(it_first, it_end)) + 2);

        ImuSample k0 = meas0;
        sample_time(k0) = t0_s;
        knots.push_back(k0);

        for (auto it = it_first; it != it_end; ++it)
        {
            if (sample_time(*it) <= t0_s)
                continue;
            if (sample_time(*it) > t1_s)
                break;
            knots.push_back(*it);
        }

        if (knots.size() < 2)
            return std::nullopt;

        if (sample_time(knots.back()) < t1_s)
        {
            ImuSample k1 = knots.back(); // reuse last measurement
            sample_time(k1) = t1_s;
            knots.push_back(k1);
        }
        else
        {
            sample_time(knots.back()) = t1_s; // force exact
        }

        // Consume samples <= t1, optionally keep anchor (last <= t1)
        if (it_end != buf_.begin())
        {
            auto it_last_le = std::prev(it_end);
            if (cfg_.keep_anchor)
            {
                buf_.erase(buf_.begin(), it_last_le); // keep *it_last_le
            }
            else
            {
                buf_.erase(buf_.begin(), std::next(it_last_le)); // drop it too
            }
        }

        pruneLocked_();
    }

    // Preintegrate outside lock
    auto params = makeParams(cfg_);
    const gtsam::imuBias::ConstantBias biasHat(toGtsam(bias_hat_local.accel),
                                               toGtsam(bias_hat_local.gyro));
    gtsam::PreintegratedCombinedMeasurements pim(params, biasHat);

    for (size_t i = 0; i + 1 < knots.size(); ++i)
    {
        const double ta = sample_time(knots[i]);
        const double tb = sample_time(knots[i + 1]);
        const double dt = tb - ta;
        if (dt <= 0.0)
            continue;

        pim.integrateMeasurement(
            toGtsam(sample_accel(knots[i])),
            toGtsam(sample_gyro(knots[i])),
            dt);
    }

    // Serialize to bytes
    std::vector<uint8_t> bytes;
    {
        std::ostringstream oss(std::ios::binary);
        boost::archive::binary_oarchive oa(oss);
        oa.register_type<gtsam::PreintegrationCombinedParams>();
        oa << pim;
        const std::string s = oss.str();
        bytes.assign(s.begin(), s.end());
    }

    Packet pkt{};
    pkt.kf_id0 = kf_id0;
    pkt.kf_id1 = kf_id1;
    pkt.t0_s = t0_s;
    pkt.t1_s = t1_s;
    pkt.bias_hat = bias_hat_local;
    pkt.bytes = std::move(bytes);
    pkt.valid = !pkt.bytes.empty();

    return pkt;
}
