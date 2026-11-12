#include "state_estimator.h"
#include "frame.h"
#include "leg_controller.h"

// rpy = (roll, pitch, yaw) [rad], ZYX convention -> body-to-world rotation.
Mat3f rotFromRpy(const Vec3f& rpy) {
    using AA = Eigen::AngleAxisf;
    return (AA(rpy.z(), Vec3f::UnitZ()) *
            AA(rpy.y(), Vec3f::UnitY()) *
            AA(rpy.x(), Vec3f::UnitX())).toRotationMatrix();
}

// ----------------------------------------------------------------------
StateEstimator::StateEstimator(StateEstimator::Config cfg, LegController& leg_controller)
    : leg_controller_(leg_controller), 
      cfg_(cfg) 
{
    t_stance_.fill(1e9f);
    trust_.fill(1.0f);
    for (auto& v : r_prev_) {
        v.setZero();
    }
    for (auto& v : dr_) {
        v.setZero();
    }
    state_.g = cfg_.gravity_z;
}

Array<Vec3f, StateEstimator::NL> StateEstimator::footRelBody() const {
    const auto feet = leg_controller_.footPositionsBody();   // mm, body frame
    Array<Vec3f, NL> r;
    for (int i = 0; i < NL; i++) {
        r[i] = mmToM(feet[i]);
    }
    return r;
}

void StateEstimator::reset(const Vec3f& p0, const Vec3f& v0, const ImuData& imu) {
    if (!imu.valid) return;

    const Vec3f rpy  = toEigen(imu.rpy_rad);
    const Vec3f gyro = toEigen(imu.gyro_rad_s);
    const Mat3f R    = rotFromRpy(rpy);
    const auto r     = footRelBody();

    x_.setZero();
    x_.segment<3>(0) = p0;
    x_.segment<3>(3) = v0;
    for (int i = 0; i < NL; i++) {
        x_.segment<3>(6 + 3 * i) = p0 + R * r[i];
    }

    P_.setZero();
    P_.block<3, 3>(0, 0).diagonal().setConstant(cfg_.init_var_pos);
    P_.block<3, 3>(3, 3).diagonal().setConstant(cfg_.init_var_vel);
    for (int i = 0; i < NL; i++)
        P_.block<3, 3>(6 + 3 * i, 6 + 3 * i).diagonal().setConstant(cfg_.init_var_foot);

    t_stance_.fill(1e9f);
    trust_.fill(1.0f);
    r_prev_ = r;
    for (auto& v : dr_) {
        v.setZero();
    }
    have_prev_   = true;
    initialized_ = true;
    publish(rpy, R, gyro);
}

// ----------------------------------------------------------------------
void StateEstimator::updateContact(float dt) {
    // Crawl gait: exactly one swing leg at a time (-1 = all feet planted).
    const int swing = leg_controller_.swingLeg();
    for (int i = 0; i < NL; i++) {
        if (i == swing) {
            t_stance_[i] = 0.0f;
            trust_[i]    = 0.0f;
        } else {
            t_stance_[i] += dt;
            trust_[i] = (cfg_.touchdown_trust_s > 0.0f) ? fmin(1.0f, t_stance_[i] / cfg_.touchdown_trust_s) : 1.0f;
        }
    }
}

// ----------------------------------------------------------------------
const State& StateEstimator::update(float dt, const ImuData& imu) {
    if (!imu.valid || dt <= 0.0f) return state_;   // no IMU -> hold last estimate
    if (!initialized_) reset(Vec3f::Zero(), Vec3f::Zero(), imu);

    const Vec3f rpy  = toEigen(imu.rpy_rad);
    const Vec3f gyro = toEigen(imu.gyro_rad_s);   // body frame
    const Mat3f R    = rotFromRpy(rpy);

    updateContact(dt);

    // Foot kinematics in the body frame, and a filtered finite-difference foot velocity
    const auto r = footRelBody();
    for (int i = 0; i < NL; i++) {
        const Vec3f dr_raw = have_prev_ ? Vec3f((r[i] - r_prev_[i]) / dt) : Vec3f::Zero();
        dr_[i] = cfg_.foot_vel_alpha * dr_raw + (1.0f - cfg_.foot_vel_alpha) * dr_[i];
        r_prev_[i] = r[i];
    }
    have_prev_ = true;

    // ---------------- predict (constant velocity) ----------------
    MatSS A = MatSS::Identity();
    A.block<3, 3>(0, 3) = dt * Mat3f::Identity();
    x_ = A * x_;

    MatSS Q = MatSS::Zero();
    Q.block<3, 3>(0, 0).diagonal().setConstant(dt * cfg_.process_pos);
    Q.block<3, 3>(3, 3).diagonal().setConstant(dt * cfg_.process_vel);
    for (int i = 0; i < NL; i++) {
        const float s = 1.0f + (1.0f - trust_[i]) * cfg_.high_suspect;
        Q.block<3, 3>(6 + 3 * i, 6 + 3 * i).diagonal().setConstant(dt * cfg_.process_foot * s);
    }
    P_ = A * P_ * A.transpose() + Q;

    // ---------------- measurement ----------------
    Mat<float, NY, 1> y  = Mat<float, NY, 1>::Zero();
    Mat<float, NY, NS> C = Mat<float, NY, NS>::Zero();
    Mat<float, NY, 1> Rd;

    const Vec3f v_pred = x_.segment<3>(3);
    const float pz_pred = x_(2);

    for (int i = 0; i < NL; i++) {
        const int ip = 3 * i, iv = 12 + 3 * i, ih = 24 + i, ifoot = 6 + 3 * i;
        const float trust = trust_[i];
        const float scale = 1.0f + (1.0f - trust) * cfg_.high_suspect;

        const Vec3f p_f = R * r[i];   // body -> foot, world axes

        // (1) foot position relative to body:  p_body - p_foot = -R r
        y.segment<3>(ip) = -p_f;
        C.block<3, 3>(ip, 0)     = Mat3f::Identity();
        C.block<3, 3>(ip, ifoot) = -Mat3f::Identity();

        // (2) body velocity from a planted foot:  v_body = -R (w x r + dr)
        const Vec3f dp_f = R * (gyro.cross(r[i]) + dr_[i]);
        y.segment<3>(iv) = (1.0f - trust) * v_pred + trust * (-dp_f);
        C.block<3, 3>(iv, 3) = Mat3f::Identity();

        // (3) planted foot height = 0 (flat ground)
        y(ih) = (1.0f - trust) * (pz_pred + p_f.z());
        C(ih, ifoot + 2) = 1.0f;

        Rd.segment<3>(ip).setConstant(cfg_.meas_foot_pos * scale);
        Rd.segment<3>(iv).setConstant(cfg_.meas_foot_vel * scale);
        Rd(ih) = cfg_.meas_foot_height * scale;
    }

    // ---------------- update ----------------
    const Mat<float, NY, 1> innov = y - C * x_;
    const Mat<float, NY, NY> S = C * P_ * C.transpose() + Mat<float, NY, NY>(Rd.asDiagonal());
    const Mat<float, NY, NS> SiCP = S.ldlt().solve(C * P_);
    const Mat<float, NS, NY> K    = SiCP.transpose();   // P is symmetric

    x_ += K * innov;
    P_ = (MatSS::Identity() - K * C) * P_;
    P_ = 0.5 * (P_ + P_.transpose());

    publish(rpy, R, gyro);
    return state_;
}

// ----------------------------------------------------------------------
void StateEstimator::publish(const Vec3f& rpy, const Mat3f& R, const Vec3f& gyro) {
    state_.pos         = x_.segment<3>(0);
    state_.linear_vel  = x_.segment<3>(3);
    state_.roll        = rpy.x();
    state_.pitch       = rpy.y();
    state_.yaw         = rpy.z();
    state_.angular_vel = R * gyro;   // world frame, as the MPC expects
    state_.g           = cfg_.gravity_z;
}