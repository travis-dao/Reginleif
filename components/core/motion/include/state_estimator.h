#ifndef STATE_ESTIMATOR_h
#define STATE_ESTIMATOR_h

#include "rtos_utils.h"
#include "leg_controller.h"
#include "mpc.h"
#include "motion_config.h"

/// Cheetah-style linear Kalman filter for body position + velocity, kinematics driven.
///
///   x = [ p_body(3), v_body(3), p_foot_FL(3), p_foot_FR(3), p_foot_BR(3), p_foot_BL(3) ]  (world, m)
///
///   predict : p += v dt, v constant (no accelerometer available), feet stationary
///   measure : foot position relative to body (leg FK), body velocity from stance-foot
///             kinematics (-R (w x r + dr)), and foot height = 0 on flat ground
///   swing feet get inflated process/measurement noise, so they don't corrupt the estimate.
///
/// Orientation comes from ImuData::rpy_rad, angular rate from ImuData::gyro_rad_s.
/// Contact state comes from the gait: every leg except LegController::swingLeg().
///
/// Assumes the body frame origin is the CoM (as Core does when it uses s.pos + R * foot_body).
class StateEstimator {
    public:
        static constexpr int NL = 4;
        static constexpr int NS = 6 + 3 * NL;   // 18
        static constexpr int NY = 7 * NL;       // 28: 12 pos + 12 vel + 4 height

        struct Config {
            LegController& leg_controller;

            /// Only used to fill State::g (the MPC's gravity slot).
            float gravity_z = GRAVITY;

            // Process noise (scaled by dt each tick).
            // There is no accelerometer, so velocity is a random walk: process_vel has to cover
            // the real body accelerations. Raise it if the estimate lags, lower it if it is noisy.
            float process_pos  = 1.0f / 20.0f;
            float process_vel  = 9.8f / 20.0f;
            float process_foot = 1.0f;

            // Measurement noise
            float meas_foot_pos    = 0.001f;
            float meas_foot_vel    = 0.1f;
            float meas_foot_height = 0.001f;

            /// Noise multiplier (1 + (1-trust)*k) for feet that are not in stance.
            float high_suspect = 100.0f;

            /// After touchdown, trust ramps 0 -> 1 over this time [s].
            float touchdown_trust_s = 0.05f;

            /// Low-pass weight of the newest finite-difference foot velocity (1 = no filtering).
            float foot_vel_alpha = 0.5f;

            // Initial covariance diagonal after reset()
            float init_var_pos  = 1e-2f;
            float init_var_vel  = 1e-2f;
            float init_var_foot = 1e-2f;
        };

        StateEstimator(Config cfg);

        /// init at known pose (ex. neutral)
        void reset(const Vec3f& p0, const Vec3f& v0, const ImuData& imu);

        /// One filter tick. Call every control tick (not only on MPC solve ticks).
        /// If imu.valid is false the previous estimate is returned unchanged.
        /// If reset() was never called, starts at the world origin.
        const State& update(float dt_s, const ImuData& imu);

        const State& state() const { return state_; }
        bool initialized() const { return initialized_; }
        const Array<float, NL>& trust() const { return trust_; }

    private:
        using VecS  = Mat<float, NS, 1>;
        using MatSS = Mat<float, NS, NS>;

        void updateContact(float dt);
        Array<Vec3f, NL> footRelBody() const;   // metres
        void publish(const Vec3f& rpy, const Mat3f& R, const Vec3f& gyro);

        LegController& leg_controller_;
        Config cfg_;

        VecS x_  = VecS::Zero();
        MatSS P_ = MatSS::Identity();
        State state_;
        bool initialized_ = false;

        Array<float, NL> t_stance_;
        Array<float, NL> trust_;
        Array<Vec3f, NL> r_prev_;   // previous foot-in-body positions [m]
        Array<Vec3f, NL> dr_;       // filtered foot velocity in body frame [m/s]
        bool have_prev_ = false;
};

#endif