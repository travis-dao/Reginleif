#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "types.h"
#include "imu.h"

class PostureShaper {
    public:
        explicit PostureShaper(
            float max_roll = 8.0f * RAD_TO_DEG,
            float max_pitch = 6.0f * RAD_TO_DEG,
            float w_ref = 0.5f,
            float v_ref = 0.09f,
            float a_ref = 0.15f,
            float tau = 0.15f,
            float turn_in_place_lean = 0.5f
        )
            : max_roll_(max_roll),
            max_pitch_(max_pitch),
            w_ref_(w_ref),
            v_ref_(v_ref),
            a_ref_(a_ref),
            tau_(tau),
            tip_(turn_in_place_lean) {}

        Imu::EulerAngles update(float dt, float v_mps, float w_rad_s, float a_mps2) {
            // Clamp values to [-1, 1].
            const float speed = clamp(std::abs(v_mps) / v_ref_);

            // Increase lean with forward speed.
            const float gain = tip_ + (1.0f - tip_) * speed;

            // Lean into the turn.
            const float roll_t =
                -max_roll_ * clamp(w_rad_s / w_ref_) * gain;

            // Nose-down during acceleration, nose-up during braking.
            const float pitch_t =
                max_pitch_ * clamp(a_mps2 / a_ref_);

            // Exponential smoothing.
            const float alpha = 1.0f - std::exp(-dt / tau_);

            roll_ += (roll_t - roll_) * alpha;
            pitch_ += (pitch_t - pitch_) * alpha;

            return Imu::EulerAngles{roll_, pitch_, 0.0f};
        }

        float getRoll() const { return roll_; }
        float getPitch() const { return pitch_; }

    private:
        float roll_ = 0.0f;
        float pitch_ = 0.0f;

        float max_roll_;
        float max_pitch_;

        float w_ref_;
        float v_ref_;
        float a_ref_;
        float tau_;
        float tip_;

        static float clamp(float x) {
            return std::max(-1.0f, std::min(1.0f, x));
        }
};
