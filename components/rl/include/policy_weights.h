#ifndef POLICY_WEIGHTS_h
#define POLICY_WEIGHTS_h

// DUMMY WEIGHTS: placeholder until the RL policy is trained.
// All weights and biases are zero, so the policy outputs exactly zero for every
// observation -> q_target = q_default. The robot stands and holds, which is a
// good bring-up test for the whole pipeline (IMU, history, 50 Hz tick, slew
// limiter, servo mapping) with no learned behavior.
//
// Observation layout (135 = 3 frames x 45, oldest first), per frame:
//   0..2    gyro * GYRO_SCALE
//   3..5    gravity direction in body frame
//   6..8    command vx, vy, yaw rate (scaled)
//   9..20   joint positions - default (rad)
//   21..32  joint velocities * QD_SCALE
//   33..44  previous action
//
// Output layout (12): normalized joint offsets, leg-major FL, FR, BR, BL x (coxa, femur, tibia).
// Weights are row-major [out][in], matching PyTorch nn.Linear.weight layout.

#include "motion_config.h"

// ---- architecture: 135 -> 256 -> 256 -> 12 ----
constexpr int POLICY_H1 = 256;
constexpr int POLICY_H2 = 256;

constexpr int POLICY_ACTIVATION = 1;     // Hidden layers: 0 = ReLU, 1 = ELU, 2 = tanh (match training)
constexpr float POLICY_OBS_CLIP = 0.0f;  // <= 0 disables clipping (set e.g. 5.0 if training used it)

static_assert(POLICY_OBS_DIM == 135, "Set POLICY_OBS_DIM to 135 in motion_config.h");
static_assert(POLICY_ACT_DIM == 12, "Set POLICY_ACT_DIM to 12 in motion_config.h");

// ---- observation normalization (identity placeholders) ----
// If training uses a running normalizer, export per-frame mean/std tiled OBS_HISTORY times.
// If the env uses the fixed scales from motion_config.h, keep mean 0 / std 1.
template <int N>
struct ConstFloatArray {
    float v[N];

    constexpr ConstFloatArray(float value) : v() {
        for (int i = 0; i < N; ++i) {
            v[i] = value;
        }
    }

    constexpr float operator[](int i) const {
        return v[i];
    }
};

constexpr ConstFloatArray<POLICY_OBS_DIM> POLICY_OBS_MEAN(0.0f);
constexpr ConstFloatArray<POLICY_OBS_DIM> POLICY_OBS_STD(1.0f);

// ---- dummy weights and biases ----
// Zero initialization is intentional. Do not use this as a trained controller.
constexpr float POLICY_W1[POLICY_H1 * POLICY_OBS_DIM] = {};
constexpr float POLICY_B1[POLICY_H1] = {};

constexpr float POLICY_W2[POLICY_H2 * POLICY_H1] = {};
constexpr float POLICY_B2[POLICY_H2] = {};

constexpr float POLICY_W3[POLICY_ACT_DIM * POLICY_H2] = {};
constexpr float POLICY_B3[POLICY_ACT_DIM] = {};

#endif