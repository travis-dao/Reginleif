#ifndef POLICY_h
#define POLICY_h

// Header-only MLP policy:
//   normalize (+clip) -> 2 hidden layers -> tanh output.
//
// Architecture: POLICY_OBS_DIM -> POLICY_H1 -> POLICY_H2 -> POLICY_ACT_DIM
// Expected architecture: 135 -> 256 -> 256 -> 12.
//
// Observation = OBS_HISTORY (3) frames, oldest first, each OBS_FRAME_DIM (45):
//   0..2    gyro (body frame) * GYRO_SCALE
//   3..5    gravity direction in body frame (unit vector)
//   6..8    command vx, vy (* CMD_SCALE_LIN), yaw rate (* CMD_SCALE_ANG)
//   9..20   joint positions minus default (rad)
//   21..32  joint velocities * QD_SCALE
//   33..44  previous action (clamped, un-ramped)
//
// Weights are row-major [out][in], matching PyTorch nn.Linear.weight layout.
// policy_weights.h provides the architecture, normalization, and weights.
// NOTE: include this header from ONE translation unit only (the weight tables
// are constexpr arrays and would be duplicated per TU).

#include <cmath>

#include "types.h"
#include "motion_config.h"
#include "policy_weights.h"

using PolicyObservation = Mat<float, POLICY_OBS_DIM, 1>;
using PolicyAction = Mat<float, POLICY_ACT_DIM, 1>;

static_assert(POLICY_OBS_DIM == 135, "Policy expects 135 observation values (3 x 45)");
static_assert(POLICY_ACT_DIM == 12, "Policy expects 12 joint-target outputs");

static_assert(sizeof(POLICY_OBS_MEAN) / sizeof(float) == POLICY_OBS_DIM, "obs mean size != POLICY_OBS_DIM");
static_assert(sizeof(POLICY_OBS_STD)  / sizeof(float) == POLICY_OBS_DIM, "obs std size != POLICY_OBS_DIM");
static_assert(sizeof(POLICY_W1)       / sizeof(float) == POLICY_H1 * POLICY_OBS_DIM, "W1 shape mismatch");
static_assert(sizeof(POLICY_W2)       / sizeof(float) == POLICY_H2 * POLICY_H1, "W2 shape mismatch");
static_assert(sizeof(POLICY_W3)       / sizeof(float) == POLICY_ACT_DIM * POLICY_H2, "W3 shape mismatch");

class PolicyNet {
    public:
        PolicyNet() = default;

        // Deterministic forward pass.
        // Returns 12 normalized joint offsets in [-1, 1]. The call site converts
        // them to radians: q_target = q_default + ACTION_SCALE_RAD * a.
        PolicyAction infer(const PolicyObservation& obs) const {
            // 1. Normalize and clip observations.
            Mat<float, POLICY_OBS_DIM, 1> x;
            for (int i = 0; i < POLICY_OBS_DIM; ++i) {
                float v = (obs[i] - POLICY_OBS_MEAN[i]) / (POLICY_OBS_STD[i] + NORM_EPS);
                if (POLICY_OBS_CLIP > 0.0f) {
                    v = fmaxf(-POLICY_OBS_CLIP, fminf(POLICY_OBS_CLIP, v));
                }
                x[i] = v;
            }

            // 2. Two hidden layers.
            Mat<float, POLICY_H1, 1> h1 = layer<POLICY_H1, POLICY_OBS_DIM>(POLICY_W1, POLICY_B1, x);
            activate_hidden(h1);

            Mat<float, POLICY_H2, 1> h2 = layer<POLICY_H2, POLICY_H1>(POLICY_W2, POLICY_B2, h1);
            activate_hidden(h2);

            // 3. Output layer, followed by tanh to bound normalized actions.
            PolicyAction out = layer<POLICY_ACT_DIM, POLICY_H2>(POLICY_W3, POLICY_B3, h2);
            for (int i = 0; i < POLICY_ACT_DIM; ++i) {
                out[i] = tanhf(out[i]);
            }

            return out;
        }

    private:
        static constexpr float NORM_EPS = 1e-8f;

        template <int OUT, int IN>
        static Mat<float, OUT, 1> layer(
            const float* W,
            const float* b,
            const Mat<float, IN, 1>& in) {
            Eigen::Map<const Eigen::Matrix<float, OUT, IN, Eigen::RowMajor>> Wm(W);
            Eigen::Map<const Eigen::Matrix<float, OUT, 1>> bm(b);
            Mat<float, OUT, 1> out = Wm * in + bm;
            return out;
        }

        template <int N>
        static void activate_hidden(Mat<float, N, 1>& v) {
            for (int i = 0; i < N; ++i) {
                const float z = v[i];
                switch (POLICY_ACTIVATION) {
                    case 0:  v[i] = z > 0.0f ? z : 0.0f; break;       // ReLU
                    case 1:  v[i] = z > 0.0f ? z : expm1f(z); break;  // ELU, alpha=1
                    default: v[i] = tanhf(z); break;                  // tanh
                }
            }
        }
    };

#endif