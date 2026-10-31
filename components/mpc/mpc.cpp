#include "mpc.h"
#include "mpc_helper.h"
// #include "leg.h"

FootholdMpc::FootholdMpc(const Config& cfg) {
    this->cfg = cfg;
}

FootholdMpc::Result FootholdMpc::planFoothold(const State& state) const {
    Result result;
    result.valid = false;
    result.cost = k_float_max;

    if (state.swing_leg < 0 || state.swing_leg >= 4) {
        return result;
    }

    if (this->cfg.candidate_count_x < 1 || this->cfg.candidate_count_y < 1) {
        return result;
    }

    for (int i = 0; i < this->cfg.candidate_count_x; ++i) {
        float dx = 0.0f;

        if (this->cfg.candidate_count_x > 1) {
            float fraction = static_cast<float>(i) / static_cast<float>(this->cfg.candidate_count_x - 1);

            dx = -this->cfg.max_step_x_mm + (2.0f * this->cfg.max_step_x_mm * fraction);
        }

        for (int j = 0; j < this->cfg.candidate_count_y; ++j) {
            float dy = 0.0f;

            if (this->cfg.candidate_count_y > 1) {
                float fraction = static_cast<float>(j) / static_cast<float>(this->cfg.candidate_count_y - 1);

                dy = -this->cfg.max_step_y_mm + (2.0f * this->cfg.max_step_y_mm * fraction);
            }

            Vec3 candidate = state.nominal_foothold_bf;
            candidate.x += dx;
            candidate.y += dy;

            if (!isCandidateValid(state, candidate)) {
                continue;
            }

            float cost = scoreCandidate(state, candidate);

            if (!std::isfinite(cost)) {
                continue;
            }

            if (!result.valid || cost < result.cost) {
                result.cost = cost;
                result.foothold_bf = candidate;
                result.valid = true;
            }
        }
    }

    return result;
}

float FootholdMpc::scoreCandidate(const State& state, const Vec3& candidate_foothold_body_bf) const {
    // J_s
    Vec3 com = calculateCenterOfMass(components.size(), components.data());
    float stability_term = this->cfg.stability_weight * calculateStabilityCost(state, com, candidate_foothold_body_bf, this->cfg.target_stability_margin_mm);

    // J_m
    Vec3 delta = candidate_foothold_body_bf - state.nominal_foothold_bf;
    float movement_term = this->cfg.movement_weight * (delta.x * delta.x + delta.y * delta.y);

    // J_t
    float tilt_term = 0.0f;

    return stability_term + movement_term + tilt_term;
}

bool FootholdMpc::isCandidateValid(const State& state, const Vec3& candidate_foothold_body_bf) const {
    // valid swing leg index
    if (state.swing_leg < 0 || state.swing_leg >= 4) {
        return false;
    }

    // reject non-finite coordinates
    if (!std::isfinite(candidate_foothold_body_bf.x) || !std::isfinite(candidate_foothold_body_bf.y) || !std::isfinite(candidate_foothold_body_bf.z)) {
        return false;
    }

    // reject unreachable legs (ik)
    Theta3 servo_angles;
    // if (!Leg::IK(candidate_foothold_body_bf, servo_angles)) {
    //     return false;
    // }

    // // reject bad angles
    // if (!Leg::outsideServoLimits(servo_angles, state.swing_leg)) {
    //     return false;
    // }

    // Keep the candidate away from the other feet.
    for (int i = 0; i < 4; ++i) {
        if (i == state.swing_leg) {
            continue;
        }

        const Vec3& foot = state.current_foot_positions_bf[i];

        float dx = candidate_foothold_body_bf.x - foot.x;
        float dy = candidate_foothold_body_bf.y - foot.y;

        float distance_squared = dx * dx + dy * dy;

        if (distance_squared < (this->cfg.min_foot_separation_mm * this->cfg.min_foot_separation_mm)) {
            return false;
        }
    }

    return true;
}