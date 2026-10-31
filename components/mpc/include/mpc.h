
#pragma once

#include <array>
#include <cstddef>

#include "types.h"

struct Component {
    const char* name;
    Vec3 position;
    float mass;
};

// mass components for COM calculation
constexpr Array<Component, 5> components = {{
    {"leg0", Vec3 {0, 0, 0}, 1.0f}, // front left
    {"leg1", Vec3 {0, 0, 0}, 1.0f}, // front right
    {"leg2", Vec3 {0, 0, 0}, 1.0f}, // back right
    {"leg3", Vec3 {0, 0, 0}, 1.0f},  // back left
    {"body", Vec3 {0, 0, 0}, 1.0f}, // main body
}};

/**
 * Selects foot landing positions to improve quadruped stability.
 *
 * Responsibilities:
 * - Evaluate candidate landing positions for the swing leg.
 * - Prefer footholds that improve the robot's support geometry.
 * - Avoid unnecessarily large deviations from nominal footholds.
 * - Respect configured limits on foothold adjustments.
 *
 * This class does not control gait timing, perform inverse kinematics,
 * or send commands to servos.
 *
 * The initial implementation can use candidate search. A predictive
 * dynamics model and multi-step optimization must be added for this
 * to function as a full model predictive controller (MPC)
 */
class FootholdMpc {
    public:
        // Configuration parameters for foothold optimization
        struct Config {
            // Relative importance of improving the robot's stability
            // Larger values favor stability over staying near the nominal landing position
            float stability_weight = 1.0f;

            // Relative penalty for moving away from the nominal foothold
            // Larger values produce smaller landing-position adjustments
            float movement_weight = 1.0f;

            // Relative penalty for predicted roll & pitch
            float tilt_weight = 1.0f;

            // margin from COM to edge of support polygon
            float target_stability_margin_mm = 20.0f;

            float min_foot_separation_mm = 40.0f;

            // Maximum allowed foothold adjustment along the body-frame X axis
            float max_step_x_mm = 5.0f;

            // Maximum allowed foothold adjustment along the body-frame Y axis
            float max_step_y_mm = 5.0f;

            // Number of candidate offsets sampled along X
            // Use an odd number to include zero offset
            int candidate_count_x = 5;

            // Number of candidate offsets sampled along Y
            // Use an odd number to include zero offset
            int candidate_count_y = 5;
        };

        // Robot and gait information needed to plan a foothold, body frame
        struct State {
            // Current commanded foot positions for all four legs
            // These are not measured foot positions.
            Array<Vec3, 4> current_foot_positions_bf;

            // Current body roll, pitch, and yaw in degrees
            // The initial planner uses roll and pitch; yaw is ignored
            // Vec3 body_angles_deg;

            // Current angular velocity around the body axes, in degrees
            // per second. The initial candidate-search implementation may
            // not use this until predictive dynamics are implemented
            // Vec3 body_rates_dps;

            // Index of the leg currently swinging, from 0 to 3
            int swing_leg = -1;

            // Nominal landing position supplied by the gait planner
            // The optimizer adjusts this position to improve stability
            Vec3 nominal_foothold_bf;
        };

        // Result returned by the foothold planner
        struct Result {
            // Selected landing position in the body coordinate frame
            Vec3 foothold_bf;

            // Objective-function value for the selected candidate
            // Lower = better
            float cost = 0.0f;

            // False if the planner could not produce a valid candidate
            bool valid = false;
        };
        
        FootholdMpc() = default;
        FootholdMpc(const Config& cfg);

        // Select a landing position for the current swing leg
        Result planFoothold(const State& state) const;

    private:
        Config cfg;

        // Calculate the objective value for a candidate landing position
        // Lower scores should correspond to better stability and smaller deviations from the nominal foothold
        float scoreCandidate(const State& state, const Vec3& candidate_foothold_body_bf) const;

        // Check whether a candidate landing position is valid
        bool isCandidateValid(const State& state,const Vec3& candidate_foothold_body_bf) const;
};
