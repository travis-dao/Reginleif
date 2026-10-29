
#pragma once

#include <array>
#include <cstddef>

#include "types.h"

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
 * to function as a full model predictive controller (MPC).
 */
class FootholdMpc {
    public:
        /**
        * Configuration parameters for foothold optimization.
        */
        struct Config {
            // Relative importance of improving the robot's stability.
            // Larger values favor stability over staying near the nominal
            // landing position.
            float stability_weight = 1.0f;

            // Relative penalty for moving away from the nominal foothold.
            // Larger values produce smaller landing-position adjustments.
            float movement_weight = 1.0f;

            // Maximum allowed foothold adjustment along the body-frame X axis,
            // in the same distance units used by Vec3.
            float max_step_x = 0.02f;

            // Maximum allowed foothold adjustment along the body-frame Y axis,
            // in the same distance units used by Vec3.
            float max_step_y = 0.02f;

            // Number of candidate offsets sampled along X.
            // Use an odd number to include zero offset.
            int candidate_count_x = 5;

            // Number of candidate offsets sampled along Y.
            // Use an odd number to include zero offset.
            int candidate_count_y = 5;
        };

        /**
        * Robot and gait information needed to plan a foothold.
        *
        * Foot positions and the nominal foothold must all use the same
        * coordinate frame: the robot body frame.
        */
        struct State {
            // Current commanded foot positions for all four legs.
            // These are not measured foot positions.
            Array<Vec3, 4> current_foot_positions_bf;

            // True for legs currently expected to be in stance.
            // Since the robot has no contact sensors, these values normally
            // come from the gait scheduler.
            Array<bool, 4> is_in_stance;

            // Current body roll, pitch, and yaw in degrees.
            // The initial planner uses roll and pitch; yaw is ignored.
            Vec3 body_angles_deg;

            // Current angular velocity around the body axes, in degrees
            // per second. The initial candidate-search implementation may
            // not use this until predictive dynamics are implemented.
            Vec3 body_rates_dps;

            // Index of the leg currently swinging, from 0 to 3.
            int swing_leg = -1;

            // Nominal landing position supplied by the gait planner.
            // The optimizer adjusts this position to improve stability.
            Vec3 nominal_foothold_bf{};
        };

        /**
        * Result returned by the foothold planner.
        */
        struct Result {
            // Selected landing position in the body coordinate frame.
            Vec3 foothold_bf{};

            // Objective-function value for the selected candidate.
            // Lower values are better.
            float cost = 0.0f;

            // False if the planner could not produce a valid candidate.
            bool valid = false;
        };

        /**
        * Construct the planner with its optimization configuration.
        *
        * @param config Optimization weights, search resolution, and limits.
        */
        FootholdMpc(const Config& cfg);

        /**
        * Select a landing position for the current swing leg.
        *
        * The planner evaluates candidate offsets around the nominal
        * foothold and selects the lowest-cost valid candidate.
        *
        * @param state Current robot state and nominal foothold.
        * @return Selected foothold, its cost, and whether it is valid.
        */
        Result planFoothold(const State& state) const;

    private:
        // Stored configuration used by the optimizer.
        Config cfg;

        /**
        * Calculate the objective value for a candidate landing position.
        *
        * Lower scores should correspond to better stability and smaller
        * deviations from the nominal foothold.
        *
        * The implementation must define the stability metric and reject
        * candidates that violate geometric or reachability constraints.
        *
        * @param state Current robot and gait state.
        * @param candidate_foothold_body Candidate landing position.
        * @return Candidate cost; lower is better.
        */
        float scoreCandidate(const State& state, const Vec3& candidate_foothold_body) const;

        /**
        * Check whether a candidate landing position is valid.
        *
        * This should check configured adjustment limits and, once the
        * necessary leg geometry is available, the leg's reachable workspace.
        *
        * @param state Current robot and gait state.
        * @param candidate_foothold_body Candidate landing position.
        * @return True if the candidate satisfies the implemented constraints.
        */
        bool isCandidateValid(const State& state,const Vec3& candidate_foothold_body) const;
};
