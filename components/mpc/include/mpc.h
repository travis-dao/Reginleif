#pragma once

#include "srb_model.h"
#include "eigen.h"

// Per-foot constraint rows: 4 friction-pyramid rows + 1 normal-force row
constexpr int CONSTRAINTS_PER_FOOT = 5;
constexpr int PYRAMID_ROWS = 4;
constexpr float INF = 1e20f;

// Per-solve inputs. All vectors have length `horizon`
struct MPCProblem {
    State x0;  // measured current state

    // Desired state at steps 1..N. Gravity slot must equal State::g.
    Vector<StateVec> x_ref;

    // r_i = foot position minus CoM, world frame, per step and per foot.
    Vector<Array<Vec3f, NUM_LEGS>> foot_rel_pos;

    // true = foot i is in stance at step k, false = swing (force forced to 0).
    Vector<Array<bool, NUM_LEGS>> contact;
};

// Solver output
struct MPCResult {
    bool success = false;
    Input u0;                  // forces to apply now (first step of U)
    VecXf U;                    // full stacked solution, size 12 * N
    float solve_time_ms = 0.f;
};

class MPC {
    public:
        struct Config {
            int horizon = 10;        // N: number of prediction steps
            float dt = 0.03f;        // prediction step [s]

            float mass = 12.0f;      // robot mass [kg]
            Mat3f I_body = Mat3f::Identity();  // body-frame inertia about the CoM

            float mu = 0.4f;         // friction coefficient
            float f_min = 0.0f;      // min normal force per stance foot [N]
            float f_max = 150.0f;    // max normal force per stance foot [N]

            // State tracking weights (diagonal of Q), ordered
            // [roll pitch yaw | x y z | wx wy wz | vx vy vz | g]
            Eigen::Matrix<float, NX, 1> Q_diag = Eigen::Matrix<float, NX, 1>::Ones();
            // Q_diag=np.array([5, 5, 10, 1, 1, 50, 1, 1, 1, 1, 1, 1, 0.0]),

            float R_weight = 1e-3f;  // force regularization (diagonal of R)
        };

        MPC(const Config& cfg);

        // Build the QP from `problem`, solve it, and return the first-step forces.
        MPCResult solve(const MPCProblem& problem);

        Config config() const { return cfg_; }
        int N() const { return N_; }

    private:
        // 1. Linearized, discretized model at every step k = 0..N-1.
        //    Uses x_ref yaw and foot_rel_pos[k] (both known ahead of time).
        void buildDiscreteModels(const MPCProblem& problem);

        // 2. Condense the dynamics:  X = A_qp * x0 + B_qp * U
        //    A_qp: 13N x 13,  B_qp: 13N x 12N (block lower triangular)
        void buildCondensedDynamics();

        // 3. Cost:  min 0.5 U^T H U + U^T g
        //    H = 2 (B_qp^T L B_qp + K),  g = 2 B_qp^T L (A_qp x0 - Y)
        void buildCost(const MPCProblem& problem);

        // 4. Constraints:  lb <= C U <= ub
        //    Friction pyramid + force limits; swing feet get fz in [0, 0],
        //    which also forces fx = fy = 0 through the pyramid rows.
        void buildConstraints(const MPCProblem& problem);

        // 5. Hand H, q, C, lb, ub to the QP backend (OSQP).
        bool solveQP(const MPCProblem& problem, VecXf& U_out);

        // Shift a stacked solution forward one step: [u0 u1 ... uN-1] -> [u1 ... uN-1 uN-1]
        VecXf shift(const VecXf& U) const;

        // Discrete model per step
        Vector<DiscreteModel> models_;

        Config cfg_;
        int N_;

        // Condensed dynamics
        MatXf A_qp_;  // 13N x 13
        MatXf B_qp_;  // 13N x 12N

        // Cost weights (stacked over the horizon)
        VecXf L_diag_;  // 13N, repeated Q_diag
        VecXf K_diag_;  // 12N, repeated R_weight

        // QP data
        MatXf H_;       // 12N x 12N
        VecXf q_;       // 12N
        MatXf C_;       // (5 * 4 * N) x 12N
        VecXf lb_;      // 5 * 4 * N
        VecXf ub_;      // 5 * 4 * N

        VecXf U_sol_;    // 12N, current solution (written by solveQP)
        VecXf U_prev_;   // 12N, last successful solution (warm start / fallback)

        bool solver_initialized_ = false;   // for the one-time OSQP setup
};
