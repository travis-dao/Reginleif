#include "mpc.h"
#include "esp_timer.h"

MPC::MPC(const Config& cfg) : 
    cfg_(cfg)
{
    N_ = cfg_.horizon;

    models_.resize(N_);

    A_qp_.setZero(NX * N_, NX);
    B_qp_.setZero(NX * N_, NU * N_);   // upper triangle stays zero forever
    H_.setZero(NU * N_, NU * N_);
    q_.setZero(NU * N_);

    C_.setZero(CONSTRAINTS_PER_FOOT * NUM_LEGS * N_, NU * N_);
    lb_.setZero(CONSTRAINTS_PER_FOOT * NUM_LEGS * N_);
    ub_.setZero(CONSTRAINTS_PER_FOOT * NUM_LEGS * N_);

    const float mu = cfg_.mu;
    Eigen::Matrix<float, CONSTRAINTS_PER_FOOT, 3> C_foot;
    C_foot <<  1,  0, -mu,
              -1,  0, -mu,
               0,  1, -mu,
               0, -1, -mu,
               0,  0,  1;

    // Tile it along the diagonal: one block per (step, foot)
    for (int i = 0; i < N_; i++) {
        for (int j = 0; j < NUM_LEGS; j++) {
            const int row = CONSTRAINTS_PER_FOOT * (NUM_LEGS * i + j);
            const int col = NU * i + 3 * j;   // foot j's (fx, fy, fz) in U
            C_.block(row, col, CONSTRAINTS_PER_FOOT, 3) = C_foot;
            for (int r = 0; r < PYRAMID_ROWS; r++) {
                lb_(row + r) = -INF;
                ub_(row + r) = 0.0f;
            }
        }
    }

    // constant weights, so fill once here too
    L_diag_.resize(NX * N_);
    for (int i = 0; i < N_; i++) {
        L_diag_.segment(NX * i, NX) = cfg_.Q_diag;
    }
    K_diag_ = VecXf::Constant(NU * N_, cfg_.R_weight);

    // init QP solution matricies
    U_sol_.setZero(NU * N_);
    U_prev_.setZero(NU * N_);
}

void MPC::buildDiscreteModels(const MPCProblem& problem) {
    models_.resize(cfg_.horizon);
    for (int i = 0; i < cfg_.horizon; i++) {
        const float yaw = problem.x_ref[i](2);  // yaw is the third element of the state vector

        Model model;
        model.buildA(yaw);
        model.buildB(problem.foot_rel_pos[i], model.worldInertiaInv(cfg_.I_body, yaw), cfg_.mass);

        models_[i] = DiscreteModel::discretize(model, cfg_.dt);
    }
}

void MPC::buildCondensedDynamics() {
    for (int i = 0; i < N_; i++) {
        if (i == 0) {
            A_qp_.block(0, 0, NX, NX) = models_[i].Ad;
        } else {
            A_qp_.block(NX * i, 0, NX, NX) = models_[i].Ad * A_qp_.block(NX * (i - 1), 0, NX, NX);
        }

        B_qp_.block(NX * i, NU * i, NX, NU) = models_[i].Bd;

        for (int j = 0; j < i; j++) {
            B_qp_.block(NX * i, NU * j, NX, NU) = models_[i].Ad * B_qp_.block(NX * (i - 1), NU * j, NX, NU);
        }
    }
}

void MPC::buildCost(const MPCProblem& problem) {
    const StateVec x0 = problem.x0.toVector();

    VecXf Y(NX * N_);

    for (int i = 0; i < N_; i++) {
        Y.segment(NX * i, NX) = problem.x_ref[i];
    }

    VecXf error = A_qp_ * x0 - Y;
    
    // linear term
    q_ = 2 * B_qp_.transpose() * L_diag_.asDiagonal() * error;

    // quadratic term
    H_ = 2 * (B_qp_.transpose() * L_diag_.asDiagonal() * B_qp_);
    H_.diagonal() += 2 * K_diag_;
}

void MPC::buildConstraints(const MPCProblem& problem) {
    for (int i = 0; i < N_; i++) {
        for (int j = 0; j < NUM_LEGS; j++) {
            const int row = CONSTRAINTS_PER_FOOT * (NUM_LEGS * i + j);

            // row + 4 -> force
            if (problem.contact[i][j]) { // grounded legs apply force, need bounds
                lb_[row + PYRAMID_ROWS] = cfg_.f_min;
                ub_[row + PYRAMID_ROWS] = cfg_.f_max;
            } else { // airborned leg apply 0 force, bounds pinned to 0
                lb_[row + PYRAMID_ROWS] = 0.0f;
                ub_[row + PYRAMID_ROWS] = 0.0f;
            }
        }
    }
}

VecXf MPC::shift(const VecXf& U) const {
    VecXf out(NU * N_);
    out.head(NU * (N_ - 1)) = U.tail(NU * (N_ - 1));  // u1..uN-1 move to slots 0..N-2
    out.tail(NU) = U.tail(NU);                      // last slot repeats uN-1
    return out;
}

extern "C" {
    #include "cpg_workspace.h"
    #include "cpg_solve.h"
}
#include <cstring>

bool MPC::solveQP(const MPCProblem& problem, VecXf& U_out) {
    // err = A_qp * x0 - Y (same as in builtCost)
    VecXf Y(NX * N_);
    for (int i = 0; i < N_; i++) {
        Y.segment(NX * i, NX) = problem.x_ref[i];
    }
    const VecXf err = A_qp_ * problem.x0.toVector() - Y;

    // parameters
    for (int k = 0; k < B_qp_.size(); k++) {
        mpc_cpg_update_B_qp(k, B_qp_.data()[k]);
    }
    for (int k = 0; k < err.size(); k++) {
        mpc_cpg_update_err(k, err(k));
    }

    for (int i = 0; i < N_; i++) {
        for (int j = 0; j < NUM_LEGS; j++) {
            const int r = CONSTRAINTS_PER_FOOT * (NUM_LEGS * i + j) + PYRAMID_ROWS;
            const int k = NUM_LEGS * i + j;
            mpc_cpg_update_lb_f(k, lb_(r));
            mpc_cpg_update_ub_f(k, ub_(r));
        }
    }

    mpc_cpg_solve();

    if (strcmp(mpc_CPG_Result.info->status, "solved") != 0) return false;
    for (int k = 0; k < NU * N_; k++) {
        U_out(k) = mpc_CPG_Result.prim->U[k];
    }
    return true;
}

MPCResult MPC::solve(const MPCProblem& problem) {
    const int64_t t_start = esp_timer_get_time();

    // build qp data
    buildDiscreteModels(problem);
    buildCondensedDynamics();
    buildCost(problem);
    buildConstraints(problem);

    // solve
    MPCResult res;
    bool ok = solveQP(problem, U_sol_);
    if (!ok) {
        U_sol_ = shift(U_prev_);
    }
    res.U = U_sol_;
    res.u0 = Input::fromVector(U_sol_.head(NU));
    U_prev_ = U_sol_;

    res.solve_time_ms = (esp_timer_get_time() - t_start) / 1000.f;

    return res;

}