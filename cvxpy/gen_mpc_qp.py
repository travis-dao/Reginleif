
"""
gen_mpc_qp.py -- generate a float32 embedded OSQP solver for condensed MPC.

Install:
    pip install osqp numpy scipy

Generate:
    python cvxpy/gen_mpc_qp.py --target components/qp

Sanity-check:
    python cvxpy/gen_mpc_qp.py --test

Generated solver:
    components/qp/c/

The generated OSQP solver uses float32 (OSQPFloat = float).
P and q can be updated every MPC tick. A has fixed sparsity and values.
"""

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import osqp
from scipy import sparse


NX = 13
NU = 12
NUM_LEGS = 4
PYRAMID_ROWS = 4


@dataclass
class Config:
    horizon: int = 10
    mu: float = 0.6
    R_weight: float = 1e-5

    # State order: roll, pitch, yaw, x, y, z,
    #              omega_x, omega_y, omega_z,
    #              vel_x, vel_y, vel_z, gravity
    Q_diag: np.ndarray = field(default_factory=lambda: np.array(
        [
            5, 5, 10,
            1, 1, 50,
            1, 1, 1,
            1, 1, 1,
            0.0,
        ],
        dtype=np.float64,
    ))


def friction_pyramid(cfg: Config) -> sparse.csc_matrix:
    """C_pyr @ U <= 0; shape (16*N, 12*N)."""
    N = cfg.horizon

    C_foot = np.array(
        [
            [ 1,  0, -cfg.mu],
            [-1,  0, -cfg.mu],
            [ 0,  1, -cfg.mu],
            [ 0, -1, -cfg.mu],
        ],
        dtype=np.float64,
    )

    rows, cols, values = [], [], []

    for i in range(N):
        for j in range(NUM_LEGS):
            row0 = PYRAMID_ROWS * (NUM_LEGS * i + j)
            col0 = NU * i + 3 * j

            for r in range(PYRAMID_ROWS):
                for c in range(3):
                    rows.append(row0 + r)
                    cols.append(col0 + c)
                    values.append(C_foot[r, c])

    return sparse.csc_matrix(
        (values, (rows, cols)),
        shape=(PYRAMID_ROWS * NUM_LEGS * N, NU * N),
    )


def fz_selector(cfg: Config) -> sparse.csc_matrix:
    """Select fz for every foot and timestep."""
    N = cfg.horizon
    rows = np.arange(NUM_LEGS * N)
    cols = np.array(
        [
            NU * i + 3 * j + 2
            for i in range(N)
            for j in range(NUM_LEGS)
        ]
    )

    return sparse.csc_matrix(
        (np.ones(NUM_LEGS * N), (rows, cols)),
        shape=(NUM_LEGS * N, NU * N),
    )


def constraint_matrix(cfg: Config) -> sparse.csc_matrix:
    """A = [C_pyr; Fz]."""
    return sparse.vstack(
        [friction_pyramid(cfg), fz_selector(cfg)],
        format="csc",
    )


def constraint_bounds(
    cfg: Config,
    lb_f: np.ndarray,
    ub_f: np.ndarray,
):
    """Combine friction and vertical-force bounds."""
    num_pyramid = PYRAMID_ROWS * NUM_LEGS * cfg.horizon

    lower = np.concatenate(
        [
            np.full(num_pyramid, -np.inf),
            lb_f,
        ]
    )
    upper = np.concatenate(
        [
            np.zeros(num_pyramid),
            ub_f,
        ]
    )
    return lower, upper


def hessian_from_B(
    cfg: Config,
    B_qp: np.ndarray,
) -> np.ndarray:
    """P = 2 * (B.T @ W @ B + R*I)."""
    N = cfg.horizon
    weights = np.tile(cfg.Q_diag, N)

    # Multiplication by diagonal W without allocating a dense W matrix.
    P = 2.0 * (
        B_qp.T @ (weights[:, None] * B_qp)
        + cfg.R_weight * np.eye(NU * N)
    )

    # OSQP uses only the upper triangular part of P.
    return 0.5 * (P + P.T)


def gradient_from_B(
    cfg: Config,
    B_qp: np.ndarray,
    err: np.ndarray,
) -> np.ndarray:
    """q = 2 * B.T @ W @ err."""
    weights = np.tile(cfg.Q_diag, cfg.horizon)
    return 2.0 * B_qp.T @ (weights * err)


def upper_triangular_csc(P: np.ndarray) -> sparse.csc_matrix:
    """
    Preserve every upper-triangular entry, including explicit zeros.

    This is essential because the numerical values of P change every
    MPC tick, but the generated solver requires a fixed sparsity pattern.
    """
    rows, cols = np.triu_indices(P.shape[0])
    values = P[rows, cols]

    return sparse.csc_matrix(
        (values, (rows, cols)),
        shape=P.shape,
    )


def initial_problem_data(cfg: Config):
    """Build fixed problem structure for OSQP code generation."""
    n = NU * cfg.horizon

    P = upper_triangular_csc(
        np.eye(n, dtype=np.float64) * (2.0 * cfg.R_weight)
    )
    q = np.zeros(n, dtype=np.float64)

    A = constraint_matrix(cfg)

    num_constraints = A.shape[0]
    l = np.full(num_constraints, -np.inf, dtype=np.float64)
    u = np.full(num_constraints, np.inf, dtype=np.float64)

    num_pyramid = PYRAMID_ROWS * NUM_LEGS * cfg.horizon
    l[:num_pyramid] = -np.inf
    u[:num_pyramid] = 0.0

    return P, q, A, l, u


def generate(cfg: Config, target: str = "components/qp"):
    """Generate standalone embedded C code using native OSQP codegen."""
    target_c = Path(target) / "c"
    target_c.parent.mkdir(parents=True, exist_ok=True)

    P, q, A, l, u = initial_problem_data(cfg)

    solver = osqp.OSQP()
    solver.setup(
        P=P,
        q=q,
        A=A,
        l=l,
        u=u,
        max_iter=50,
        eps_abs=1e-3,
        eps_rel=1e-3,
        polishing=False,
        warm_starting=True,
        verbose=False,
    )

    solver.codegen(
        str(target_c),
        parameters="matrices",
        force_rewrite=True,
        use_float=True,
        printing_enable=False,
        profiling_enable=False,
        interrupt_enable=False,
        include_codegen_src=True,
    )

    print(f"Generated OSQP C solver: {target_c.resolve()}")
    print("Precision: float32")
    print("Runtime-updatable matrices: P, A")
    print("Runtime-updatable vectors: q, l, u")
    print(f"Horizon: {cfg.horizon}")
    print(f"Decision variables: {NU * cfg.horizon}")
    print(f"Constraints: {A.shape[0]}")


def test(cfg: Config):
    """Solve a random condensed MPC QP and validate feasibility."""
    rng = np.random.default_rng(0)
    N = cfg.horizon
    n = NU * N

    # Random lower-block-triangular condensed dynamics matrix.
    B = np.zeros((NX * N, n))
    for i in range(N):
        for j in range(i + 1):
            B[
                NX * i:NX * (i + 1),
                NU * j:NU * (j + 1),
            ] = 0.01 * rng.standard_normal((NX, NU))

    err = rng.standard_normal(NX * N)

    contact = rng.random((N, NUM_LEGS)) > 0.4
    lb_f = np.where(contact, 5.0, 0.0).reshape(-1)
    ub_f = np.where(contact, 150.0, 0.0).reshape(-1)

    P_dense = hessian_from_B(cfg, B)
    P = upper_triangular_csc(P_dense)
    q = gradient_from_B(cfg, B, err)

    A = constraint_matrix(cfg)
    l, u = constraint_bounds(cfg, lb_f, ub_f)

    solver = osqp.OSQP()
    solver.setup(
        P=P,
        q=q,
        A=A,
        l=l,
        u=u,
        max_iter=5000,
        eps_abs=1e-6,
        eps_rel=1e-6,
        polishing=True,
        verbose=False,
    )

    result = solver.solve()

    print("OSQP status:", result.info.status)

    if result.x is None or result.info.status_val not in (1, 2):
        raise RuntimeError("OSQP failed to find a usable solution")

    U = result.x

    # Validate all constraints numerically.
    constraint_values = A @ U
    lower_violation = np.maximum(l - constraint_values, 0.0)
    upper_violation = np.maximum(constraint_values - u, 0.0)
    max_violation = max(
        float(np.max(lower_violation)),
        float(np.max(upper_violation)),
    )

    objective = 0.5 * U @ (P_dense @ U) + q @ U

    print(f"Objective: {objective:.8f}")
    print(f"Max constraint violation: {max_violation:.3e}")
    print(f"Decision vector size: {U.size}")

    if max_violation > 1e-4:
        raise AssertionError("Solution violates constraints")

    print("Sanity check passed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--horizon", type=int, default=10)
    parser.add_argument(
        "--target",
        default="components/qp",
        help="ESP-IDF component directory",
    )
    args = parser.parse_args()

    if args.horizon < 1:
        parser.error("--horizon must be >= 1")

    config = Config(horizon=args.horizon)

    if args.test:
        test(config)
    else:
        generate(config, args.target)
