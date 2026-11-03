"""
gen_mpc_qp.py  --  generate embedded C QP solver for the condensed MPC.

pip install cvxpy cvxpygen numpy
source .venv-cvxpy/bin/activate                             # activate venv
python cvxpy/gen_mpc_qp.py --target components/qp           # generate code
python cvxpy/gen_mpc_qp.py --test                           # sanity-check (no codegen)
"""
import argparse
from dataclasses import dataclass, field

import numpy as np
import cvxpy as cp
from cvxpygen import cpg


import shutil
import tempfile
from pathlib import Path

NX = 13
NU = 12
NUM_LEGS = 4
PYRAMID_ROWS = 4

# folders under <code_dir>/c/ that get copied into the target
COPY_DIRS = ("src", "include", "solver_code")

@dataclass
class Config:
    horizon: int = 10
    mu: float = 0.6
    R_weight: float = 1e-5
    # per-state weights, order must match your StateVec
    Q_diag: np.ndarray = field(default_factory=lambda: np.array(
        [100, 100, 10,      # roll pitch yaw
         10, 10, 100,       # x y z
         1, 1, 1,           # omega
         1, 1, 1,           # vel
         0.0]))             # gravity state


def friction_pyramid(cfg: Config) -> np.ndarray:
    """Constant block: C_pyr @ U <= 0, shape (4*NUM_LEGS*N, NU*N)."""
    mu, N = cfg.mu, cfg.horizon
    C_foot = np.array([[ 1,  0, -mu],
                       [-1,  0, -mu],
                       [ 0,  1, -mu],
                       [ 0, -1, -mu]], dtype=float)
    C = np.zeros((PYRAMID_ROWS * NUM_LEGS * N, NU * N))
    for i in range(N):
        for j in range(NUM_LEGS):
            row = PYRAMID_ROWS * (NUM_LEGS * i + j)
            col = NU * i + 3 * j
            C[row:row + PYRAMID_ROWS, col:col + 3] = C_foot
    return C


def fz_selector(cfg: Config) -> np.ndarray:
    """Picks fz of every (step, foot): shape (NUM_LEGS*N, NU*N)."""
    N = cfg.horizon
    S = np.zeros((NUM_LEGS * N, NU * N))
    for i in range(N):
        for j in range(NUM_LEGS):
            S[NUM_LEGS * i + j, NU * i + 3 * j + 2] = 1.0
    return S


def build_problem(cfg: Config):
    N = cfg.horizon

    # ---- parameters (updated every MPC tick from C++) ----
    B_qp = cp.Parameter((NX * N, NU * N), name="B_qp")   # condensed input matrix
    err  = cp.Parameter(NX * N,           name="err")    # A_qp @ x0 - Y
    lb_f = cp.Parameter(NUM_LEGS * N,     name="lb_f")   # fz lower bound (f_min or 0)
    ub_f = cp.Parameter(NUM_LEGS * N,     name="ub_f")   # fz upper bound (f_max or 0)

    # ---- constants ----
    L_sqrt = np.sqrt(np.tile(cfg.Q_diag, N))
    C_pyr = friction_pyramid(cfg)
    Fz = fz_selector(cfg)

    # ---- variable ----
    U = cp.Variable(NU * N, name="U")

    cost = (cp.sum_squares(cp.multiply(L_sqrt, B_qp @ U + err))
            + cfg.R_weight * cp.sum_squares(U))

    constraints = [
        C_pyr @ U <= 0,
        Fz @ U >= lb_f,
        Fz @ U <= ub_f,
    ]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    assert prob.is_dcp(dpp=True), "problem is not DPP, cvxpygen will reject it"
    return prob, dict(U=U, B_qp=B_qp, err=err, lb_f=lb_f, ub_f=ub_f)


def generate(cfg: Config, target="components/qp"):
    prob, _ = build_problem(cfg)

    with tempfile.TemporaryDirectory() as tmp:
        code_dir = Path(tmp) / "mpc_qp_codegen"
        cpg.generate_code(
            prob,
            code_dir=str(code_dir),
            solver="OSQP",
            solver_opts={
                "max_iter": 50,      # hard cap for real-time
                "eps_abs": 1e-3,
                "eps_rel": 1e-3,
                "polishing": False,
                "warm_start": True,
            },
            wrapper=False,           # no pybind wrapper, pure C for ESP32
            prefix="mpc",
        )

        c_dir = code_dir / "c"
        target_c = Path(target) / "c"

        for name in COPY_DIRS:
            src = c_dir / name
            dst = target_c / name
            if not src.is_dir():
                raise FileNotFoundError(
                    f"expected {src} after codegen; your cvxpygen version "
                    f"may use a different layout")
            if dst.exists():
                shutil.rmtree(dst)   # drop stale files from older generations
            shutil.copytree(
                src, dst,
                ignore=shutil.ignore_patterns("cpg_example.c"),  # has main()
            )
            print(f"copied {src.relative_to(code_dir)} -> {dst}")

    print(f"Generated code is in ./{target_c}")


def test(cfg: Config):
    """Compare cvxpy solve against the explicit dense-H formulation."""
    rng = np.random.default_rng(0)
    N = cfg.horizon
    prob, p = build_problem(cfg)

    # random lower-block-triangular B
    B = np.zeros((NX * N, NU * N))
    for i in range(N):
        for j in range(i + 1):
            B[NX*i:NX*(i+1), NU*j:NU*(j+1)] = 0.01 * rng.standard_normal((NX, NU))
    e = rng.standard_normal(NX * N)

    contact = rng.random((N, NUM_LEGS)) > 0.4
    f_min, f_max = 5.0, 150.0
    lb = np.where(contact, f_min, 0.0).reshape(-1)
    ub = np.where(contact, f_max, 0.0).reshape(-1)

    p["B_qp"].value, p["err"].value = B, e
    p["lb_f"].value, p["ub_f"].value = lb, ub
    prob.solve(solver=cp.OSQP, eps_abs=1e-6, eps_rel=1e-6)
    U1 = p["U"].value.copy()

    # reference: same math as your buildCost (H, q) with explicit constraints
    L = np.tile(cfg.Q_diag, N)
    H = 2 * B.T @ (L[:, None] * B) + 2 * cfg.R_weight * np.eye(NU * N)
    q = 2 * B.T @ (L * e)
    U = cp.Variable(NU * N)
    ref = cp.Problem(cp.Minimize(0.5 * cp.quad_form(U, cp.psd_wrap(H)) + q @ U),
                     [friction_pyramid(cfg) @ U <= 0,
                      fz_selector(cfg) @ U >= lb,
                      fz_selector(cfg) @ U <= ub])
    ref.solve(solver=cp.OSQP, eps_abs=1e-6, eps_rel=1e-6)

    print("status:", prob.status, "| max |U - U_ref| =",
          np.abs(U1 - U.value).max())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--horizon", type=int, default=10)
    ap.add_argument("--target", default="components/qp",
                    help="component dir; files land in <target>/c/{src,include,solver_code}")
    a = ap.parse_args()

    cfg = Config(horizon=a.horizon)
    test(cfg) if a.test else generate(cfg, a.target)