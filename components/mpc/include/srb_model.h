#pragma once

#include "motion_config.h"
#include "types.h"
#include "eigen.h"

// single rigid body
// namespace srb {

constexpr int NX = 13;       // state dimension
constexpr int NU = 12;       // input dimension (4 feet x 3)
constexpr int NUM_LEGS = 4;

using StateVec = Mat<float, NX, 1>;
using InputVec = Mat<float, NU, 1>;
using MatA = Mat<float, NX, NX>;
using MatB = Mat<float, NX, NU>;

// Index offsets into the 13x1 state vector
constexpr int IDX_THETA = 0;  // roll, pitch, yaw
constexpr int IDX_POS = 3;
constexpr int IDX_OMEGA = 6;
constexpr int IDX_VEL = 9;
constexpr int IDX_G = 12;

// 13 x 1
struct State {
    float roll = 0, pitch = 0, yaw = 0;  // 3 = theta
    Vec3f pos = Vec3f::Zero();             // 3 = p
    Vec3f angular_vel = Vec3f::Zero();     // 3 = omega
    Vec3f linear_vel = Vec3f::Zero();      // 3 = p dot
    float g = GRAVITY;                    // 1 = gravity slot (constant)

    StateVec toVector() const {
        StateVec x;
        x << roll, pitch, yaw, pos, angular_vel, linear_vel, g;
        return x;
    }

    static State fromVector(const StateVec& x) {
        State s;
        s.roll = x(0);
        s.pitch = x(1);
        s.yaw = x(2);
        s.pos = x.segment<3>(IDX_POS);
        s.angular_vel = x.segment<3>(IDX_OMEGA);
        s.linear_vel = x.segment<3>(IDX_VEL);
        s.g = x(IDX_G);
        return s;
    }
};

// 12 x 1
struct Input {
    Array<Vec3f, NUM_LEGS> foot_ground_force{};  // f1..f4, world frame

    InputVec toVector() const {
        InputVec u;
        for (int i = 0; i < NUM_LEGS; i++) u.segment<3>(3 * i) = foot_ground_force[i];
        return u;
    }

    static Input fromVector(const InputVec& u) {
        Input in;
        for (int i = 0; i < NUM_LEGS; i++) in.foot_ground_force[i] = u.segment<3>(3 * i);
        return in;
    }
};

// Skew-symmetric matrix so that skew(r) * f == r.cross(f)
inline Mat3f skew(const Vec3f& r) {
    Mat3f S;
    S << 0, -r.z(), r.y(),
         r.z(), 0, -r.x(),
        -r.y(), r.x(), 0;
    return S;
}

inline Mat3f rotZ(float yaw) {
    const float c = cosf(yaw), s = sinf(yaw);
    Mat3f R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

// Continuous-time model: x_dot = A x + B u
struct Model {
    MatA A = MatA::Zero();  // 13 x 13
    MatB B = MatB::Zero();  // 13 x 12

    // A depends only on yaw. Gravity enters through the constant state slot:
    // p_ddot row, column IDX_G holds [0, 0, 1]^T, and x(IDX_G) = g = -9.81.
    void buildA(float yaw) {
        A.setZero();
        // theta_dot = Rz^T * omega
        A.block<3, 3>(IDX_THETA, IDX_OMEGA) = rotZ(yaw).transpose();
        // p_dot = v
        A.block<3, 3>(IDX_POS, IDX_VEL) = Mat3f::Identity();
        // v_dot += [0,0,1] * g
        A.block<3, 1>(IDX_VEL, IDX_G) = Vec3f(0, 0, 1);
    }

    // B depends on foot positions relative to the CoM (world-frame vectors r_i),
    // the body inertia expressed in the world frame, and the mass.
    void buildB(const Array<Vec3f, NUM_LEGS>& r, const Mat3f& I_world_inv, float mass) {
        B.setZero();
        for (int i = 0; i < NUM_LEGS; i++) {
            B.block<3, 3>(IDX_OMEGA, 3 * i) = I_world_inv * skew(r[i]);  // omega_dot
            B.block<3, 3>(IDX_VEL, 3 * i) = Mat3f::Identity() / mass;     // v_dot
        }
    }

    // World-frame inertia approximated using yaw only: I = Rz * I_body * Rz^T
    static Mat3f worldInertiaInv(const Mat3f& I_body, float yaw) {
        const Mat3f R = rotZ(yaw);
        return (R * I_body * R.transpose()).inverse();
    }
};

// Discrete-time model: x[k+1] = Ad x[k] + Bd u[k]
struct DiscreteModel {
    MatA Ad;
    MatB Bd;

    static DiscreteModel discretize(const Model& m, float dt) {
        DiscreteModel d;
        MatA Adt = m.A * dt;
        d.Ad = Adt.exp();   // matrix exponential
        d.Bd = m.B * dt;    // first-order approximation
        return d;
    }
};

// }  // namespace