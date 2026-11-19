#ifndef LEG_CONTROLLER_h
#define LEG_CONTROLLER_h

#include "types.h"
#include "servo_driver.h"
#include "motion_config.h"

// Thin joint-space interface to the servos.
//
// JOINT CONVENTION (the sim URDF / Python env must use the same one):
//   q[3*leg + {0,1,2}] = {coxa, femur, tibia} in RADIANS, expressed in the
//   "IK domain": the angles returned by IK() for a leg-frame foot position,
//   with the y axis mirrored for right legs, BEFORE the reverse-mounted servo
//   inversion and BEFORE servo-limit clamping. Those two steps are applied
//   only at the servo boundary (toServoDeg).
//   q_default is IK(Offset::neutral_foot_pos_lf[leg]).
class LegController {
    public:
        explicit LegController() = default;

        // Init the driver and compute q_default from the neutral foot pose.
        // Latches a fault if IK fails for the neutral pose.
        void init();

        // Clamp to servo limits, slew-limit, write to servos. Call every loop.
        void setJointTargets(const float dt_s, const Array<float, NUM_JOINTS>& q_target);

        // Slew toward q_default at speed_rad_s. Returns true when reached.
        bool moveToDefault(const float dt_s, const float speed_rad_s);

        const Array<float, NUM_JOINTS>& jointPos() const { return q_; }          // last commanded
        const Array<float, NUM_JOINTS>& defaultPos() const { return q_default_; }

        void estop();   // latch fault, stop updating servos (they hold the last pose)
        bool faulted() const { return faulted_; }

    private:
        void writeServos();

        ServoDriver driver_;
        Array<float, NUM_JOINTS> q_{};
        Array<float, NUM_JOINTS> q_default_{};
        float buf_[NUM_JOINTS] = {};
        bool faulted_ = false;
};

#endif