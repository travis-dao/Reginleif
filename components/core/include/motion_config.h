#ifndef MOTION_CONFIG_h
#define MOTION_CONFIG_h

constexpr int NUM_LEGS   = 4;
constexpr int NUM_JOINTS = 12;   // leg-major (FL, FR, BR, BL) x (coxa, femur, tibia)

// ---- policy interface (must match the Python training env exactly) ----
// One frame = 3 gyro + 3 gravity + 3 cmd + 12 (q - q_default) + 12 qd + 12 last action
constexpr int   OBS_FRAME_DIM    = 45;
constexpr int   OBS_HISTORY      = 3;                          // oldest -> newest
constexpr int   POLICY_OBS_DIM   = OBS_FRAME_DIM * OBS_HISTORY; // 135
constexpr int   POLICY_ACT_DIM   = NUM_JOINTS;
constexpr float POLICY_PERIOD_S  = 0.02f;                      // 50 Hz
constexpr float ACTION_SCALE_RAD = 0.25f;                      // q_target = q_default + scale * a

// ---- fixed obs scales (same as the Python env) ----
constexpr float GYRO_SCALE    = 0.25f;
constexpr float QD_SCALE      = 0.05f;
constexpr float CMD_SCALE_LIN = 2.0f;
constexpr float CMD_SCALE_ANG = 0.25f;

// ---- safety / startup ----
constexpr float MAX_TILT_RAD    = 1.2f;   // e-stop beyond this
constexpr float MAX_SLEW_RAD_S  = 12.0f;  // per joint; set from servo datasheet
constexpr float STANDUP_RAD_S   = 0.8f;
constexpr float STARTUP_RAMP_S  = 1.0f;
#endif