#ifndef SERVO_h
#define SERVO_h

struct ServoLimit {
	float min_deg;
	float max_deg;

	constexpr float clamp(float angle) const {
		return angle < min_deg ? min_deg : (angle > max_deg ? max_deg : angle);
	}
};

struct LegServoLimits {
	ServoLimit coxa;
	ServoLimit femur;
	ServoLimit tibia;
};



namespace ServoConfig {
    constexpr float SERVO_FREQ = 50.0f;
    constexpr float SERVO_MIN_PULSE = 160.0f;
    constexpr float SERVO_MAX_PULSE = 560.0f;

	constexpr LegServoLimits limits[4] = {
		{ {  60.0f, 240.0f }, {  50.0f, 250.0f }, {  40.0f, 260.0f } },  // Front Left
		{ {  60.0f, 240.0f }, {  50.0f, 250.0f }, {  40.0f, 260.0f } },  // Front Right
		{ {  60.0f, 240.0f }, {  50.0f, 250.0f }, {  40.0f, 260.0f } },  // Back Right
		{ {  60.0f, 240.0f }, {  50.0f, 250.0f }, {  40.0f, 260.0f } },  // Back Left
	};
}

#endif