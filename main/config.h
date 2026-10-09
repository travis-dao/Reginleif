#ifndef CONFIG_h
#define CONFIG_h

#include "helpers.h"
#include "servo.h"

namespace Config {
    namespace Leg {
        constexpr float femur_length = 122.64f;
        constexpr float tibia_length = 34.66f + 147.03f;
        constexpr float body_to_coxa_x_offset = 28.58f;
        constexpr float body_to_coxa_z_offset = -9.72f;
    }

    namespace Offset {
        constexpr Vec3 neutral_offset = { 150.0f, 150.0f, 10.0f };

        const Vec3 neutral_vector[4] = {
            {  1,  1, 1 },  // Front Left
            {  1, -1, 1 },  // Front Right
            { -1, -1, 1 },  // Back Right
            { -1,  1, 1 },  // Back Left
        };

        constexpr Vec3 body_offset = { 33.72f, 43.13f, 0.0f };
    }

    namespace Movement {
        constexpr float MAX_STEP_LENGTH_MM = 0.0f;
        constexpr float STEP_HEIGHT_MM = 100.0f;
        constexpr float SWING_DURATION_S = 0.250f;

        constexpr float WALK_OPPOSITE_LEG_Z_OFFSET[4] = {
            5.0f, 5.0f, 10.0f, 10.0f
        };

        constexpr float WALK_ADJACENT_LEG_Z_OFFSET[4] = {
            -8.0f, -8.0f, -6.0f, -6.0f
        };
        
        constexpr int next_leg_gait[5] = {
            /* -1 -> */ 0,
            /*  0 -> */ 2,
            /*  1 -> */ 3,
            /*  2 -> */ 1,
            /*  3 -> */ 0,
        };
    }

    namespace PID {
        constexpr float K_p = 1.5;
        constexpr float K_i = 0.0;
        constexpr float K_d = 0.0;

        constexpr float INTEGRAL_LIMIT_DEG = 30.0;  // max contribution of integral_sum, in degrees
        constexpr float COMPENSATION_LIMIT_DEG = 30.0; // max total correction angle, in degrees
    }

    namespace I2C {
        constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
        constexpr gpio_num_t SDA_GPIO = GPIO_NUM_3;
        constexpr gpio_num_t SCL_GPIO = GPIO_NUM_4;
    }
}

#endif