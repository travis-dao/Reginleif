#ifndef IMU_h
#define IMU_h

#include <cstdint>
#include "esp_err.h"

class IMU {
    public:
        struct Config;
        
        struct EulerAngles {
            float roll;
            float pitch;
            float yaw;
        };

        struct Raw {
            int16_t ax, ay, az;
            int16_t temp;
            int16_t gx, gy, gz;
        };

        struct Data {
            float ax, ay, az;      /* g */
            float gx, gy, gz;      /* deg/s (gyro bias already removed) */
            float temp_c;          /* degrees Celsius */
        };
    
        esp_err_t init();
        esp_err_t read(Data &d);
        esp_err_t getAngles(EulerAngles &angles);
        esp_err_t reset();
};

#endif