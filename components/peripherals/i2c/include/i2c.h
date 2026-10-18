#ifndef I2C_h
#define I2C_h

#include "esp_log.h"
#include "mpu6050.h"
#include "ism330.h"


constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t SDA_GPIO = GPIO_NUM_3;
constexpr gpio_num_t SCL_GPIO = GPIO_NUM_4;

class I2CBus {
    public:
        esp_err_t init() {
            esp_log_level_set("gpio", ESP_LOG_WARN); // disable warnings
            printf("Initializing I2C .......... ");
            i2c_master_bus_config_t cfg = {};
            cfg.i2c_port = I2C_PORT;
            cfg.sda_io_num = SDA_GPIO;
            cfg.scl_io_num = SCL_GPIO;
            cfg.clk_source = I2C_CLK_SRC_DEFAULT;
            cfg.glitch_ignore_cnt = 7;
            cfg.flags.enable_internal_pullup = true;
            printf("Success\n");
            return i2c_new_master_bus(&cfg, &bus_);
        };
        i2c_master_bus_handle_t handle() const { return bus_; }
        
    private:
    	i2c_master_bus_handle_t bus_ = nullptr;
};

#endif