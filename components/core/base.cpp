#include <algorithm>
#include <math.h>

#include "esp_log.h"
#include "freertos/projdefs.h"

#include "base.h"
#include "config.h"

float SPEED_LERP_RATE = 4.0f;

Base::Base()
    : stabilizer_({Configs::PID::K_p, Configs::PID::K_i, Configs::PID::K_d}) {
  this->current_airborne_leg = -1;
  this->current_speed = 0.0f;
  this->state = REST;
  this->target_orientation = Vec3{0.0f, 0.0f, 0.0f};
}

void Base::init() {
  printf("Initialization Sequence\n");
  initI2C();

  // ping connected devices
  // for (uint8_t addr = 0x03; addr < 0x78; addr++) {
  // 	if (i2c_master_probe(this->bus, addr, 100) == ESP_OK) {
  // 		printf("Found device at 0x%02X\n", addr);
  // 	}
  // }

  initServoDriver();
  initLegs();
  // calibrateServos();

  initImu();
}

void Base::initI2C() {
  // init i2c bus
  esp_log_level_set("gpio", ESP_LOG_WARN); // disable warnings
  printf("Initializing I2C .......... ");
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = Configs::I2C::I2C_PORT;
  bus_cfg.sda_io_num = Configs::I2C::SDA_GPIO;
  bus_cfg.scl_io_num = Configs::I2C::SCL_GPIO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_));

  vTaskDelay(pdMS_TO_TICKS(1000));
  printf("Success\n");
}

void Base::initServoDriver() {
  // init servo driver and connect to master i2c bus
  printf("Initializing PCA9685 .......... ");
  pca9685::Pca9685::Config cfg;
  cfg.bus = this->bus_;
  cfg.pwm_freq_hz = ServoConfig::SERVO_FREQ;
  ESP_ERROR_CHECK(this->servo_driver_.init(cfg));

  vTaskDelay(pdMS_TO_TICKS(1000));
  printf("Success\n");
}

void Base::initLegs() {
  // instantiate legs
  printf("Initializing legs to neutral positions .......... ");
  for (int i = 0; i < 4; i++) {
    this->legs_[i] = new Leg({
        i,
        *this,
        this->servo_driver_,
        this->stabilizer_,
    });
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
  printf("Success\n");
}

void Base::calibrateServos() {
  // Calibrate all servos on 1 leg
  // servoSetAngle(this->servo_driver, 3, 90); // coxa
  // servoSetAngle(this->servo_driver, 4, 0); // femur
  // servoSetAngle(this->servo_driver, 5, 180); // tibia

  // Calibrate all servos of 1 leg section
  // for (int i = 0; i < 4; i++) {
  // 	servoSetAngle(this->servo_driver, i * 3, 100);
  // }

  servoSetAngle(this->servo_driver_, 1, 30);
}

void Base::initImu() {
  // init imu and connect to master i2c bus
  esp_log_level_set("mpu6050", ESP_LOG_ERROR);
  printf("Initializing MPU6050 .......... ");
  mpu6050::Mpu6050::Config cfg;
  cfg.bus = this->bus_;
  ESP_ERROR_CHECK(this->imu_.init(cfg));
  vTaskDelay(pdMS_TO_TICKS(1000));
  printf("Success\n");

  // calibrating imu
  vTaskDelay(pdMS_TO_TICKS(500));
  printf("Calibrating MPU6050 .......... ");

  ESP_ERROR_CHECK(this->imu_.calibrateGyro(200));
  ESP_ERROR_CHECK(this->imu_.calibrateLevel(200));

  this->current_orientation = getImuAngles();

  vTaskDelay(pdMS_TO_TICKS(1000));
  printf("Success\n");
}

void Base::updateLegs() {
  // call update on each leg
  for (int i = 0; i < 4; i++) {
    legs_[i]->update();
  }
}

void Base::updateImu() {
  // try to update imu with new measurements
  if (this->imu_.getAngles(this->angles) != ESP_OK) {
    printf("Failed to read IMU\n");
    return;
  }

  // update current orientation -- maybe remove in the future
  this->current_orientation.x = angles.roll;
  this->current_orientation.y = -angles.pitch;
  this->current_orientation.z = 0.0f;

  // printf("roll: %.2f | pitch: %.2f\n", current_orientation.x,
  // current_orientation.y);
}

void Base::move() {
  if (state == REST) {
    return;
  }

  // check if all legs are grounded
  bool all_legs_grounded = true;
  for (int i = 0; i < 4; i++) {
    if (!legs_[i]->isGrounded()) {
      all_legs_grounded = false;
    }
  }

  // find next leg to move
  if (all_legs_grounded) {
    this->current_airborne_leg =
        Configs::Movement::next_leg_gait[this->current_airborne_leg + 1];
    this->legs_[this->current_airborne_leg]->updateState(SWING);
  }
}

void Base::inputController(Vec3 input) {
  this->input = input;

  if (input.magnitude() == 0) {
    state = REST;
  } else {
    state = WALK;
  }
}

void Base::updateSpeed() {
  float target_speed;

  switch (this->state) {
  case REST:
    target_speed = 0.0f;
    break;
  case WALK:
  case RUN:
    target_speed =
        (Configs::Movement::MAX_STEP_LENGTH_MM * 2) /
        (Configs::Movement::SWING_DURATION_S * 3); // calc stance speed
    break;
  default:
    target_speed = 0.0f;
    break;
  }

  // higher SPEED_LERP_RATE = snappier response, lower = smoother/slower
  float t = 1.0f - expf(-SPEED_LERP_RATE * this->dt_s);
  t = std::clamp(t, 0.0f, 1.0f);

  this->current_speed = std::lerp(this->current_speed, target_speed, t);

  // snap to zero to avoid tiny residual velocity keeping legs "moving"
  if (fabsf(this->current_speed) < 0.001f) {
    this->current_speed = 0.0f;
  }
}

void Base::update(float dt_s) {
  this->dt_s = dt_s;

  updateImu();
  stabilizer_.update(dt_s, this->current_orientation);

  // inputController(Vec3 {0.0f, 0.0f, 0.0f});

  // updateSpeed();

  // move();

  updateLegs();
}