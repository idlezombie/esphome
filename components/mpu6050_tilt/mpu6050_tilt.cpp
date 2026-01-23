#include "mpu6050_tilt.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace mpu6050_tilt {

static const char *const TAG = "mpu6050_tilt";

void MPU6050Tilt::setup() {
  ESP_LOGI(TAG, "Setting up MPU6050 tilt sensor...");

  // Basic MPU6050 init sequence (registers are standard; tweak if needed)
  // Power management: wake up
  this->write_byte(0x6B, 0x00);
  // Accelerometer config: +/- 2g
  this->write_byte(0x1C, 0x00);

  this->calibrate();
}

void MPU6050Tilt::calibrate() {
  ESP_LOGI(TAG, "Calibrating MPU6050 (keep device still)...");
  const int samples = 200;
  float sum_ax = 0.0f;
  float sum_ay = 0.0f;
  float sum_az = 0.0f;

  for (int i = 0; i < samples; i++) {
    uint8_t data[6];
    // ACCEL_XOUT_H register
    if (!this->read_bytes(0x3B, data, 6)) {
      continue;
    }

    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);

    sum_ax += raw_ax;
    sum_ay += raw_ay;
    sum_az += raw_az;

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }

  offset_ax_ = sum_ax / samples;
  offset_ay_ = sum_ay / samples;
  offset_az_ = sum_az / samples;

  ESP_LOGI(TAG, "Calibration done: offsets ax=%.1f ay=%.1f az=%.1f", offset_ax_, offset_ay_, offset_az_);
}

void MPU6050Tilt::update() {
  uint8_t data[6];
  if (!this->read_bytes(0x3B, data, 6)) {
    ESP_LOGW(TAG, "Failed to read accelerometer data");
    return;
  }

  int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
  int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
  int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);

  float ax = (raw_ax - offset_ax_);
  float ay = (raw_ay - offset_ay_);
  float az = (raw_az - offset_az_);

  // Simple tilt angles from accelerometer only (in degrees)
  float angle_x = std::atan2(ay, std::sqrt(ax * ax + az * az)) * 180.0f / float(M_PI);
  float angle_y = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * 180.0f / float(M_PI);

  if (this->angle_x_sensor_ != nullptr)
    this->angle_x_sensor_->publish_state(angle_x);
  if (this->angle_y_sensor_ != nullptr)
    this->angle_y_sensor_->publish_state(angle_y);
  if (this->angle_z_sensor_ != nullptr) {
    // For now, publish 0 or derive something else if you want
    this->angle_z_sensor_->publish_state(0.0f);
  }
}

}  // namespace mpu6050_tilt
}  // namespace esphome