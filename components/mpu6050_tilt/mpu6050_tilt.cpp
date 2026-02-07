#include "mpu6050_tilt.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace mpu6050_tilt {

static const char *const TAG = "mpu6050_tilt";

// MPU6050 register addresses
static const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU6050_REG_GYRO_XOUT_H = 0x43;

// Scale factors
static const float ACCEL_SCALE = 16384.0f;  // ±2g
static const float GYRO_SCALE = 131.0f;     // ±250°/s

void MPU6050Tilt::setup() {
  ESP_LOGI(TAG, "Initializing MPU6050...");

  // Wake up the MPU6050
  this->write_byte(MPU6050_REG_PWR_MGMT_1, 0x00);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  this->calibrate();
}

void MPU6050Tilt::calibrate() {
  ESP_LOGI(TAG, "Calibrating MPU6050... keep device still");

  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    this->read_raw(ax, ay, az, gx, gy, gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  offset_ax_ = sum_ax / samples;
  offset_ay_ = sum_ay / samples;
  offset_az_ = (sum_az / samples) - 16384;  // remove gravity
  offset_gx_ = sum_gx / samples;
  offset_gy_ = sum_gy / samples;
  offset_gz_ = sum_gz / samples;

  ESP_LOGI(TAG, "Calibration complete");
}

void MPU6050Tilt::read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                           int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[14];
  this->read_bytes(MPU6050_REG_ACCEL_XOUT_H, data, 14);

  ax = (data[0] << 8) | data[1];
  ay = (data[2] << 8) | data[3];
  az = (data[4] << 8) | data[5];
  gx = (data[8] << 8) | data[9];
  gy = (data[10] << 8) | data[11];
  gz = (data[12] << 8) | data[13];
}

void MPU6050Tilt::update() {
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  this->read_raw(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz);

  // Apply calibration offsets
  float ax = (raw_ax - offset_ax_) / ACCEL_SCALE;
  float ay = (raw_ay - offset_ay_) / ACCEL_SCALE;
  float az = (raw_az - offset_az_) / ACCEL_SCALE;

  float gx = (raw_gx - offset_gx_) / GYRO_SCALE;
  float gy = (raw_gy - offset_gy_) / GYRO_SCALE;
  float gz = (raw_gz - offset_gz_) / GYRO_SCALE;

  // Compute accelerometer angles
  float accel_angle_x = atan2(ay, az) * 180.0f / M_PI;
  float accel_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

  // Complementary filter
  const float alpha = 0.98f;
  const float dt = this->update_interval_ / 1000.0f;

  angle_x_ = alpha * (angle_x_ + gx * dt) + (1 - alpha) * accel_angle_x;
  angle_y_ = alpha * (angle_y_ + gy * dt) + (1 - alpha) * accel_angle_y;
  angle_z_ += gz * dt;  // no accel reference for Z

  // Publish
  if (this->angle_x_sensor_ != nullptr)
    this->angle_x_sensor_->publish_state(angle_x_);

  if (this->angle_y_sensor_ != nullptr)
    this->angle_y_sensor_->publish_state(angle_y_);

  if (this->angle_z_sensor_ != nullptr)
    this->angle_z_sensor_->publish_state(angle_z_);
}

}  // namespace mpu6050_tilt
}  // namespace esphome