#include "mpu6050_tilt.h"
#include "esphome/core/log.h"
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace mpu6050_tilt {

static const char *const TAG = "mpu6050_tilt";

// MPU6050 register addresses
static const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;
static const uint8_t MPU6050_REG_CONFIG = 0x1A;
static const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU6050_REG_GYRO_XOUT_H = 0x43;

// Only publish when value changes by at least this much
static const float ANGLE_REPORT_THRESHOLD = 1.5f;    // degrees
static const float POSITION_REPORT_THRESHOLD = 3.0f; // percent

// Gyro Z below this (dps) is treated as zero so angle Z doesn't drift when still
static const float GYRO_Z_DEADZONE = 0.5f;

MPU6050Tilt::MPU6050Tilt() {
  this->setup_complete_ = false;
}

void MPU6050Tilt::setup() {
  PollingComponent::setup();

  if (!this->write_byte(MPU6050_REG_PWR_MGMT_1, 0x00)) {
    ESP_LOGE(TAG, "Failed to wake MPU6050");
    this->mark_failed();
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  if (!this->configure_mpu_()) {
    ESP_LOGE(TAG, "Failed to configure MPU6050");
    this->mark_failed();
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(200));
  this->calibrate();
  this->setup_complete_ = true;
}

void MPU6050Tilt::calibrate() {
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!this->read_raw(ax, ay, az, gx, gy, gz)) {
      continue;
    }

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  offset_ax_ = sum_ax / samples;
  offset_ay_ = sum_ay / samples;
  offset_az_ = sum_az / samples;
  offset_gx_ = sum_gx / samples;
  offset_gy_ = sum_gy / samples;
  offset_gz_ = sum_gz / samples;
}

bool MPU6050Tilt::read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                           int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[14];
  if (!this->read_bytes(MPU6050_REG_ACCEL_XOUT_H, data, 14)) {
    return false;
  }

  ax = (data[0] << 8) | data[1];
  ay = (data[2] << 8) | data[3];
  az = (data[4] << 8) | data[5];
  gx = (data[8] << 8) | data[9];
  gy = (data[10] << 8) | data[11];
  gz = (data[12] << 8) | data[13];
  return true;
}

void MPU6050Tilt::update() {
  if (!this->setup_complete_) {
    return;
  }

  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  if (!this->read_raw(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz)) {
    return;
  }

  float ax = (raw_ax - offset_ax_) / accel_scale_;
  float ay = (raw_ay - offset_ay_) / accel_scale_;
  float az = (raw_az - offset_az_) / accel_scale_;

  float gx = (raw_gx - offset_gx_) / gyro_scale_;
  float gy = (raw_gy - offset_gy_) / gyro_scale_;
  float gz = (raw_gz - offset_gz_) / gyro_scale_;

  // Raw diagnostics (unchanged)
  if (this->raw_accel_x_sensor_ != nullptr)
    this->raw_accel_x_sensor_->publish_state(ax);
  if (this->raw_accel_y_sensor_ != nullptr)
    this->raw_accel_y_sensor_->publish_state(ay);
  if (this->raw_accel_z_sensor_ != nullptr)
    this->raw_accel_z_sensor_->publish_state(az);
  if (this->raw_gyro_x_sensor_ != nullptr)
    this->raw_gyro_x_sensor_->publish_state(gx);
  if (this->raw_gyro_y_sensor_ != nullptr)
    this->raw_gyro_y_sensor_->publish_state(gy);
  if (this->raw_gyro_z_sensor_ != nullptr)
    this->raw_gyro_z_sensor_->publish_state(gz);

  // Accel tilt angles per chip axis (choose axis in config to match your mounting)
  float accel_angle_x = atan2(ay, az) * 180.0f / M_PI;   // tilt in Y,Z plane
  float accel_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
  float accel_angle_z = atan2(ax, az) * 180.0f / M_PI;   // tilt in X,Z plane

  const float alpha = 0.98f;
  const float dt = this->update_interval_ / 1000.0f;

  // Per-axis angles (for optional angle_x / angle_y / angle_z sensors)
  angle_x_ = alpha * (angle_x_ + gx * dt) + (1 - alpha) * accel_angle_x;
  angle_y_ = alpha * (angle_y_ + gy * dt) + (1 - alpha) * accel_angle_y;
  if (this->axis_index_ == 2) {
    angle_z_ = alpha * (angle_z_ + gz * dt) + (1 - alpha) * accel_angle_z;
  } else {
    float gz_eff = (fabsf(gz) < GYRO_Z_DEADZONE) ? 0.0f : gz;
    angle_z_ = angle_z_ + gz_eff * dt;
  }

  // Tilt for position: use accel from selected axis + gyro from selected axis (can differ, e.g. accel Z + gyro Y)
  float accel_tilt = (this->axis_index_ == 0) ? accel_angle_x : ((this->axis_index_ == 1) ? accel_angle_y : accel_angle_z);
  float gyro_rate = (this->gyro_axis_index_ == 0) ? gx : ((this->gyro_axis_index_ == 1) ? gy : gz);
  tilt_angle_ = alpha * (tilt_angle_ + gyro_rate * dt) + (1 - alpha) * accel_tilt;

  angle_smoothed_ = 0.92f * angle_smoothed_ + 0.08f * tilt_angle_;
  float angle_rounded = roundf(angle_smoothed_ * 10.0f) / 10.0f;

  const float unset = 9999.0f;

  if (this->angle_x_sensor_ != nullptr) {
    float val = (this->axis_index_ == 0) ? angle_rounded : (roundf(angle_x_ * 10.0f) / 10.0f);
    if (last_published_x_ == unset || fabsf(val - last_published_x_) >= ANGLE_REPORT_THRESHOLD) {
      this->angle_x_sensor_->publish_state(val);
      last_published_x_ = val;
    }
  }

  if (this->angle_y_sensor_ != nullptr) {
    float val = (this->axis_index_ == 1) ? angle_rounded : (roundf(angle_y_ * 10.0f) / 10.0f);
    if (last_published_y_ == unset || fabsf(val - last_published_y_) >= ANGLE_REPORT_THRESHOLD) {
      this->angle_y_sensor_->publish_state(val);
      last_published_y_ = val;
    }
  }

  if (this->angle_z_sensor_ != nullptr) {
    float val = (this->axis_index_ == 2) ? angle_rounded : (roundf(angle_z_ * 10.0f) / 10.0f);
    if (last_published_z_ == unset || fabsf(val - last_published_z_) >= ANGLE_REPORT_THRESHOLD) {
      this->angle_z_sensor_->publish_state(val);
      last_published_z_ = val;
    }
  }

  // Position: from selected angle, rounded to integer percent; publish only if change >= 1%
  if (this->position_sensor_ != nullptr && this->open_angle_ != this->closed_angle_) {
    float position = (angle_rounded - this->closed_angle_) /
                     (this->open_angle_ - this->closed_angle_) * 100.0f;
    if (position < 0.0f) position = 0.0f;
    if (position > 100.0f) position = 100.0f;
    float position_rounded = roundf(position);

    if (last_published_position_ == unset ||
        fabsf(position_rounded - last_published_position_) >= POSITION_REPORT_THRESHOLD) {
      this->position_sensor_->publish_state(position_rounded);
      last_published_position_ = position_rounded;
    }
  }
}

bool MPU6050Tilt::configure_mpu_() {
  switch (this->accel_fs_sel_) {
    case 0: this->accel_scale_ = 16384.0f; break;
    case 1: this->accel_scale_ = 8192.0f; break;
    case 2: this->accel_scale_ = 4096.0f; break;
    case 3: this->accel_scale_ = 2048.0f; break;
    default: this->accel_scale_ = 8192.0f; break;
  }

  switch (this->gyro_fs_sel_) {
    case 0: this->gyro_scale_ = 131.0f; break;
    case 1: this->gyro_scale_ = 65.5f; break;
    case 2: this->gyro_scale_ = 32.8f; break;
    case 3: this->gyro_scale_ = 16.4f; break;
    default: this->gyro_scale_ = 65.5f; break;
  }

  if (!this->write_byte(MPU6050_REG_CONFIG, this->dlpf_cfg_)) {
    ESP_LOGE(TAG, "Failed to configure DLPF");
    return false;
  }

  if (!this->write_byte(MPU6050_REG_GYRO_CONFIG, this->gyro_fs_sel_ << 3)) {
    ESP_LOGE(TAG, "Failed to configure gyro range");
    return false;
  }

  if (!this->write_byte(MPU6050_REG_ACCEL_CONFIG, this->accel_fs_sel_ << 3)) {
    ESP_LOGE(TAG, "Failed to configure accel range");
    return false;
  }

  if (!this->write_byte(MPU6050_REG_SMPLRT_DIV, 0x00)) {
    ESP_LOGE(TAG, "Failed to configure sample rate");
    return false;
  }

  return true;
}

void MPU6050Tilt::compute_position_() {
  if (this->position_sensor_ == nullptr || this->open_angle_ == this->closed_angle_)
    return;

  float angle = (this->axis_index_ == 0) ? this->angle_x_ :
                (this->axis_index_ == 1) ? this->angle_y_ : this->angle_z_;
  float position = (angle - this->closed_angle_) /
                   (this->open_angle_ - this->closed_angle_) * 100.0f;

  if (position < 0.0f) position = 0.0f;
  if (position > 100.0f) position = 100.0f;

  this->position_sensor_->publish_state(roundf(position));
}

}  // namespace mpu6050_tilt
}  // namespace esphome
