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

MPU6050Tilt::MPU6050Tilt() {
  this->setup_complete_ = false;
}

void MPU6050Tilt::setup() {
  PollingComponent::setup();

  // Wake up the MPU6050
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

  // Apply calibration offsets
  float ax = (raw_ax - offset_ax_) / accel_scale_;
  float ay = (raw_ay - offset_ay_) / accel_scale_;
  float az = (raw_az - offset_az_) / accel_scale_;

  float gx = (raw_gx - offset_gx_) / gyro_scale_;
  float gy = (raw_gy - offset_gy_) / gyro_scale_;
  float gz = (raw_gz - offset_gz_) / gyro_scale_;

  // Publish raw (calibrated) accel and gyro for diagnostics
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

  // Compute accelerometer angles
  float accel_angle_x = atan2(ay, az) * 180.0f / M_PI;
  float accel_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

  // Complementary filter
  const float alpha = 0.98f;
  const float dt = this->update_interval_ / 1000.0f;

  float filtered_x = alpha * (angle_x_ + gx * dt) + (1 - alpha) * accel_angle_x;
  float filtered_y = alpha * (angle_y_ + gy * dt) + (1 - alpha) * accel_angle_y;
  float filtered_z = angle_z_ + gz * dt;  // no accel reference for Z

  // Additional exponential smoothing for noise reduction
  angle_x_ = this->smoothing_factor_ * angle_x_ + (1 - this->smoothing_factor_) * filtered_x;
  angle_y_ = this->smoothing_factor_ * angle_y_ + (1 - this->smoothing_factor_) * filtered_y;
  angle_z_ = filtered_z;  // Z doesn't need extra smoothing

  // Stationary detection: check if gyro readings indicate device is not moving
  float gyro_magnitude = sqrt(gx * gx + gy * gy + gz * gz);
  const int required_stationary_cycles = 10;  // Require 10 cycles (~500ms at 50ms interval) of stillness
  
  if (gyro_magnitude < this->stationary_threshold_) {
    stationary_count_++;
    if (stationary_count_ >= required_stationary_cycles && !is_stationary_) {
      // Device has been stationary - lock to ACCELEROMETER angles (gravity truth),
      // not the integrated filter state which can have gyro drift. Also reset
      // filter state to accel so internal state doesn't keep drifting.
      is_stationary_ = true;
      locked_angle_x_ = accel_angle_x;
      locked_angle_y_ = accel_angle_y;
      angle_x_ = accel_angle_x;
      angle_y_ = accel_angle_y;
    }
  } else {
    // Device is moving - unlock and reset counter
    stationary_count_ = 0;
    if (is_stationary_) {
      is_stationary_ = false;
      locked_angle_x_ = 9999.0f;  // Reset locked values
      locked_angle_y_ = 9999.0f;
    }
  }

  // Publish logic: when stationary, publish locked values (prevents drift);
  // when moving, publish current filtered values
  bool angle_changed = false;
  const float publish_threshold = 0.05f;  // Small threshold to prevent jitter when moving
  
  if (this->angle_x_sensor_ != nullptr) {
    float value_to_publish = is_stationary_ ? locked_angle_x_ : angle_x_;
    // Only publish if value changed (when moving) or first time (when stationary)
    if (fabs(value_to_publish - last_published_x_) >= publish_threshold || 
        (is_stationary_ && last_published_x_ == 9999.0f)) {
      this->angle_x_sensor_->publish_state(value_to_publish);
      last_published_x_ = value_to_publish;
      angle_changed = true;
    }
  }

  if (this->angle_y_sensor_ != nullptr) {
    float value_to_publish = is_stationary_ ? locked_angle_y_ : angle_y_;
    // Only publish if value changed (when moving) or first time (when stationary)
    if (fabs(value_to_publish - last_published_y_) >= publish_threshold || 
        (is_stationary_ && last_published_y_ == 9999.0f)) {
      this->angle_y_sensor_->publish_state(value_to_publish);
      last_published_y_ = value_to_publish;
      angle_changed = true;
    }
  }

  if (this->angle_z_sensor_ != nullptr) {
    if (fabs(angle_z_ - last_published_z_) >= publish_threshold || last_published_z_ == 9999.0f) {
      this->angle_z_sensor_->publish_state(angle_z_);
      last_published_z_ = angle_z_;
    }
  }

  // Position calculation
  if (angle_changed && this->position_sensor_ != nullptr) {
    float angle = (this->axis_index_ == 0) ? 
                  (is_stationary_ ? locked_angle_x_ : angle_x_) : 
                  (is_stationary_ ? locked_angle_y_ : angle_y_);
    float position = (angle - this->closed_angle_) /
                     (this->open_angle_ - this->closed_angle_) * 100.0f;

    if (position < 0.0f)
      position = 0.0f;
    if (position > 100.0f)
      position = 100.0f;

    // Only publish position if it changed
    if (fabs(position - last_published_position_) >= 0.5f || last_published_position_ == 9999.0f) {
      this->position_sensor_->publish_state(position);
      last_published_position_ = position;
    }
  }
}

bool MPU6050Tilt::configure_mpu_() {
  // Compute scale factors from FS_SEL settings
  switch (this->accel_fs_sel_) {
    case 0: // ±2g
      this->accel_scale_ = 16384.0f;
      break;
    case 1: // ±4g
      this->accel_scale_ = 8192.0f;
      break;
    case 2: // ±8g
      this->accel_scale_ = 4096.0f;
      break;
    case 3: // ±16g
      this->accel_scale_ = 2048.0f;
      break;
    default:
      this->accel_scale_ = 8192.0f;  // safe default ±4g
      break;
  }

  switch (this->gyro_fs_sel_) {
    case 0: // ±250 dps
      this->gyro_scale_ = 131.0f;
      break;
    case 1: // ±500 dps
      this->gyro_scale_ = 65.5f;
      break;
    case 2: // ±1000 dps
      this->gyro_scale_ = 32.8f;
      break;
    case 3: // ±2000 dps
      this->gyro_scale_ = 16.4f;
      break;
    default:
      this->gyro_scale_ = 65.5f;  // safe default ±500 dps
      break;
  }

  // Configure DLPF
  if (!this->write_byte(MPU6050_REG_CONFIG, this->dlpf_cfg_)) {
    ESP_LOGE(TAG, "Failed to configure DLPF");
    return false;
  }

  // Configure gyro full-scale range
  uint8_t gyro_val = this->gyro_fs_sel_ << 3;
  if (!this->write_byte(MPU6050_REG_GYRO_CONFIG, gyro_val)) {
    ESP_LOGE(TAG, "Failed to configure gyro range");
    return false;
  }

  // Configure accel full-scale range
  uint8_t accel_val = this->accel_fs_sel_ << 3;
  if (!this->write_byte(MPU6050_REG_ACCEL_CONFIG, accel_val)) {
    ESP_LOGE(TAG, "Failed to configure accel range");
    return false;
  }

  // Set sample rate divider to 0 (use internal rate / (1 + 0))
  if (!this->write_byte(MPU6050_REG_SMPLRT_DIV, 0x00)) {
    ESP_LOGE(TAG, "Failed to configure sample rate");
    return false;
  }

  return true;
}

void MPU6050Tilt::compute_position_() {
  if (this->position_sensor_ == nullptr)
    return;

  if (this->open_angle_ == this->closed_angle_)
    return;

  float angle = (this->axis_index_ == 0) ? this->angle_x_ : this->angle_y_;

  float position = (angle - this->closed_angle_) /
                   (this->open_angle_ - this->closed_angle_) * 100.0f;

  if (position < 0.0f)
    position = 0.0f;
  if (position > 100.0f)
    position = 100.0f;

  this->position_sensor_->publish_state(position);
}

}  // namespace mpu6050_tilt
}  // namespace esphome
