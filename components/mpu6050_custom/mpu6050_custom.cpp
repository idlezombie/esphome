#include "mpu6050_custom.h"
#include "esphome/core/log.h"
#include <cmath>

static const char *TAG = "mpu6050_custom";

void MPU6050Custom::setup() {
  ESP_LOGI(TAG, "Initializing MPU6050 using ESPHome I2C API...");

  // Wake up MPU6050 (PWR_MGMT_1 = 0)
  this->write_byte(0x6B, 0x00);

  // Accelerometer config: ±4g (ACCEL_CONFIG = 0x08)
  this->write_byte(0x1C, 0x08);

  // Gyro config: ±250°/s (GYRO_CONFIG = 0x00)
  this->write_byte(0x1B, 0x00);

  // Auto-calibrate on boot
  this->calibrate();
}

void MPU6050Custom::calibrate() {
  ESP_LOGI(TAG, "Calibrating MPU6050...");

  const int samples = 200;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;

  uint8_t data[14];

  for (int i = 0; i < samples; i++) {
    if (!this->read_bytes(0x3B, data, 14)) {
      ESP_LOGW(TAG, "I2C read failed during calibration");
      continue;
    }

    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];

    int16_t raw_gx = (data[8] << 8) | data[9];
    int16_t raw_gy = (data[10] << 8) | data[11];
    int16_t raw_gz = (data[12] << 8) | data[13];

    ax_sum += raw_ax / 8192.0;
    ay_sum += raw_ay / 8192.0;
    az_sum += raw_az / 8192.0;

    gx_sum += raw_gx / 131.0;
    gy_sum += raw_gy / 131.0;
    gz_sum += raw_gz / 131.0;

    delay(5);
  }

  accel_x_offset = ax_sum / samples;
  accel_y_offset = ay_sum / samples;
  accel_z_offset = (az_sum / samples) - 1.0;  // remove gravity

  gyro_x_offset = gx_sum / samples;
  gyro_y_offset = gy_sum / samples;
  gyro_z_offset = gz_sum / samples;

  ESP_LOGI(TAG, "Calibration complete.");
}

void MPU6050Custom::do_recalibrate() {
  ESP_LOGI(TAG, "Manual recalibration triggered.");
  this->calibrate();
}

void MPU6050Custom::update() {
  uint8_t data[14];

  // Read all accel, temp, gyro registers in one burst
  if (!this->read_bytes(0x3B, data, 14)) {
    ESP_LOGW(TAG, "I2C read failed during update()");
    return;
  }

  // --- Accelerometer ---
  int16_t raw_ax = (data[0] << 8) | data[1];
  int16_t raw_ay = (data[2] << 8) | data[3];
  int16_t raw_az = (data[4] << 8) | data[5];

  float ax = (raw_ax / 8192.0) - accel_x_offset;
  float ay = (raw_ay / 8192.0) - accel_y_offset;
  float az = (raw_az / 8192.0) - accel_z_offset;

  accel_x->publish_state(ax);
  accel_y->publish_state(ay);
  accel_z->publish_state(az);

  // --- Temperature ---
  int16_t raw_temp = (data[6] << 8) | data[7];
  float temp_c = (raw_temp / 340.0) + 36.53;
  temperature->publish_state(temp_c);

  // --- Gyroscope ---
  int16_t raw_gx = (data[8] << 8) | data[9];
  int16_t raw_gy = (data[10] << 8) | data[11];
  int16_t raw_gz = (data[12] << 8) | data[13];

  float gx = (raw_gx / 131.0) - gyro_x_offset;
  float gy = (raw_gy / 131.0) - gyro_y_offset;
  float gz = (raw_gz / 131.0) - gyro_z_offset;

  gyro_x->publish_state(gx);
  gyro_y->publish_state(gy);
  gyro_z->publish_state(gz);

  // --- Angle Calculation (0–180°) ---
  float angle_x_deg = std::abs(std::atan2(ax, std::sqrt(ay * ay + az * az)) * 180.0 / M_PI);
  float angle_y_deg = std::abs(std::atan2(ay, std::sqrt(ax * ax + az * az)) * 180.0 / M_PI);

  angle_x->publish_state(angle_x_deg);
  angle_y->publish_state(angle_y_deg);
}