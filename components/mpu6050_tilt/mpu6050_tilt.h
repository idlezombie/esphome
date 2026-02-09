#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace mpu6050_tilt {

class MPU6050Tilt : public PollingComponent, public i2c::I2CDevice {
 public:
  // Constructor
  MPU6050Tilt();

  // Setup and update
  void setup() override;
  void update() override;

  // Calibration
  void calibrate();

  // Raw read helper
  bool read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);

  // Sensor setters
  void set_angle_x_sensor(sensor::Sensor *s) { angle_x_sensor_ = s; }
  void set_angle_y_sensor(sensor::Sensor *s) { angle_y_sensor_ = s; }
  void set_angle_z_sensor(sensor::Sensor *s) { angle_z_sensor_ = s; }

  void set_position_sensor(sensor::Sensor *s) { position_sensor_ = s; }

  // Configuration setters (called from Python)
  void set_axis_index(uint8_t axis) { axis_index_ = axis; }
  void set_accel_fs_sel(uint8_t fs) { accel_fs_sel_ = fs; }
  void set_gyro_fs_sel(uint8_t fs) { gyro_fs_sel_ = fs; }
  void set_dlpf_cfg(uint8_t cfg) { dlpf_cfg_ = cfg; }
  void set_closed_angle(float angle) { closed_angle_ = angle; }
  void set_open_angle(float angle) { open_angle_ = angle; }
  void set_smoothing_factor(float factor) { smoothing_factor_ = factor; }
  void set_deadband_threshold(float threshold) { deadband_threshold_ = threshold; }

 protected:
  // Internal helpers
  bool configure_mpu_();
  void compute_position_();

  // Calibration offsets
  int16_t offset_ax_{0};
  int16_t offset_ay_{0};
  int16_t offset_az_{0};
  int16_t offset_gx_{0};
  int16_t offset_gy_{0};
  int16_t offset_gz_{0};

  // Filtered angles
  float angle_x_{0.0f};
  float angle_y_{0.0f};
  float angle_z_{0.0f};

  // Last published values (for deadband)
  float last_published_x_{0.0f};
  float last_published_y_{0.0f};
  float last_published_z_{0.0f};

  // Configuration
  uint8_t axis_index_{0};   // 0: X, 1: Y
  uint8_t accel_fs_sel_{1}; // default ±4g
  uint8_t gyro_fs_sel_{1};  // default ±500 dps
  uint8_t dlpf_cfg_{4};     // default ~20Hz bandwidth

  float accel_scale_{16384.0f}; // LSB per g
  float gyro_scale_{65.5f};     // LSB per dps

  float closed_angle_{0.0f};
  float open_angle_{90.0f};

  // Setup state tracking
  bool setup_complete_{false};

  // Noise reduction parameters
  float smoothing_factor_{0.8f};    // Exponential smoothing (0-1, higher = more smoothing)
  float deadband_threshold_{0.1f};  // Minimum change to publish (degrees)

  // Output sensors
  sensor::Sensor *angle_x_sensor_{nullptr};
  sensor::Sensor *angle_y_sensor_{nullptr};
  sensor::Sensor *angle_z_sensor_{nullptr};
  sensor::Sensor *position_sensor_{nullptr};
};

}  // namespace mpu6050_tilt
}  // namespace esphome
