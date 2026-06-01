#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace mpu6050_tilt {

class MPU6050Tilt : public PollingComponent, public i2c::I2CDevice {
 public:
  MPU6050Tilt();

  void setup() override;
  void update() override;

  void calibrate();

  bool read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);

  void set_angle_x_sensor(sensor::Sensor *s) { angle_x_sensor_ = s; }
  void set_angle_y_sensor(sensor::Sensor *s) { angle_y_sensor_ = s; }
  void set_angle_z_sensor(sensor::Sensor *s) { angle_z_sensor_ = s; }
  void set_position_sensor(sensor::Sensor *s) { position_sensor_ = s; }

  void set_raw_accel_x_sensor(sensor::Sensor *s) { raw_accel_x_sensor_ = s; }
  void set_raw_accel_y_sensor(sensor::Sensor *s) { raw_accel_y_sensor_ = s; }
  void set_raw_accel_z_sensor(sensor::Sensor *s) { raw_accel_z_sensor_ = s; }
  void set_raw_gyro_x_sensor(sensor::Sensor *s) { raw_gyro_x_sensor_ = s; }
  void set_raw_gyro_y_sensor(sensor::Sensor *s) { raw_gyro_y_sensor_ = s; }
  void set_raw_gyro_z_sensor(sensor::Sensor *s) { raw_gyro_z_sensor_ = s; }

  void set_axis_index(uint8_t axis) { axis_index_ = axis; }
  void set_gyro_axis_index(uint8_t axis) { gyro_axis_index_ = axis; }
  void set_accel_fs_sel(uint8_t fs) { accel_fs_sel_ = fs; }
  void set_gyro_fs_sel(uint8_t fs) { gyro_fs_sel_ = fs; }
  void set_dlpf_cfg(uint8_t cfg) { dlpf_cfg_ = cfg; }
  void set_closed_angle(float angle) { closed_angle_ = angle; }
  void set_open_angle(float angle) { open_angle_ = angle; }

 protected:
  bool configure_mpu_();
  void compute_position_();

  int16_t offset_ax_{0};
  int16_t offset_ay_{0};
  int16_t offset_az_{0};
  int16_t offset_gx_{0};
  int16_t offset_gy_{0};
  int16_t offset_gz_{0};

  float angle_x_{0.0f};
  float angle_y_{0.0f};
  float angle_z_{0.0f};
  float tilt_angle_{0.0f};      // accel[axis] + gyro[gyro_axis] for position
  float angle_smoothed_{0.0f};

  float last_published_x_{9999.0f};
  float last_published_y_{9999.0f};
  float last_published_z_{9999.0f};
  float last_published_position_{9999.0f};

  uint8_t axis_index_{0};       // which accel angle formula (x/y/z)
  uint8_t gyro_axis_index_{1};  // which gyro axis (0=x, 1=y, 2=z), default y
  uint8_t accel_fs_sel_{1};
  uint8_t gyro_fs_sel_{1};
  uint8_t dlpf_cfg_{4};

  float accel_scale_{16384.0f};
  float gyro_scale_{65.5f};

  float closed_angle_{0.0f};
  float open_angle_{90.0f};

  bool setup_complete_{false};

  sensor::Sensor *angle_x_sensor_{nullptr};
  sensor::Sensor *angle_y_sensor_{nullptr};
  sensor::Sensor *angle_z_sensor_{nullptr};
  sensor::Sensor *position_sensor_{nullptr};
  sensor::Sensor *raw_accel_x_sensor_{nullptr};
  sensor::Sensor *raw_accel_y_sensor_{nullptr};
  sensor::Sensor *raw_accel_z_sensor_{nullptr};
  sensor::Sensor *raw_gyro_x_sensor_{nullptr};
  sensor::Sensor *raw_gyro_y_sensor_{nullptr};
  sensor::Sensor *raw_gyro_z_sensor_{nullptr};
};

}  // namespace mpu6050_tilt
}  // namespace esphome
