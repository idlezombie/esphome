#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace mpu6050_tilt {

class MPU6050Tilt : public PollingComponent, public i2c::I2CDevice {
 public:
  // Constructor
  MPU6050Tilt() = default;

  // Setup and update
  void setup() override;
  void update() override;

  // Calibration
  void calibrate();

  // Raw read helper
  void read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz);

  // Sensor setters
  void set_angle_x_sensor(sensor::Sensor *s) { angle_x_sensor_ = s; }
  void set_angle_y_sensor(sensor::Sensor *s) { angle_y_sensor_ = s; }
  void set_angle_z_sensor(sensor::Sensor *s) { angle_z_sensor_ = s; }

 protected:
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

  // Output sensors
  sensor::Sensor *angle_x_sensor_{nullptr};
  sensor::Sensor *angle_y_sensor_{nullptr};
  sensor::Sensor *angle_z_sensor_{nullptr};
};

}  // namespace mpu6050_tilt
}  // namespace esphome