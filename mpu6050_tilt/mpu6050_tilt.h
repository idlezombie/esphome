#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace mpu6050_tilt {

class MPU6050Tilt : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_angle_x_sensor(sensor::Sensor *s) { angle_x_sensor_ = s; }
  void set_angle_y_sensor(sensor::Sensor *s) { angle_y_sensor_ = s; }
  void set_angle_z_sensor(sensor::Sensor *s) { angle_z_sensor_ = s; }

  void setup() override;
  void update() override;

  void calibrate();

 protected:
  sensor::Sensor *angle_x_sensor_{nullptr};
  sensor::Sensor *angle_y_sensor_{nullptr};
  sensor::Sensor *angle_z_sensor_{nullptr};

  float offset_ax_{0.0f};
  float offset_ay_{0.0f};
  float offset_az_{0.0f};
};

}  // namespace mpu6050_tilt
}  // namespace esphome