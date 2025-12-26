#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "Wire.h"

namespace esphome {
namespace mpu6050_custom {

class MPU6050Custom : public PollingComponent {
 public:
  // Accelerometer sensors
  sensor::Sensor *accel_x = new sensor::Sensor();
  sensor::Sensor *accel_y = new sensor::Sensor();
  sensor::Sensor *accel_z = new sensor::Sensor();

  // Gyroscope sensors
  sensor::Sensor *gyro_x = new sensor::Sensor();
  sensor::Sensor *gyro_y = new sensor::Sensor();
  sensor::Sensor *gyro_z = new sensor::Sensor();

  // Temperature sensor
  sensor::Sensor *temperature = new sensor::Sensor();

  // Angle sensors
  sensor::Sensor *angle_x = new sensor::Sensor();
  sensor::Sensor *angle_y = new sensor::Sensor();

  // Calibration offsets
  float accel_x_offset = 0.0;
  float accel_y_offset = 0.0;
  float accel_z_offset = 0.0;

  float gyro_x_offset = 0.0;
  float gyro_y_offset = 0.0;
  float gyro_z_offset = 0.0;

  MPU6050Custom() : PollingComponent(1000) {}

  void setup() override;
  void update() override;

  void calibrate();
  void do_recalibrate();
};

}  // namespace mpu6050_custom
}  // namespace esphome