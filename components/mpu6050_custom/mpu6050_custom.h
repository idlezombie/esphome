#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

using namespace esphome;

class MPU6050Custom : public PollingComponent, public i2c::I2CDevice {
 public:
  sensor::Sensor *accel_x = new sensor::Sensor();
  sensor::Sensor *accel_y = new sensor::Sensor();
  sensor::Sensor *accel_z = new sensor::Sensor();

  sensor::Sensor *gyro_x = new sensor::Sensor();
  sensor::Sensor *gyro_y = new sensor::Sensor();
  sensor::Sensor *gyro_z = new sensor::Sensor();

  sensor::Sensor *temperature = new sensor::Sensor();

  sensor::Sensor *angle_x = new sensor::Sensor();
  sensor::Sensor *angle_y = new sensor::Sensor();

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