#include "esphome.h"
#include "Wire.h"

class MPU6050Custom : public PollingComponent, public Sensor {
 public:
  // Accelerometer sensors
  Sensor *accel_x = new Sensor();
  Sensor *accel_y = new Sensor();
  Sensor *accel_z = new Sensor();

  // Gyroscope sensors
  Sensor *gyro_x = new Sensor();
  Sensor *gyro_y = new Sensor();
  Sensor *gyro_z = new Sensor();

  // Temperature sensor
  Sensor *temperature = new Sensor();

  // Angle sensors (0–180°)
  Sensor *angle_x = new Sensor();
  Sensor *angle_y = new Sensor();

  // Calibration offsets
  float accel_x_offset = 0.0;
  float accel_y_offset = 0.0;
  float accel_z_offset = 0.0;

  float gyro_x_offset = 0.0;
  float gyro_y_offset = 0.0;
  float gyro_z_offset = 0.0;

  MPU6050Custom() : PollingComponent(1000) {}

  void setup() override {
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();

    // Set accelerometer sensitivity to ±4g
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    // Set gyroscope sensitivity to ±250°/s
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Auto-calibrate on boot
    calibrate();
  }

  // -------------------------
  // Auto Calibration Routine
  // -------------------------
  void calibrate() {
    const int samples = 200;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < samples; i++) {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 14, true);

      int16_t raw_ax = Wire.read() << 8 | Wire.read();
      int16_t raw_ay = Wire.read() << 8 | Wire.read();
      int16_t raw_az = Wire.read() << 8 | Wire.read();

      int16_t raw_temp = Wire.read() << 8 | Wire.read();

      int16_t raw_gx = Wire.read() << 8 | Wire.read();
      int16_t raw_gy = Wire.read() << 8 | Wire.read();
      int16_t raw_gz = Wire.read() << 8 | Wire.read();

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
  }

  // Allow YAML to trigger recalibration
  void do_recalibrate() {
    calibrate();
  }

  void update() override {
    // --- Accelerometer ---
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    int16_t raw_ax = Wire.read() << 8 | Wire.read();
    int16_t raw_ay = Wire.read() << 8 | Wire.read();
    int16_t raw_az = Wire.read() << 8 | Wire.read();

    float ax = (raw_ax / 8192.0) - accel_x_offset;
    float ay = (raw_ay / 8192.0) - accel_y_offset;
    float az = (raw_az / 8192.0) - accel_z_offset;

    accel_x->publish_state(ax);
    accel_y->publish_state(ay);
    accel_z->publish_state(az);

    // --- Temperature ---
    Wire.beginTransmission(0x68);
    Wire.write(0x41);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2, true);

    int16_t raw_temp = Wire.read() << 8 | Wire.read();
    float temp_c = (raw_temp / 340.0) + 36.53;
    temperature->publish_state(temp_c);

    // --- Gyroscope ---
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    int16_t raw_gx = Wire.read() << 8 | Wire.read();
    int16_t raw_gy = Wire.read() << 8 | Wire.read();
    int16_t raw_gz = Wire.read() << 8 | Wire.read();

    float gx = (raw_gx / 131.0) - gyro_x_offset;
    float gy = (raw_gy / 131.0) - gyro_y_offset;
    float gz = (raw_gz / 131.0) - gyro_z_offset;

    gyro_x->publish_state(gx);
    gyro_y->publish_state(gy);
    gyro_z->publish_state(gz);

    // --- Angle Calculation (0–180°) ---
    float angle_x_deg = abs(atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI);
    float angle_y_deg = abs(atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI);

    angle_x->publish_state(angle_x_deg);
    angle_y->publish_state(angle_y_deg);
  }
};