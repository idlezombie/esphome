# MPU6050 Custom ESPHome Component

A fully featured ESPHome external component for the MPU6050 including:

- Accelerometer (±4g)
- Gyroscope (±250°/s)
- Temperature sensor
- Auto‑calibration at boot
- Manual recalibration button
- Angle output (0–180° tilt)
- Clean external component structure

## Installation

Copy the `external_components/mpu6050_custom` folder into your ESPHome config directory.

Then include it in your YAML:

```yaml
external_components:
  - source: ./external_components/mpu6050_custom