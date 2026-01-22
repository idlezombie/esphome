import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    UNIT_CELSIUS,
    UNIT_DEGREES,
    UNIT_G,
    ICON_THERMOMETER,
)

# Required so ESPHome knows this is a sensor platform
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

mpu6050_ns = cg.esphome_ns.namespace("mpu6050_custom")
MPU6050Custom = mpu6050_ns.class_("MPU6050Custom", cg.PollingComponent, i2c.I2CDevice)

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"

CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"

CONF_TEMPERATURE = "temperature"

CONF_ANGLE_X = "angle_x"
CONF_ANGLE_Y = "angle_y"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPU6050Custom),

            # Accelerometer
            cv.Optional(CONF_ACCEL_X): sensor.sensor_schema(
                unit_of_measurement=UNIT_G,
                icon="mdi:axis-arrow",
                accuracy_decimals=3,
            ),
            cv.Optional(CONF_ACCEL_Y): sensor.sensor_schema(
                unit_of_measurement=UNIT_G,
                icon="mdi:axis-arrow",
                accuracy_decimals=3,
            ),
            cv.Optional(CONF_ACCEL_Z): sensor.sensor_schema(
                unit_of_measurement=UNIT_G,
                icon="mdi:axis-arrow",
                accuracy_decimals=3,
            ),

            # Gyroscope (no unit — your ESPHome version lacks gyro units)
            cv.Optional(CONF_GYRO_X): sensor.sensor_schema(
                unit_of_measurement=None,
                icon="mdi:rotate-right",
                accuracy_decimals=3,
            ),
            cv.Optional(CONF_GYRO_Y): sensor.sensor_schema(
                unit_of_measurement=None,
                icon="mdi:rotate-right",
                accuracy_decimals=3,
            ),
            cv.Optional(CONF_GYRO_Z): sensor.sensor_schema(
                unit_of_measurement=None,
                icon="mdi:rotate-right",
                accuracy_decimals=3,
            ),

            # Temperature
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
            ),

            # Angles
            cv.Optional(CONF_ANGLE_X): sensor.sensor_schema(
                unit_of_measurement=UNIT_DEGREES,
                icon="mdi:angle-acute",
                accuracy_decimals=1,
            ),
            cv.Optional(CONF_ANGLE_Y): sensor.sensor_schema(
                unit_of_measurement=UNIT_DEGREES,
                icon="mdi:angle-acute",
                accuracy_decimals=1,
            ),
        }
    )
    .extend(i2c.i2c_device_schema(0x68))
)

PLATFORM_SCHEMA = CONFIG_SCHEMA

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_ACCEL_X in config:
        sens = await sensor.new_sensor(config[CONF_ACCEL_X])
        cg.add(var.accel_x.set_parent(sens))
    if CONF_ACCEL_Y in config:
        sens = await sensor.new_sensor(config[CONF_ACCEL_Y])
        cg.add(var.accel_y.set_parent(sens))
    if CONF_ACCEL_Z in config:
        sens = await sensor.new_sensor(config[CONF_ACCEL_Z])
        cg.add(var.accel_z.set_parent(sens))

    if CONF_GYRO_X in config:
        sens = await sensor.new_sensor(config[CONF_GYRO_X])
        cg.add(var.gyro_x.set_parent(sens))
    if CONF_GYRO_Y in config:
        sens = await sensor.new_sensor(config[CONF_GYRO_Y])
        cg.add(var.gyro_y.set_parent(sens))
    if CONF_GYRO_Z in config:
        sens = await sensor.new_sensor(config[CONF_GYRO_Z])
        cg.add(var.gyro_z.set_parent(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.temperature.set_parent(sens))

    if CONF_ANGLE_X in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_X])
        cg.add(var.angle_x.set_parent(sens))
    if CONF_ANGLE_Y in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_Y])
        cg.add(var.angle_y.set_parent(sens))

# Register this as a sensor platform
sensor.register_sensor(MPU6050Custom)