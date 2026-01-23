import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_UPDATE_INTERVAL,
    UNIT_DEGREES,
    ICON_EMPTY,
)

AUTO_LOAD = ["sensor"]
DEPENDENCIES = ["i2c"]

CONF_ANGLE_X = "angle_x"
CONF_ANGLE_Y = "angle_y"
CONF_ANGLE_Z = "angle_z"

mpu6050_ns = cg.esphome_ns.namespace("mpu6050_tilt")
MPU6050Tilt = mpu6050_ns.class_("MPU6050Tilt", cg.PollingComponent, i2c.I2CDevice)

ANGLE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon=ICON_EMPTY,
    accuracy_decimals=1,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPU6050Tilt),
            cv.Optional(CONF_ADDRESS, default=0x68): cv.i2c_address,
            cv.Optional(CONF_ANGLE_X): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_ANGLE_Y): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_ANGLE_Z): ANGLE_SENSOR_SCHEMA,
        }
    )
    .extend(cv.polling_component_schema("50ms"))
    .extend(i2c.i2c_device_schema(0x68))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_ANGLE_X in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_X])
        cg.add(var.set_angle_x_sensor(sens))

    if CONF_ANGLE_Y in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_Y])
        cg.add(var.set_angle_y_sensor(sens))

    if CONF_ANGLE_Z in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_Z])
        cg.add(var.set_angle_z_sensor(sens))