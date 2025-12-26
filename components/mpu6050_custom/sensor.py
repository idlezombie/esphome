import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import CONF_ID

mpu6050_ns = cg.esphome_ns.namespace("mpu6050_custom")
MPU6050Custom = mpu6050_ns.class_("MPU6050Custom", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPU6050Custom),
        }
    )
    .extend(i2c.i2c_device_schema(0x68))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)