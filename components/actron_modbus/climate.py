import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, modbus_controller
from esphome.const import (
    CONF_ID,
)

AUTO_LOAD = ["climate"]
DEPENDENCIES = ["modbus_controller"]

CONF_MODBUS_CONTROLLER_ID = "modbus_controller_id"
CONF_POWER_REGISTER = "power_register"
CONF_FAN_REGISTER = "fan_register"
CONF_MODE_REGISTER = "mode_register"
CONF_SETPOINT_REGISTER = "setpoint_register"
CONF_CONTINUOUS_FAN_REGISTER = "continuous_fan_register"
CONF_ROOM_TEMP_REGISTER = "room_temp_register"
CONF_COMMAND_INTERVAL = "command_interval"
CONF_SETTLE_TIMEOUT = "settle_timeout"
CONF_READ_TIMEOUT = "read_timeout"
CONF_OPTIMISTIC = "optimistic"
CONF_ROOM_TEMP_EVERY = "room_temp_every"

actron_modbus_ns = cg.esphome_ns.namespace("actron_modbus")
ActronModbusClimate = actron_modbus_ns.class_(
    "ActronModbusClimate", climate.Climate, cg.PollingComponent
)

CONFIG_SCHEMA = (
    climate.climate_schema(ActronModbusClimate)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(ActronModbusClimate),
            cv.GenerateID(CONF_MODBUS_CONTROLLER_ID): cv.use_id(
                modbus_controller.ModbusController
            ),
            cv.Optional(CONF_POWER_REGISTER, default=1): cv.positive_int,
            cv.Optional(CONF_FAN_REGISTER, default=4): cv.positive_int,
            cv.Optional(CONF_MODE_REGISTER, default=101): cv.positive_int,
            cv.Optional(CONF_SETPOINT_REGISTER, default=102): cv.positive_int,
            cv.Optional(CONF_CONTINUOUS_FAN_REGISTER, default=105): cv.positive_int,
            cv.Optional(CONF_ROOM_TEMP_REGISTER, default=851): cv.positive_int,
            cv.Optional(CONF_COMMAND_INTERVAL, default="200ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SETTLE_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
            # Per-step wait before skipping a hung read (hub refuse / dropped frame).
            cv.Optional(CONF_READ_TIMEOUT, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OPTIMISTIC, default=True): cv.boolean,
            # Poll room temp on every Nth climate cycle (slow-changing).
            cv.Optional(CONF_ROOM_TEMP_EVERY, default=3): cv.int_range(min=1, max=60),
        }
    )
    .extend(cv.polling_component_schema("3s"))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    parent = await cg.get_variable(config[CONF_MODBUS_CONTROLLER_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_power_register(config[CONF_POWER_REGISTER]))
    cg.add(var.set_fan_register(config[CONF_FAN_REGISTER]))
    cg.add(var.set_mode_register(config[CONF_MODE_REGISTER]))
    cg.add(var.set_setpoint_register(config[CONF_SETPOINT_REGISTER]))
    cg.add(var.set_continuous_fan_register(config[CONF_CONTINUOUS_FAN_REGISTER]))
    cg.add(var.set_room_temp_register(config[CONF_ROOM_TEMP_REGISTER]))
    cg.add(var.set_command_interval_ms(config[CONF_COMMAND_INTERVAL].total_milliseconds))
    cg.add(var.set_settle_timeout_ms(config[CONF_SETTLE_TIMEOUT].total_milliseconds))
    cg.add(var.set_read_timeout_ms(config[CONF_READ_TIMEOUT].total_milliseconds))
    cg.add(var.set_optimistic(config[CONF_OPTIMISTIC]))
    cg.add(var.set_room_temp_every(config[CONF_ROOM_TEMP_EVERY]))
