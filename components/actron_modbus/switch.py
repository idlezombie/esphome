import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import modbus_controller, switch
from esphome.const import CONF_ADDRESS, CONF_ID

DEPENDENCIES = ["modbus_controller"]
AUTO_LOAD = ["switch"]

CONF_MODBUS_CONTROLLER_ID = "modbus_controller_id"
CONF_BITMASK = "bitmask"
CONF_SETTLE_TIMEOUT = "settle_timeout"
CONF_OPTIMISTIC = "optimistic"

actron_modbus_ns = cg.esphome_ns.namespace("actron_modbus")
ActronModbusSwitch = actron_modbus_ns.class_(
    "ActronModbusSwitch", switch.Switch, cg.PollingComponent
)

CONFIG_SCHEMA = (
    switch.switch_schema(ActronModbusSwitch)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(ActronModbusSwitch),
            cv.GenerateID(CONF_MODBUS_CONTROLLER_ID): cv.use_id(
                modbus_controller.ModbusController
            ),
            cv.Required(CONF_ADDRESS): cv.positive_int,
            cv.Optional(CONF_BITMASK, default=0x0001): cv.hex_uint16_t,
            cv.Optional(CONF_SETTLE_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OPTIMISTIC, default=True): cv.boolean,
        }
    )
    .extend(cv.polling_component_schema("15s"))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await switch.register_switch(var, config)

    parent = await cg.get_variable(config[CONF_MODBUS_CONTROLLER_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_bitmask(config[CONF_BITMASK]))
    cg.add(var.set_settle_timeout_ms(config[CONF_SETTLE_TIMEOUT].total_milliseconds))
    cg.add(var.set_optimistic(config[CONF_OPTIMISTIC]))
