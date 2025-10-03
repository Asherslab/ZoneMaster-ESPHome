import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
)
from esphome.components import uart, binary_sensor

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor"]

zonemaster_proto_ns = cg.esphome_ns.namespace("zonemaster_proto")
ZonemasterProto = zonemaster_proto_ns.class_("ZonemasterProto", cg.Component, uart.UARTDevice)

CONF_DEVICE_ID = "device_id"
CONF_POLL_INTERVAL = "poll_interval"

# Button keys (bit 0..5 => button_1..button_6)
CONF_BUTTON_1 = "button_1"
CONF_BUTTON_2 = "button_2"
CONF_BUTTON_3 = "button_3"
CONF_BUTTON_4 = "button_4"
CONF_BUTTON_5 = "button_5"
CONF_BUTTON_6 = "button_6"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZonemasterProto),
            cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_DEVICE_ID, default=0xB1): cv.uint8_t,
            cv.Optional(CONF_POLL_INTERVAL, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_BUTTON_1): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_BUTTON_2): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_BUTTON_3): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_BUTTON_4): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_BUTTON_5): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_BUTTON_6): binary_sensor.binary_sensor_schema(),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_device_id(config[CONF_DEVICE_ID]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))

    # Attach binary sensors if configured
    for idx, key in enumerate(
        [CONF_BUTTON_1, CONF_BUTTON_2, CONF_BUTTON_3, CONF_BUTTON_4, CONF_BUTTON_5, CONF_BUTTON_6],
        start=1,
    ):
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, f"set_button{idx}")(sens))
