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

CONF_RESPONSE_WINDOW = "response_window"
CONF_ACCEPT_ANY_RESPONSE = "accept_any_response"

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
            
            # If True, publish every valid response (CRC OK) regardless of a prior request.
            # If False (default), require response ID to match the last seen request ID within the window.
            cv.Optional(CONF_ACCEPT_ANY_RESPONSE, default=False): cv.boolean,

            # Max time between the last seen request and its matching response (default 150ms)
            cv.Optional(CONF_RESPONSE_WINDOW, default="150ms"): cv.positive_time_period_milliseconds,
            
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

    cg.add(var.set_accept_any_response(config[CONF_ACCEPT_ANY_RESPONSE]))
    cg.add(var.set_response_window(config[CONF_RESPONSE_WINDOW]))

    # Attach binary sensors if configured
    for idx, key in enumerate(
        [CONF_BUTTON_1, CONF_BUTTON_2, CONF_BUTTON_3, CONF_BUTTON_4, CONF_BUTTON_5, CONF_BUTTON_6],
        start=1,
    ):
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, f"set_button{idx}")(sens))
