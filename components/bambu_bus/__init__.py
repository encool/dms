import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, output 
from esphome.const import CONF_ID

DEPENDENCIES = ['uart', 'output']

empty_uart_component_ns = cg.esphome_ns.namespace('bambu_bus')
EmptyUARTComponent = empty_uart_component_ns.class_('BambuBus', cg.Component, uart.UARTDevice)

CONF_DE_PIN_ID = 'de_pin_id' # <<<--- 定义新的配置键

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(EmptyUARTComponent),
    # <<<--- 添加 DE 引脚配置选项 (可选)
    cv.Optional(CONF_DE_PIN_ID): cv.use_id(output.GPIOPin)
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

        # <<<--- 如果配置了 DE 引脚，生成设置代码 ---
    if CONF_DE_PIN_ID in config:
        de_pin = yield cg.get_variable(config[CONF_DE_PIN_ID])
        cg.add(var.set_de_pin(de_pin))