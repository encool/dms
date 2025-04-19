import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, output 
from esphome.const import CONF_ID
# vvv--- 导入 gpio 相关的验证 ---vvv
from esphome.components.gpio import PinSchema
DEPENDENCIES = ['uart', 'output']

empty_uart_component_ns = cg.esphome_ns.namespace('bambu_bus')
EmptyUARTComponent = empty_uart_component_ns.class_('BambuBus', cg.Component, uart.UARTDevice)

CONF_DE_PIN = 'de_pin'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(EmptyUARTComponent),
    # <<<--- 添加 DE 引脚配置选项 (可选)
    # vvv--- 修改 DE 引脚的配置方式 ---vvv
    cv.Optional(CONF_DE_PIN): PinSchema({ # 使用 PinSchema 来定义引脚
        cv.Required(CONF_ID): cv.declare_id(cg.GPIOPin), # 声明一个 GPIOPin ID
        # 你可以在这里添加其他引脚选项，比如反转模式 (inverted) 等，如果需要的话
        # cv.Optional(CONF_MODE, default="OUTPUT"): cv.enum(GPIO_MODES, upper=True), # 强制为输出
    }).extend({cv.Required("mode"): "OUTPUT"}), # 确保模式是 OUTPUT
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

        # <<<--- 如果配置了 DE 引脚，生成设置代码 ---
    if CONF_DE_PIN in config:
        # vvv--- 获取 GPIOPin* 对象 ---vvv
        # 如果使用 PinSchema:
        conf = config[CONF_DE_PIN]
        # 这里可能需要手动创建 GPIOPin 对象，或者 PinSchema 内部处理了？
        # 让我们尝试直接从配置生成表达式
        pin_expression = yield cg.gpio_pin_expression(conf)
        cg.add(var.set_de_pin(pin_expression))