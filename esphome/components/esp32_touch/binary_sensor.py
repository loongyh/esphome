import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (CONF_NAME, CONF_PIN, CONF_THRESHOLD, CONF_ADAPTIVE_THRESHOLD,
                           CONF_FILTER_PERIOD, CONF_FACTOR, ESP_PLATFORM_ESP32, CONF_ID)
from esphome.pins import validate_gpio_pin
from . import esp32_touch_ns, ESP32TouchComponent

ESP_PLATFORMS = [ESP_PLATFORM_ESP32]
DEPENDENCIES = ['esp32_touch']

CONF_ESP32_TOUCH_ID = 'esp32_touch_id'

TOUCH_PADS = {
    4: cg.global_ns.TOUCH_PAD_NUM0,
    0: cg.global_ns.TOUCH_PAD_NUM1,
    2: cg.global_ns.TOUCH_PAD_NUM2,
    15: cg.global_ns.TOUCH_PAD_NUM3,
    13: cg.global_ns.TOUCH_PAD_NUM4,
    12: cg.global_ns.TOUCH_PAD_NUM5,
    14: cg.global_ns.TOUCH_PAD_NUM6,
    27: cg.global_ns.TOUCH_PAD_NUM7,
    33: cg.global_ns.TOUCH_PAD_NUM8,
    32: cg.global_ns.TOUCH_PAD_NUM9,
}


def validate_touch_pad(value):
    value = validate_gpio_pin(value)
    if value not in TOUCH_PADS:
        raise cv.Invalid("Pin {} does not support touch pads.".format(value))
    return value


ESP32TouchBinarySensor = esp32_touch_ns.class_('ESP32TouchBinarySensor', binary_sensor.BinarySensor)

ADAPTIVE_THRESHOLD_SCHEMA = cv.Schema({
    cv.Required(CONF_FILTER_PERIOD): cv.positive_time_period,
    cv.Required(CONF_FACTOR): cv.positive_float
})

CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ESP32TouchBinarySensor),
    cv.GenerateID(CONF_ESP32_TOUCH_ID): cv.use_id(ESP32TouchComponent),
    cv.Required(CONF_PIN): validate_touch_pad,
    cv.Exclusive(CONF_THRESHOLD, 'threshold_group'): cv.uint16_t,
    cv.Exclusive(CONF_ADAPTIVE_THRESHOLD, 'threshold_group'): ADAPTIVE_THRESHOLD_SCHEMA
})


def to_code(config):
    hub = yield cg.get_variable(config[CONF_ESP32_TOUCH_ID])
    if CONF_ADAPTIVE_THRESHOLD in config:
        conf = config[CONF_ADAPTIVE_THRESHOLD]
        var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME], TOUCH_PADS[config[CONF_PIN]],
                               True, conf.get(CONF_FILTER_PERIOD), conf.get(CONF_FACTOR))
    else:
        var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME], TOUCH_PADS[config[CONF_PIN]],
                               config[CONF_THRESHOLD])
    yield binary_sensor.register_binary_sensor(var, config)
    cg.add(hub.register_touch_pad(var))
