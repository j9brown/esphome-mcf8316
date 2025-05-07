from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TRIGGER_ID

CODEOWNERS = ["@j9brown"]

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_WAKE_PIN = "wake"
CONF_NFAULT_PIN = "nfault"
CONF_WATCHDOG = "watchdog"
CONF_ON_FAULT = "on_fault"

mcf8316_ns = cg.esphome_ns.namespace("mcf8316")

MCF8316Component = mcf8316_ns.class_("MCF8316Component", cg.Component)

FaultStatus = MCF8316Component.class_("FaultStatus")
FaultTrigger = mcf8316_ns.class_(
    "FaultTrigger", automation.Trigger.template(FaultStatus)
)

CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(MCF8316Component),
            cv.Required(CONF_WAKE_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_NFAULT_PIN): pins.gpio_input_pullup_pin_schema,
            cv.Optional(CONF_WATCHDOG, False): bool,
            cv.Optional(CONF_ON_FAULT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(FaultTrigger),
                }
            ),
        }
    ).extend(i2c.i2c_device_schema(0x01))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_wake_pin(await cg.gpio_pin_expression(config[CONF_WAKE_PIN])))
    cg.add(var.set_nfault_pin(await cg.gpio_pin_expression(config[CONF_NFAULT_PIN])))
    cg.add(var.set_watchdog(config[CONF_WATCHDOG]))
    for conf in config.get(CONF_ON_FAULT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(FaultStatus, "x")], conf)
