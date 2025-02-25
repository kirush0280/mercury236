import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_PASSWORD,
    CONF_VOLTAGE,
    CONF_POWER,
    CONF_ENERGY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_ENERGY,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_WATT,
)

# Если константы отсутствуют, используем их строковые эквиваленты
UNIT_KILOWATT_HOURS = getattr(cv, "UNIT_KILOWATT_HOURS", "kWh")
CONF_REACTIVE_ENERGY = getattr(cv, "CONF_REACTIVE_ENERGY", "reactive_energy")

DEPENDENCIES = ["uart"]

mercury230_ns = cg.esphome_ns.namespace("energy_meter_mercury230")
Mercury230Component = mercury230_ns.class_(
    "Mercury230Component", cg.PollingComponent, uart.UARTDevice
)

CONF_METERS = "meters"
CONF_VOLTAGE_A = "voltage_a"
CONF_POWER_SUMM = "power_summ"
CONF_ENERGY_AA = "energy_aa"
CONF_ENERGY_RA = "energy_ra"

METER_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ADDRESS): cv.uint8_t,
        cv.Required(CONF_PASSWORD): cv.string,
        cv.Optional(CONF_VOLTAGE_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_SUMM): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ENERGY_AA): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ENERGY_RA): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Mercury230Component),
        cv.Required(CONF_METERS): cv.ensure_list(METER_SCHEMA),
    }
).extend(cv.polling_component_schema("30s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for meter in config[CONF_METERS]:
        address = meter[CONF_ADDRESS]
        password = meter[CONF_PASSWORD]
        cg.add(var.add_meter(address, password))

        if CONF_VOLTAGE_A in meter:
            sens = await sensor.new_sensor(meter[CONF_VOLTAGE_A])
            cg.add(var.set_voltage_a_sensor(address, sens))

        if CONF_POWER_SUMM in meter:
            sens = await sensor.new_sensor(meter[CONF_POWER_SUMM])
            cg.add(var.set_power_summ_sensor(address, sens))

        if CONF_ENERGY_AA in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_AA])
            cg.add(var.set_energy_aa_sensor(address, sens))

        if CONF_ENERGY_RA in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_RA])
            cg.add(var.set_energy_ra_sensor(address, sens))
