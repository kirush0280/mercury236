import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, uart
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

# Если константа UNIT_KILOWATT_HOUR отсутствует, используем строку
UNIT_KILOWATT_HOURS = "kWh"

DEPENDENCIES = ["uart"]

# Определение пространства имен и класса компонента
mercury230_ns = cg.esphome_ns.namespace("energy_meter_mercury230")
Mercury230Multi = mercury230_ns.class_("Mercury230Multi", cg.PollingComponent, uart.UARTDevice)

# Конфигурационные константы
CONF_METERS = "meters"
CONF_VOLTAGE_A = "voltage_a"
CONF_VOLTAGE_B = "voltage_b"
CONF_VOLTAGE_C = "voltage_c"
CONF_POWER_SUMM = "power_summ"
CONF_POWER_A = "power_a"
CONF_POWER_B = "power_b"
CONF_POWER_C = "power_c"
CONF_ENERGY_AA = "energy_aa"
CONF_ENERGY_RA = "energy_ra"
CONF_ENERGY_AB = "energy_ab"
CONF_ENERGY_RB = "energy_rb"
CONF_ENERGY_AC = "energy_ac"
CONF_ENERGY_RC = "energy_rc"
CONF_FREQUENCY = "frequency"
CONF_SN_STRING = "sn_string"
CONF_VERS_STRING = "vers_string"
CONF_FAB_DATE_STRING = "fab_date_string"
CONF_CONNECT_STATUS = "connect_status"

# Схема для одного счетчика
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
        cv.Optional(CONF_VOLTAGE_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLTAGE_C): sensor.sensor_schema(
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
        cv.Optional(CONF_POWER_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_C): sensor.sensor_schema(
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
        cv.Optional(CONF_ENERGY_AB): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ENERGY_RB): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ENERGY_AC): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ENERGY_RC): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
            unit_of_measurement="Hz",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SN_STRING): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_VERS_STRING): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FAB_DATE_STRING): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_CONNECT_STATUS): text_sensor.text_sensor_schema(),
    }
)

# Основная схема конфигурации
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Mercury230Multi),
        cv.Required(CONF_METERS): cv.ensure_list(METER_SCHEMA),
    }
).extend(cv.polling_component_schema("30s")).extend(uart.UART_DEVICE_SCHEMA)

# Генерация кода
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Добавление счетчиков
    for meter in config[CONF_METERS]:
        address = meter[CONF_ADDRESS]
        password = meter[CONF_PASSWORD]
        cg.add(var.add_meter(address, password))

        # Настройка сенсоров
        if CONF_VOLTAGE_A in meter:
            sens = await sensor.new_sensor(meter[CONF_VOLTAGE_A])
            cg.add(var.set_voltage_a_sensor(address, sens))

        if CONF_VOLTAGE_B in meter:
            sens = await sensor.new_sensor(meter[CONF_VOLTAGE_B])
            cg.add(var.set_voltage_b_sensor(address, sens))

        if CONF_VOLTAGE_C in meter:
            sens = await sensor.new_sensor(meter[CONF_VOLTAGE_C])
            cg.add(var.set_voltage_c_sensor(address, sens))

        if CONF_POWER_SUMM in meter:
            sens = await sensor.new_sensor(meter[CONF_POWER_SUMM])
            cg.add(var.set_power_summ_sensor(address, sens))

        if CONF_POWER_A in meter:
            sens = await sensor.new_sensor(meter[CONF_POWER_A])
            cg.add(var.set_power_a_sensor(address, sens))

        if CONF_POWER_B in meter:
            sens = await sensor.new_sensor(meter[CONF_POWER_B])
            cg.add(var.set_power_b_sensor(address, sens))

        if CONF_POWER_C in meter:
            sens = await sensor.new_sensor(meter[CONF_POWER_C])
            cg.add(var.set_power_c_sensor(address, sens))

        if CONF_ENERGY_AA in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_AA])
            cg.add(var.set_energy_aa_sensor(address, sens))

        if CONF_ENERGY_RA in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_RA])
            cg.add(var.set_energy_ra_sensor(address, sens))

        if CONF_ENERGY_AB in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_AB])
            cg.add(var.set_energy_ab_sensor(address, sens))

        if CONF_ENERGY_RB in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_RB])
            cg.add(var.set_energy_rb_sensor(address, sens))

        if CONF_ENERGY_AC in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_AC])
            cg.add(var.set_energy_ac_sensor(address, sens))

        if CONF_ENERGY_RC in meter:
            sens = await sensor.new_sensor(meter[CONF_ENERGY_RC])
            cg.add(var.set_energy_rc_sensor(address, sens))

        if CONF_FREQUENCY in meter:
            sens = await sensor.new_sensor(meter[CONF_FREQUENCY])
            cg.add(var.set_frequency_sensor(address, sens))

        if CONF_SN_STRING in meter:
            sens = await text_sensor.new_text_sensor(meter[CONF_SN_STRING])
            cg.add(var.set_sn_string_sensor(address, sens))

        if CONF_VERS_STRING in meter:
            sens = await text_sensor.new_text_sensor(meter[CONF_VERS_STRING])
            cg.add(var.set_vers_string_sensor(address, sens))

        if CONF_FAB_DATE_STRING in meter:
            sens = await text_sensor.new_text_sensor(meter[CONF_FAB_DATE_STRING])
            cg.add(var.set_fab_date_string_sensor(address, sens))

        if CONF_CONNECT_STATUS in meter:
            sens = await text_sensor.new_text_sensor(meter[CONF_CONNECT_STATUS])
            cg.add(var.set_connect_status_sensor(address, sens))
