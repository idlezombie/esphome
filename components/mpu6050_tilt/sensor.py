import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    UNIT_DEGREES,
    UNIT_PERCENT,
    ICON_EMPTY,
)

AUTO_LOAD = ["sensor"]
DEPENDENCIES = ["i2c"]

# Sensor outputs
CONF_ANGLE_X = "angle_x"
CONF_ANGLE_Y = "angle_y"
CONF_ANGLE_Z = "angle_z"
CONF_POSITION = "position"
# Diagnostic: raw (calibrated) accel and gyro
CONF_RAW_ACCEL_X = "raw_accel_x"
CONF_RAW_ACCEL_Y = "raw_accel_y"
CONF_RAW_ACCEL_Z = "raw_accel_z"
CONF_RAW_GYRO_X = "raw_gyro_x"
CONF_RAW_GYRO_Y = "raw_gyro_y"
CONF_RAW_GYRO_Z = "raw_gyro_z"

# Configuration options
CONF_AXIS = "axis"  # which accel angle formula: x = atan2(ay,az), y = atan2(-ax,...)
CONF_TILT_GYRO_AXIS = "tilt_gyro_axis"  # which gyro axis for tilt/position: x, y, z
CONF_ACCEL_RANGE = "accel_range"
CONF_GYRO_RANGE = "gyro_range"
CONF_DLPF = "dlpf"
CONF_CLOSED_ANGLE = "closed_angle"
CONF_OPEN_ANGLE = "open_angle"
CONF_SMOOTHING_FACTOR = "smoothing_factor"
CONF_STATIONARY_THRESHOLD = "stationary_threshold"

mpu6050_ns = cg.esphome_ns.namespace("mpu6050_tilt")
MPU6050Tilt = mpu6050_ns.class_("MPU6050Tilt", cg.PollingComponent, i2c.I2CDevice)

ANGLE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon=ICON_EMPTY,
    accuracy_decimals=1,
)

POSITION_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    icon=ICON_EMPTY,
    accuracy_decimals=0,
)

# Diagnostic sensors: calibrated accel in g, gyro in °/s (show in device diagnostics only)
RAW_ACCEL_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="g",
    icon=ICON_EMPTY,
    accuracy_decimals=3,
    entity_category="diagnostic",
)
RAW_GYRO_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="°/s",
    icon=ICON_EMPTY,
    accuracy_decimals=2,
    entity_category="diagnostic",
)


def _axis_to_index(axis: str) -> int:
    # 0 -> X (angle from ay,az), 1 -> Y (angle from ax,ay,az)
    return 0 if axis.lower() == "x" else 1


def _gyro_axis_to_index(axis: str) -> int:
    # 0 -> X, 1 -> Y, 2 -> Z
    return {"x": 0, "y": 1, "z": 2}[axis.lower()]


def _accel_range_to_fs_sel(val: str) -> int:
    # Mapping to ACCEL_CONFIG AFS_SEL bits (0..3)
    mapping = {
        "2g": 0,
        "4g": 1,
        "8g": 2,
        "16g": 3,
    }
    return mapping[val]


def _gyro_range_to_fs_sel(val: str) -> int:
    # Mapping to GYRO_CONFIG FS_SEL bits (0..3)
    mapping = {
        "250dps": 0,
        "500dps": 1,
        "1000dps": 2,
        "2000dps": 3,
    }
    return mapping[val]


def _dlpf_to_cfg(val: int) -> int:
    # DLPF bandwidth codes (see MPU6050 datasheet)
    # 0: 260Hz, 1: 184Hz, 2: 94Hz, 3: 44Hz, 4: 21Hz, 5: 10Hz, 6: 5Hz
    mapping = {
        5: 6,
        10: 5,
        20: 4,
        42: 3,
        94: 2,
        184: 1,
        260: 0,
    }
    return mapping[val]


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPU6050Tilt),
            cv.Optional(CONF_ANGLE_X): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_ANGLE_Y): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_ANGLE_Z): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_POSITION): POSITION_SENSOR_SCHEMA,
            cv.Optional(CONF_AXIS, default="x"): cv.one_of("x", "y", lower=True),
            cv.Optional(CONF_TILT_GYRO_AXIS, default="y"): cv.one_of(
                "x", "y", "z", lower=True
            ),
            cv.Optional(CONF_ACCEL_RANGE, default="4g"): cv.one_of(
                "2g", "4g", "8g", "16g", lower=True
            ),
            cv.Optional(CONF_GYRO_RANGE, default="500dps"): cv.one_of(
                "250dps", "500dps", "1000dps", "2000dps", lower=True
            ),
            cv.Optional(CONF_DLPF, default=20): cv.one_of(
                5, 10, 20, 42, 94, 184, 260, int=True
            ),
            cv.Optional(CONF_CLOSED_ANGLE, default=0.0): cv.float_,
            cv.Optional(CONF_OPEN_ANGLE, default=90.0): cv.float_,
            cv.Optional(CONF_SMOOTHING_FACTOR, default=0.9): cv.float_range(
                min=0.0, max=1.0
            ),
            cv.Optional(CONF_STATIONARY_THRESHOLD, default=0.1): cv.float_,
            # Optional diagnostic sensors (raw calibrated accel/gyro)
            cv.Optional(CONF_RAW_ACCEL_X): RAW_ACCEL_SENSOR_SCHEMA,
            cv.Optional(CONF_RAW_ACCEL_Y): RAW_ACCEL_SENSOR_SCHEMA,
            cv.Optional(CONF_RAW_ACCEL_Z): RAW_ACCEL_SENSOR_SCHEMA,
            cv.Optional(CONF_RAW_GYRO_X): RAW_GYRO_SENSOR_SCHEMA,
            cv.Optional(CONF_RAW_GYRO_Y): RAW_GYRO_SENSOR_SCHEMA,
            cv.Optional(CONF_RAW_GYRO_Z): RAW_GYRO_SENSOR_SCHEMA,
        }
    )
    .extend(cv.polling_component_schema("50ms"))
    .extend(i2c.i2c_device_schema(0x68))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await i2c.register_i2c_device(var, config)
    
    # Register as component FIRST, before setting up sensors
    await cg.register_component(var, config)

    # Angles
    if CONF_ANGLE_X in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_X])
        cg.add(var.set_angle_x_sensor(sens))

    if CONF_ANGLE_Y in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_Y])
        cg.add(var.set_angle_y_sensor(sens))

    if CONF_ANGLE_Z in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE_Z])
        cg.add(var.set_angle_z_sensor(sens))

    # Position sensor
    if CONF_POSITION in config:
        sens = await sensor.new_sensor(config[CONF_POSITION])
        cg.add(var.set_position_sensor(sens))

    # Diagnostic: raw accel and gyro
    if CONF_RAW_ACCEL_X in config:
        sens = await sensor.new_sensor(config[CONF_RAW_ACCEL_X])
        cg.add(var.set_raw_accel_x_sensor(sens))
    if CONF_RAW_ACCEL_Y in config:
        sens = await sensor.new_sensor(config[CONF_RAW_ACCEL_Y])
        cg.add(var.set_raw_accel_y_sensor(sens))
    if CONF_RAW_ACCEL_Z in config:
        sens = await sensor.new_sensor(config[CONF_RAW_ACCEL_Z])
        cg.add(var.set_raw_accel_z_sensor(sens))
    if CONF_RAW_GYRO_X in config:
        sens = await sensor.new_sensor(config[CONF_RAW_GYRO_X])
        cg.add(var.set_raw_gyro_x_sensor(sens))
    if CONF_RAW_GYRO_Y in config:
        sens = await sensor.new_sensor(config[CONF_RAW_GYRO_Y])
        cg.add(var.set_raw_gyro_y_sensor(sens))
    if CONF_RAW_GYRO_Z in config:
        sens = await sensor.new_sensor(config[CONF_RAW_GYRO_Z])
        cg.add(var.set_raw_gyro_z_sensor(sens))

    # Tilt/position axis: which accel angle formula and which gyro axis
    axis_idx = _axis_to_index(config[CONF_AXIS])
    cg.add(var.set_axis_index(axis_idx))
    gyro_axis_idx = _gyro_axis_to_index(config[CONF_TILT_GYRO_AXIS])
    cg.add(var.set_tilt_gyro_axis(gyro_axis_idx))

    # Hardware configuration
    accel_fs = _accel_range_to_fs_sel(config[CONF_ACCEL_RANGE])
    gyro_fs = _gyro_range_to_fs_sel(config[CONF_GYRO_RANGE])
    dlpf_cfg = _dlpf_to_cfg(config[CONF_DLPF])

    cg.add(var.set_accel_fs_sel(accel_fs))
    cg.add(var.set_gyro_fs_sel(gyro_fs))
    cg.add(var.set_dlpf_cfg(dlpf_cfg))

    # Position mapping configuration
    cg.add(var.set_closed_angle(config[CONF_CLOSED_ANGLE]))
    cg.add(var.set_open_angle(config[CONF_OPEN_ANGLE]))

    # Noise reduction configuration
    cg.add(var.set_smoothing_factor(config[CONF_SMOOTHING_FACTOR]))
    cg.add(var.set_stationary_threshold(config[CONF_STATIONARY_THRESHOLD]))
