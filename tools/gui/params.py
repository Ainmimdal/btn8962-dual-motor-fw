"""Mirror of the firmware parameter table (src/control/params.c).

The numeric IDs are the CAN protocol's parameter IDs. Over UART the GUI
reads names, ranges and defaults from the board itself ("CFG") and warns
if they don't match this table.
"""

# id, name, group, unit, min, max, description, needs (None / "cs" / "nocs")
PARAMS = [
    (0, "M1_VEL_KP", "Motor 1 PID", "duty/(cnt/s)", 0, 100, "Velocity loop P gain", None),
    (1, "M1_VEL_KI", "Motor 1 PID", "duty/cnt", 0, 1000, "Velocity loop I gain", None),
    (2, "M1_VEL_KD", "Motor 1 PID", "duty/(cnt/s^2)", 0, 100, "Velocity loop D gain", None),
    (3, "M1_POS_KP", "Motor 1 PID", "(cnt/s)/cnt", 0, 10000, "Position loop P gain", None),
    (4, "M1_POS_KI", "Motor 1 PID", "(cnt/s)/(cnt*s)", 0, 10000, "Position loop I gain", None),
    (5, "M1_POS_KD", "Motor 1 PID", "", 0, 1000, "Position loop D gain", None),
    (6, "M2_VEL_KP", "Motor 2 PID", "duty/(cnt/s)", 0, 100, "Velocity loop P gain", None),
    (7, "M2_VEL_KI", "Motor 2 PID", "duty/cnt", 0, 1000, "Velocity loop I gain", None),
    (8, "M2_VEL_KD", "Motor 2 PID", "duty/(cnt/s^2)", 0, 100, "Velocity loop D gain", None),
    (9, "M2_POS_KP", "Motor 2 PID", "(cnt/s)/cnt", 0, 10000, "Position loop P gain", None),
    (10, "M2_POS_KI", "Motor 2 PID", "(cnt/s)/(cnt*s)", 0, 10000, "Position loop I gain", None),
    (11, "M2_POS_KD", "Motor 2 PID", "", 0, 1000, "Position loop D gain", None),
    (12, "DUTY_MAX", "Limits", "0..1", 0, 1, "Duty cap", None),
    (13, "DUTY_RAMP", "Limits", "1/s", 0.1, 1000, "Duty slew rate (full scale per second)", None),
    (14, "VEL_MAX", "Limits", "cnt/s", 1, 1e6, "Velocity set-point limit in position mode", None),
    (15, "I_LIMIT", "Current", "A", 0.1, 19, "Soft current limit", "cs"),
    (16, "I_TRIP", "Current", "A", 0.1, 19, "Overcurrent fault level", "cs"),
    (17, "I_TRIP_MS", "Current", "ms", 1, 1000, "Time above I_TRIP before fault", "cs"),
    (18, "STALL_MS", "Stall", "ms", 10, 60000, "High effort without movement for this long = stall", None),
    (19, "STALL_COUNTS", "Stall", "cnt", 1, 100000, "Movement below this counts as not moving", None),
    (20, "STALL_CURRENT", "Stall", "A", 0.1, 19, "Effort threshold (current sense builds)", "cs"),
    (21, "STALL_DUTY", "Stall", "0..1", 0.01, 1, "Effort threshold (no current sense builds)", "nocs"),
    (22, "WDG_MS", "Comms", "ms", 0, 60000, "Command watchdog (0 = off)", None),
    (23, "TELEM_MS", "Comms", "ms", 0, 60000, "UART telemetry period (0 = off)", None),
    (24, "CAN_TELEM_MS", "CAN", "ms", 0, 60000, "Vendor status frame period, cmd 0x1F (0 = off)", None),
    (25, "M1_INVERT", "Direction", "0/1", 0, 1, "Invert motor 1 drive (motors OFF)", None),
    (26, "M2_INVERT", "Direction", "0/1", 0, 1, "Invert motor 2 drive (motors OFF)", None),
    (27, "ENC1_INVERT", "Direction", "0/1", 0, 1, "Invert encoder 1 count (motors OFF)", None),
    (28, "ENC2_INVERT", "Direction", "0/1", 0, 1, "Invert encoder 2 count (motors OFF)", None),
    (29, "M1_NODE_ID", "CAN", "", 0, 62, "Motor 1 ODrive-style node ID (not used by any ODrive)", None),
    (30, "M2_NODE_ID", "CAN", "", 0, 62, "Motor 2 ODrive-style node ID", None),
    (31, "CAN_BAUD", "CAN", "kbit/s", 125, 1000, "125/250/500/1000, applied after save + reset", None),
    (32, "CAN_HB_MS", "CAN", "ms", 0, 60000, "Heartbeat period, cmd 0x01 (0 = off)", None),
    (33, "CAN_ENC_MS", "CAN", "ms", 0, 60000, "Encoder estimates period, cmd 0x09 (0 = off)", None),
    (34, "CAN_ODRV_VER", "CAN", "51/56", 51, 56, "ODrive heartbeat/error layout: 51 = 0.5.1, 56 = 0.5.6", None),
    (35, "ENC1_CPR", "Encoder", "cnt/turn", 1, 1e6, "Encoder 1 counts per turn of its shaft (4 x PPR)", None),
    (36, "ENC2_CPR", "Encoder", "cnt/turn", 1, 1e6, "Encoder 2 counts per turn of its shaft (4 x PPR)", None),
    (37, "M1_GEAR", "Encoder", "ratio", 0.001, 10000, "Motor 1 gear ratio, motor turns per output turn", None),
    (38, "M2_GEAR", "Encoder", "ratio", 0.001, 10000, "Motor 2 gear ratio, motor turns per output turn", None),
    (39, "ENC1_ON_OUTPUT", "Encoder", "0/1", 0, 1, "Encoder 1 is on the output shaft (gear ratio not applied)", None),
    (40, "ENC2_ON_OUTPUT", "Encoder", "0/1", 0, 1, "Encoder 2 is on the output shaft (gear ratio not applied)", None),
]

UNIT_PARAMS = {35, 36, 37, 38, 39, 40}


def counts_per_turn(params, m):
    """Encoder counts per output-shaft turn for motor m (0/1), from a
    {param id: value} dict; mirrors counts_per_output_turn() in firmware."""
    cpr = params.get(35 + m, 1.0)
    if params.get(39 + m, 0.0) > 0.5:
        return cpr
    return cpr * params.get(37 + m, 1.0)

BY_ID = {p[0]: p for p in PARAMS}
BY_NAME = {p[1]: p for p in PARAMS}

MODES = ["OFF", "DUTY", "VEL", "POS"]

FAULTS = [(0x01, "OC"), (0x02, "STALL"), (0x04, "LIMIT"), (0x08, "DRIVER"), (0x10, "ESTOP")]


def fault_text(bits):
    names = [n for b, n in FAULTS if bits & b]
    return ",".join(names) if names else "none"
