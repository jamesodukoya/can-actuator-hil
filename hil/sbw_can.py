"""CAN helpers shared by the SIL test suite."""
from pathlib import Path

import cantools

REPO_ROOT = Path(__file__).resolve().parents[1]
PRODUCT_DBC = REPO_ROOT / "dbc" / "steer_by_wire.dbc"
SIL_DBC = REPO_ROOT / "dbc" / "sil_only.dbc"

HEARTBEAT_ID = 0x100
COMMAND_ID = 0x110
STATUS_ID = 0x120
DIAG_ID = 0x1F0
BUILD_ID = 0x1F1
SIM_INJECT_ID = 0x700
SIM_TRUTH_ID = 0x710
SIM_STEP_ID = 0x720
SIM_STEP_DONE_ID = 0x730

MODE_OFF, MODE_MOTOR_ONLY, MODE_KINGPIN_ONLY, MODE_DUAL_LOOP, MODE_CLEAR_FAULT = range(5)

CRC_BYTE_INDEX = 7
ALIVE_COUNTER_MODULUS = 16


def crc8_sae_j1850(data: bytes) -> int:
    """CRC-8/SAE-J1850: poly 0x1D, init 0xFF, no reflection, xorout 0xFF (check value 0x4B)."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


def load_database() -> cantools.database.Database:
    database = cantools.database.load_file(PRODUCT_DBC)
    database.add_dbc_file(SIL_DBC)
    return database


def encode_command(database: cantools.database.Database, target_mrad: int, current_limit_ma: int,
                   mode: int, counter: int, corrupt_crc: bool = False) -> bytes:
    message = database.get_message_by_name("SBW_Command")
    fields = {"TargetKingpinAngle": target_mrad, "CurrentLimit": current_limit_ma,
              "ControlMode": mode, "AliveCounter": counter % ALIVE_COUNTER_MODULUS, "Crc8": 0}
    # Raw integers: the DBC's range limits are physical and would reject raw 100 against 0.524.
    data = bytearray(message.encode(fields, scaling=False, strict=False))
    crc = crc8_sae_j1850(bytes(data[:CRC_BYTE_INDEX]))
    # Inverting every bit always gives a CRC that differs from the correct one.
    data[CRC_BYTE_INDEX] = crc ^ 0xFF if corrupt_crc else crc
    return bytes(data)
