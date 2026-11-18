"""The DBC, the C++ packing code and the Python CRC must describe the same frames.

sbw_pack_vectors prints the frames the C++ code produces. Each test compares them with what
cantools derives from dbc/steer_by_wire.dbc, in both directions, so a field that the DBC forgot
fails as surely as one it got wrong.
"""
import json
import os
import subprocess
from pathlib import Path

import pytest

from sbw_can import (COMMAND_ID, CRC_BYTE_INDEX, DIAG_ID, HEARTBEAT_ID, REPO_ROOT, STATUS_ID,
                     crc8_sae_j1850)

DEFAULT_PACK_VECTORS = REPO_ROOT / "build" / "tools" / "sbw_pack_vectors"
CRC_CHECK_INPUT = b"123456789"
CRC_CHECK_VALUE = 0x4B
FRAME_LENGTH = 8


def read_cpp_vectors() -> list[dict]:
    path = Path(os.environ.get("SBW_PACK_VECTORS", DEFAULT_PACK_VECTORS))
    if not path.exists():
        pytest.fail(f"{path} not found: run cmake --build build")
    result = subprocess.run([str(path)], check=True, capture_output=True, text=True)
    return [json.loads(line) for line in result.stdout.splitlines() if line.strip()]


def first_vector(vectors: list[dict], message_name: str) -> dict:
    return next(vector for vector in vectors if vector["message"] == message_name)


def start_and_length(message, signal_name: str) -> tuple[int, int]:
    signal = message.get_signal_by_name(signal_name)
    return signal.start, signal.length


@pytest.fixture(scope="module")
def cpp_vectors() -> list[dict]:
    return read_cpp_vectors()


@pytest.mark.requirements("REQ-CAN-003")
def test_cpp_frames_decode_with_dbc(database, cpp_vectors):
    for vector in cpp_vectors:
        message = database.get_message_by_name(vector["message"])
        decoded = message.decode(bytes.fromhex(vector["hex"]), scaling=False, decode_choices=False)
        assert decoded == vector["fields"], vector["message"]


@pytest.mark.requirements("REQ-CAN-003")
def test_dbc_encodes_same_bytes_as_cpp(database, cpp_vectors):
    for vector in cpp_vectors:
        message = database.get_message_by_name(vector["message"])
        encoded = message.encode(vector["fields"], scaling=False, strict=False)
        assert encoded.hex() == vector["hex"], vector["message"]


@pytest.mark.requirements("REQ-CAN-001")
def test_python_crc_matches_cpp_crc(cpp_vectors):
    assert crc8_sae_j1850(CRC_CHECK_INPUT) == CRC_CHECK_VALUE
    for vector in cpp_vectors:
        if vector["message"] != "SBW_Command":
            continue
        data = bytes.fromhex(vector["hex"])
        assert crc8_sae_j1850(data[:CRC_BYTE_INDEX]) == data[CRC_BYTE_INDEX]


@pytest.mark.requirements("REQ-CAN-003")
def test_dbc_scaling_gives_si_units(database, cpp_vectors):
    command = first_vector(cpp_vectors, "SBW_Command")
    command_message = database.get_message_by_name("SBW_Command")
    command_si = command_message.decode(bytes.fromhex(command["hex"]))
    assert command_si["TargetKingpinAngle"] == pytest.approx(0.100)
    assert command_si["CurrentLimit"] == pytest.approx(1.5)

    status = first_vector(cpp_vectors, "SBW_Status")
    status_message = database.get_message_by_name("SBW_Status")
    status_si = status_message.decode(bytes.fromhex(status["hex"]))
    assert status_si["MotorSpeed"] == pytest.approx(1005.5)
    assert status_si["KingpinAngle"] == pytest.approx(-0.001)


@pytest.mark.requirements("REQ-CAN-002", "REQ-CAN-003")
def test_dbc_frame_ids_lengths_and_key_signal_positions(database):
    frame_ids = {"SBW_Heartbeat": HEARTBEAT_ID, "SBW_Command": COMMAND_ID,
                 "SBW_Status": STATUS_ID, "SBW_Diag": DIAG_ID}
    for name, frame_id in frame_ids.items():
        message = database.get_message_by_name(name)
        assert message.frame_id == frame_id, name
        assert message.length == FRAME_LENGTH, name

    command = database.get_message_by_name("SBW_Command")
    assert start_and_length(command, "AliveCounter") == (40, 4)
    assert start_and_length(command, "Crc8") == (56, 8)

    status = database.get_message_by_name("SBW_Status")
    assert start_and_length(status, "ActiveMode") == (56, 2)
    assert start_and_length(status, "FaultJam") == (58, 1)
    assert start_and_length(status, "FaultInternal") == (63, 1)
