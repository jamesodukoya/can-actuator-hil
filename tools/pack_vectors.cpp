// Prints the frames that the C++ packing code produces for a fixed list of messages, one JSON
// object per line. A test encodes the same raw field values from the DBC and compares the bytes,
// so the DBC and the C++ code cannot drift apart.
#include <cstddef>
#include <cstdio>
#include <initializer_list>
#include <string>

#include "sbw/can_protocol.hpp"

namespace {

std::string jsonField(const char* fieldName, long long rawValue) {
    return "\"" + std::string(fieldName) + "\": " + std::to_string(rawValue);
}

std::string joinFields(std::initializer_list<std::string> fields) {
    std::string joined;
    for (const std::string& field : fields) {
        if (!joined.empty()) {
            joined += ", ";
        }
        joined += field;
    }
    return joined;
}

void printVector(const std::string& messageName, const std::string& fieldsJson,
                 const sbw::CanFrame& frame) {
    std::printf("{\"message\": \"%s\", \"fields\": {%s}, \"hex\": \"", messageName.c_str(),
                fieldsJson.c_str());
    for (std::size_t index = 0; index < frame.length; ++index) {
        std::printf("%02x", frame.data[index]);
    }
    std::printf("\"}\n");
}

void printCommandVector(const sbw::CommandMessage& message) {
    const sbw::CanFrame frame = sbw::packCommand(message);
    constexpr std::size_t kCrcByteIndex = 7;
    const std::string fieldsJson = joinFields({
        jsonField("TargetKingpinAngle", message.targetKingpinAngleMrad),
        jsonField("CurrentLimit", message.currentLimitMilliAmp),
        jsonField("ControlMode", message.controlMode),
        jsonField("AliveCounter", message.aliveCounter),
        jsonField("Crc8", frame.data[kCrcByteIndex]),
    });
    printVector("SBW_Command", fieldsJson, frame);
}

// The DBC splits byte 7 into ActiveMode (bits 0-1) and one signal per fault bit.
long long bitOf(std::uint8_t byte, unsigned bitIndex) {
    return (static_cast<unsigned>(byte) >> bitIndex) & 1u;
}

void printStatusVector(const sbw::StatusMessage& message) {
    const sbw::CanFrame frame = sbw::packStatus(message);
    const std::uint8_t state = message.stateAndFaults;
    const std::string fieldsJson = joinFields({
        jsonField("KingpinAngle", message.kingpinAngleMrad),
        jsonField("MotorSpeed", message.motorSpeedDeciRadPerSec),
        jsonField("MotorCurrent", message.motorCurrentMilliAmp),
        jsonField("EstLoadTorque", message.estLoadTorqueMilliNm),
        jsonField("ActiveMode", state & 3),
        jsonField("FaultJam", bitOf(state, 2)),
        jsonField("FaultDisagree", bitOf(state, 3)),
        jsonField("FaultCrc", bitOf(state, 4)),
        jsonField("FaultCounter", bitOf(state, 5)),
        jsonField("FaultTimeout", bitOf(state, 6)),
        jsonField("FaultInternal", bitOf(state, 7)),
    });
    printVector("SBW_Status", fieldsJson, frame);
}

void printHeartbeatVector(const sbw::HeartbeatMessage& message) {
    const std::string fieldsJson = joinFields({
        jsonField("NodeState", message.nodeState),
        jsonField("Uptime", message.uptimeMs),
        jsonField("BusVoltage", message.busVoltageMilliVolt),
    });
    printVector("SBW_Heartbeat", fieldsJson, sbw::packHeartbeat(message));
}

void printDiagVector(const sbw::DiagMessage& message) {
    const std::string fieldsJson = joinFields({
        jsonField("SteeringOverruns", message.steeringOverruns),
        jsonField("SteeringStepMaxCycles", message.steeringStepMaxCycles),
        jsonField("CurrentStepMaxCycles", message.currentStepMaxCycles),
        jsonField("StaleKingpinReads", message.staleKingpinReads),
        jsonField("DroppedRxFrames", message.droppedRxFrames),
    });
    printVector("SBW_Diag", fieldsJson, sbw::packDiag(message));
}

void printBuildVector(const sbw::BuildMessage& message) {
    const std::string fieldsJson = joinFields({
        jsonField("CommitHash", message.commitHash),
        jsonField("BuildType", static_cast<int>(message.buildType)),
        jsonField("UncommittedChanges", message.hasUncommittedChanges ? 1 : 0),
    });
    printVector("SBW_Build", fieldsJson, sbw::packBuild(message));
}

}  // namespace

int main() {
    printCommandVector({.targetKingpinAngleMrad = 100, .currentLimitMilliAmp = 1500,
                        .controlMode = 3, .aliveCounter = 1});
    printCommandVector({.targetKingpinAngleMrad = 0, .currentLimitMilliAmp = 0,
                        .controlMode = 0, .aliveCounter = 0});
    printCommandVector({.targetKingpinAngleMrad = -524, .currentLimitMilliAmp = 0,
                        .controlMode = 0, .aliveCounter = 15});
    printCommandVector({.targetKingpinAngleMrad = 524, .currentLimitMilliAmp = 2500,
                        .controlMode = 4, .aliveCounter = 7});
    printCommandVector({.targetKingpinAngleMrad = -32768, .currentLimitMilliAmp = 65535,
                        .controlMode = 2, .aliveCounter = 14});

    printStatusVector({.kingpinAngleMrad = -1, .motorSpeedDeciRadPerSec = 10055,
                       .motorCurrentMilliAmp = 1500, .estLoadTorqueMilliNm = -5,
                       .stateAndFaults = 0x07});
    printStatusVector({.kingpinAngleMrad = 32767, .motorSpeedDeciRadPerSec = -32768,
                       .motorCurrentMilliAmp = -32768, .estLoadTorqueMilliNm = -128,
                       .stateAndFaults = 0xFF});
    printStatusVector({.kingpinAngleMrad = 0, .motorSpeedDeciRadPerSec = 0,
                       .motorCurrentMilliAmp = 0, .estLoadTorqueMilliNm = 127,
                       .stateAndFaults = 0x00});

    printHeartbeatVector({.nodeState = 1, .uptimeMs = 0x01020304, .busVoltageMilliVolt = 12000});
    printHeartbeatVector({.nodeState = 2, .uptimeMs = 4294967295u, .busVoltageMilliVolt = 65535});

    printDiagVector({.steeringOverruns = 2, .steeringStepMaxCycles = 0x1234,
                     .currentStepMaxCycles = 0x0567, .staleKingpinReads = 3, .droppedRxFrames = 1});
    printDiagVector({.steeringOverruns = 65535, .steeringStepMaxCycles = 65535,
                     .currentStepMaxCycles = 65535, .staleKingpinReads = 255,
                     .droppedRxFrames = 255});

    printBuildVector({.commitHash = 0xa027448f, .buildType = sbw::BuildType::Release,
                      .hasUncommittedChanges = true});
    printBuildVector({.commitHash = 0xffffffff, .buildType = sbw::BuildType::Debug,
                      .hasUncommittedChanges = false});
    return 0;
}
