#pragma once
#include <cstdint>
 
namespace mavlink_names {
 
// Текстові назви для значень MAVLink — для логів і відладки.
// Невідоме значення -> "UNKNOWN" (ніколи не nullptr).
 
const char* mavResultName(uint8_t result);      // COMMAND_ACK.result      (MAV_RESULT)
const char* mavStateName(uint8_t state);        // HEARTBEAT.system_status (MAV_STATE)
const char* copterModeName(uint32_t customMode); // HEARTBEAT.custom_mode — ЛИШЕ для ArduCopter
 
}  // namespace mavlink_names