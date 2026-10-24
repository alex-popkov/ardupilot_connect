#pragma once
#include <cstdint>
 
namespace mavlink_names {
 
 
const char* mavResultName(uint8_t result);      // COMMAND_ACK.result      (MAV_RESULT)
const char* mavStateName(uint8_t state);        // HEARTBEAT.system_status (MAV_STATE)
const char* copterModeName(uint32_t customMode); // HEARTBEAT.custom_mode — only for ArduCopter
 
}  // namespace mavlink_names