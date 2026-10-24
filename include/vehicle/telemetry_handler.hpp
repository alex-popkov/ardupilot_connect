#pragma once
#include "vehicle/vehicle_state.hpp"
#include <ardupilotmega/mavlink.h>

class TelemetryHandler {
public:
    TelemetryHandler(uint8_t sender_sys_id);

    void onMessage(const mavlink_message_t& msg);
    const VehicleState& state() const;
  
private:

    void onHeartbeat(const mavlink_message_t& msg);
    void onCommandAck(const mavlink_message_t& msg);
    void onGlobalPositionInt(const mavlink_message_t& msg);
    VehicleState state_{};
    uint8_t sender_sys_id_;
};