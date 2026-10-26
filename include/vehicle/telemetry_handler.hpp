#pragma once
#include "vehicle/vehicle_state.hpp"
#include <ardupilotmega/mavlink.h>

class TelemetryHandler {
public:
    void onMessage(const mavlink_message_t& msg);
    const VehicleState& state() const;
  
private:

    void onHeartbeat(const mavlink_message_t& msg);
    void onGlobalPositionInt(const mavlink_message_t& msg);
    void onStatusText(const mavlink_message_t& msg);
    VehicleState state_{};
};