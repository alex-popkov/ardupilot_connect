#pragma once
#include <chrono>
#include <cstdint>
#include <optional>
#include <ardupilotmega/mavlink.h>

#include "mavlink/mavlink_sender.hpp"
#include "vehicle/vehicle_state.hpp"

class FlightSequence {
public:
    enum class State { 
        WaitVehicle, 
        SetGuided, 
        Arming, 
        Takeoff, 
        Climbing, 
        Landing, 
        Done, 
        Failed
     };

    FlightSequence(
        MavlinkSender& sender, 
        const VehicleState& vehicle,
        uint8_t own_sysid, 
        float takeoff_alt_m
    );

    // Feed every incoming message (used for COMMAND_ACK)
    void onMessage(const mavlink_message_t& msg);

    // Call on every main-loop iteration (telemetry-driven transitions, later timeouts)
    void tick(std::chrono::steady_clock::time_point now);

    State state() const { return state_; }

private:
    static std::optional<FlightSequence::State> nextStateOnAck(State curr_state);
    void setState(State next);   // the only place where state_ changes
    void onCommandAck(const mavlink_command_ack_t& ack);
    bool sendCommand(uint16_t command, const std::array<float, 7>& params);

    MavlinkSender& sender_;
    const VehicleState& vehicle_;   // read-only view, TelemetryHandler owns updates
    uint8_t own_sysid_;
    float takeoff_alt_m_;

    State state_ = State::WaitVehicle;
    std::optional<uint16_t> pending_command_;   // command we are waiting an ACK for
};