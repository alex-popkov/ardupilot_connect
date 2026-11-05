#pragma once
#include <chrono>
#include <cstdint>
#include <optional>
#include <ardupilotmega/mavlink.h>

#include "mavlink/command_client.hpp"
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
        CommandClient& command_client,
        const VehicleState& vehicle,
        float takeoff_alt_m
    );

    // Call on every main-loop iteration (telemetry-driven transitions, later timeouts)
    void tick(std::chrono::steady_clock::time_point now);

    State state() const { return state_; }

private:
    static std::optional<FlightSequence::State> nextStateOnAck(State curr_state);
    void setState(State next);   // the only place where state_ changes
    void onCommandResult(const CommandClient::Result& result);
    void issue(uint16_t command, const std::array<float, 7>& params);

    CommandClient& command_client_;// read-only view, TelemetryHandler owns updates
    const VehicleState& vehicle_;   
    float takeoff_alt_m_;
    std::optional<uint16_t> expected_command_;
    State state_ = State::WaitVehicle;
    std::optional<std::chrono::steady_clock::time_point> state_deadline_; 
};