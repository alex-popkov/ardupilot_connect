#pragma once
#include <chrono>    // std::chrono::steady_clock
#include <cstdint>   // uint8_t, uint32_t

struct VehicleState {
    bool known = false;
    uint8_t sysid = 0;
    uint8_t compid = 0;

    // HEARTBEAT
    uint32_t customMode = 0;
    bool armed = false;
    uint8_t systemStatus = 0;
    std::chrono::steady_clock::time_point lastHeartbeat{};

    // GLOBAL_POSITION_INT
    bool hasPosition = false;
    double lat = 0, lon = 0;       // degrees
    double relAltM = 0;            // metres
};