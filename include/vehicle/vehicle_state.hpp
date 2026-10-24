#pragma once
#include <chrono>    // std::chrono::steady_clock
#include <cstdint>   // uint8_t, uint32_t

struct VehicleState {
    bool known = false;
    uint8_t sysid = 0;
    uint8_t compid = 0;

    // HEARTBEAT
    uint32_t custom_mode = 0;
    bool armed = false;
    uint8_t system_status = 0;
    std::chrono::steady_clock::time_point last_heartbeat{};

    // GLOBAL_POSITION_INT
    bool has_position = false;
    double lat = 0, lon = 0;       // degrees
    double rel_alt_m = 0;            // metres
    double climb_rate_mps = 0;
};