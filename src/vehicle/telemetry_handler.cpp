#include "vehicle/telemetry_handler.hpp"
#include "mavlink/mavlink_utils.hpp"
#include <cstdio>

void TelemetryHandler::onMessage(const mavlink_message_t& msg)
{
    // Until the autopilot is found, only heartbeats matter (they tell us its address)
    if (!state_.known && msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
        return;
    }

    // Once found, accept messages only from that autopilot: same system and same component
    if (state_.known && (msg.sysid != state_.sysid || msg.compid != state_.compid)) {
        return;
    }

    switch (msg.msgid){
        case MAVLINK_MSG_ID_HEARTBEAT:
            {
                onHeartbeat(msg);
                break;
            }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 
            {
                onGlobalPositionInt(msg);
                break; 
            }

        case MAVLINK_MSG_ID_STATUSTEXT:
            {
                onStatusText(msg);
                break;
            }
                
        default:
            break;
    }
}

const VehicleState& TelemetryHandler::state() const {
    return state_;
}


void TelemetryHandler::onHeartbeat(const mavlink_message_t& msg)
{
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
        return;
    }

    const bool was_known = state_.known;

    if (!state_.known) {
        state_.sysid = msg.sysid;
        state_.compid = msg.compid;
        state_.known = true;
    }  

    const bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

    // Print only when something visible changed (always print the very first heartbeat)
    const bool changed = !was_known
                    || hb.custom_mode != state_.custom_mode
                    || armed != state_.armed
                    || hb.system_status != state_.system_status;

    state_.custom_mode = hb.custom_mode;
    state_.armed = armed;
    state_.system_status = hb.system_status;
    state_.last_heartbeat = std::chrono::steady_clock::now();

    if (changed) {
        std::printf("mode %-9s | %-8s | status %s\n",
            mavlink_utils::copterModeName(state_.custom_mode),
            state_.armed ? "ARMED" : "DISARMED",
            mavlink_utils::mavStateName(state_.system_status)
        );
    }
}

void TelemetryHandler::onGlobalPositionInt(const mavlink_message_t& msg)
{
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(&msg, &pos);

    state_.lat = pos.lat / 1e7;            // degE7 -> degrees
    state_.lon = pos.lon / 1e7;
    state_.rel_alt_m = pos.relative_alt / 1000.0; // mm -> m
    state_.has_position = true;
    state_.climb_rate_mps = -pos.vz / 100.0; 
}

void TelemetryHandler::onStatusText(const mavlink_message_t& msg)
{
    mavlink_statustext_t st;
    mavlink_msg_statustext_decode(&msg, &st);

    // text is char[50] and is NOT guaranteed to be null-terminated
    std::printf("[autopilot sev=%s] %.*s\n",
        mavlink_utils::mavSeverityName(st.severity),
        static_cast<int>(sizeof(st.text)), st.text);
}