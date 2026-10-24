#include "vehicle/telemetry_handler.hpp"
#include "mavlink/mavlink_utils.hpp"
#include <cstdio>

TelemetryHandler::TelemetryHandler(uint8_t sender_sys_id) : sender_sys_id_(sender_sys_id)
{}

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
            
        case MAVLINK_MSG_ID_COMMAND_ACK:
            {
                onCommandAck(msg);
                break; 
            }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 
            {
                onGlobalPositionInt(msg);
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

    if (!state_.known) {
        state_.sysid = msg.sysid;
        state_.compid = msg.compid;
        state_.known = true;
    }

    state_.custom_mode = hb.custom_mode;
    state_.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    state_.system_status = hb.system_status;
    state_.last_heartbeat = std::chrono::steady_clock::now();

    std::printf("mode %-9s | %-8s | status %s\n",
        mavlink_utils::copterModeName(state_.custom_mode),
        state_.armed ? "ARMED" : "DISARMED",
        mavlink_utils::mavStateName(state_.system_status)
    );
}

void TelemetryHandler::onCommandAck(const mavlink_message_t& msg)
{
    mavlink_command_ack_t command_ack;
    mavlink_msg_command_ack_decode(&msg, &command_ack);

    if (command_ack.target_system != sender_sys_id_) { 
        return;
    }

    std::printf("ACK command %u result %s\n",
        static_cast<unsigned>(command_ack.command),
        mavlink_utils::mavResultName(command_ack.result)
    );
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