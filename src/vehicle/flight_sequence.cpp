#include "vehicle/flight_sequence.hpp"
#include "mavlink/mavlink_utils.hpp"

namespace {
constexpr auto kAckTimeout = std::chrono::seconds(1);
constexpr uint8_t kMaxAttempts = 3;

const char* stateName(FlightSequence::State s)
{
    using State = FlightSequence::State;
    switch (s) {
        case State::WaitVehicle: return "WAIT_VEHICLE";
        case State::SetGuided:   return "SET_GUIDED";
        case State::Arming:      return "ARMING";
        case State::Takeoff:     return "TAKEOFF";
        case State::Climbing:    return "CLIMBING";
        case State::Landing:     return "LANDING";
        case State::Done:        return "DONE";
        case State::Failed:      return "FAILED";
    }
    return "UNKNOWN";  // unreachable if all enumerators are handled
}
    // How long a state may last before the sequence gives up (none = unlimited)
std::optional<std::chrono::seconds> stateTimeout(FlightSequence::State s)
{
    using State = FlightSequence::State;
    switch (s) {
        case State::SetGuided:   return std::chrono::seconds(5);
        case State::Arming:      return std::chrono::seconds(60);
        case State::Takeoff:     return std::chrono::seconds(5);
        case State::Climbing:    return std::chrono::seconds(30);
        case State::Landing:     return std::chrono::seconds(60);

        case State::Done:
        case State::Failed:
        case State::WaitVehicle: 
            return std::nullopt;
    }
};
}

FlightSequence::FlightSequence(
    MavlinkSender& sender, 
    const VehicleState& vehicle,
    uint8_t own_sysid, 
    float takeoff_alt_m
) : sender_(sender), vehicle_(vehicle), own_sysid_(own_sysid), takeoff_alt_m_(takeoff_alt_m)
{
}

void FlightSequence::onMessage(const mavlink_message_t& msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
        return;
    }
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    onCommandAck(ack);
}

void FlightSequence::tick(std::chrono::steady_clock::time_point now)
{
    checkCommandTimeout(now);

    switch (state_) {
        case State::WaitVehicle:
            if (vehicle_.known) {
                setState(State::SetGuided);
            }
            break;
        case State::Climbing:
            // Small tolerance: the vehicle may settle slightly below the target
            if (vehicle_.rel_alt_m >= takeoff_alt_m_ - 0.5f) {
                setState(State::Landing);
            }
            break;
        case State::Landing:
            // ArduCopter disarms by itself after touchdown
            if (!active_command_ && !vehicle_.armed) {
                setState(State::Done);
            }
            break;
        default:
            break;
    }
}

void FlightSequence::onCommandAck(const mavlink_command_ack_t& ack)
{
    // Not for us, or not the command we are waiting for
    if (ack.target_system != own_sysid_ || !active_command_ || ack.command != active_command_->command) {
        return;
    }
    if (ack.result == MAV_RESULT_IN_PROGRESS) {
        return;  // keep waiting for the final result
    }

    if (ack.result == MAV_RESULT_TEMPORARILY_REJECTED || (state_ == State::Arming && ack.result == MAV_RESULT_FAILED)) {
// повтор через паузу

// і її варто зробити явною: окрема функція чи таблиця «команда + результат → дія», а не вкладені if по всьому onCommandAck.
    }

    if (ack.result != MAV_RESULT_ACCEPTED) {
        std::printf("[seq] command %u rejected: %s\n",
                    static_cast<unsigned>(ack.command), mavlink_utils::mavResultName(ack.result));
        active_command_.reset();
        setState(State::Failed);
        return;
    }

    active_command_.reset();
    const auto next_state = nextStateOnAck(state_);
    if (next_state != std::nullopt) {
        setState(*next_state);
    }
}

std::optional<FlightSequence::State> FlightSequence::nextStateOnAck(State curr_state)
{
    switch (curr_state) {
        case State::SetGuided: 
            return State::Arming;

        case State::Arming:   
            return State::Takeoff;

        case State::Takeoff:  
            return State::Climbing;

        case State::Landing:
            return std::nullopt;  // mode switched; tick() waits for disarm
        default:              
            return std::nullopt;  // ACK we did not expect in this state
    }
};

void FlightSequence::setState(State next)
{
    std::printf("[seq] %s -> %s\n", stateName(state_), stateName(next));
    state_ = next;
    active_command_.reset();
    auto now = std::chrono::steady_clock::now();
    auto limit = stateTimeout(state_);
    if (limit){
        state_deadline_ = now + *limit;
    }

    switch (state_) {
        case State::SetGuided:
            sendCommand(MAV_CMD_DO_SET_MODE,
                        {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_GUIDED, 0, 0, 0, 0, 0});
            break;

        case State::Arming:
            sendCommand(MAV_CMD_COMPONENT_ARM_DISARM, {1, 0, 0, 0, 0, 0, 0});
            break;

        case State::Takeoff:
            sendCommand(MAV_CMD_NAV_TAKEOFF, {0, 0, 0, 0, 0, 0, takeoff_alt_m_});
            break;

        case State::Landing:
            sendCommand(MAV_CMD_DO_SET_MODE,
                        {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_LAND, 0, 0, 0, 0, 0});
            break;

        case State::WaitVehicle:
        case State::Climbing:
        case State::Done:
        case State::Failed:
            break;  // nothing to send, we just wait
    }
}

bool FlightSequence::sendCommand(uint16_t command, const std::array<float, 7>& params)
{
    bool sent = sender_.sendCommandLong(
        vehicle_.sysid,
        vehicle_.compid,
        command,
        params,
        0
    );

    active_command_ = ActiveCommand{command, params, ActiveCommand::Phase::AwaitingAck, std::chrono::steady_clock::now() + kAckTimeout, 1};
    
    return sent;
}

void FlightSequence::checkCommandTimeout(std::chrono::steady_clock::time_point now) 
{
    if (!active_command_) {
        return;
    }

    if (now - active_command_->sent_at < kAckTimeout) {
        return;
    }

     if (active_command_->attempts >= kMaxAttempts) {
        std::printf("[seq] command %u: no ACK after %u attempts\n",
                    static_cast<unsigned>(active_command_->command),
                    static_cast<unsigned>(active_command_->attempts));
        active_command_.reset();
        setState(State::Failed);

        return;
    }

     // Resend the same command; confirmation tells the receiver this is a retry
    const uint8_t confirmation = active_command_->attempts;
    sender_.sendCommandLong(
        vehicle_.sysid, 
        vehicle_.compid,
        active_command_->command, 
        active_command_->params, 
        confirmation
    );
    active_command_->attempts++;
    active_command_->sent_at = now;

    std::printf("[seq] command %u: retry %u\n",
                static_cast<unsigned>(active_command_->command),
                static_cast<unsigned>(confirmation));

}