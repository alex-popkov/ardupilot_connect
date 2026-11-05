#include "vehicle/flight_sequence.hpp"
#include "mavlink/mavlink_utils.hpp"
#include <cstdio>

namespace {
using State = FlightSequence::State;

const char* stateName(FlightSequence::State s)
{
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
    switch (s) {
        case State::SetGuided:   return std::chrono::seconds(5);
        case State::Arming:      return std::chrono::seconds(120);
        case State::Takeoff:     return std::chrono::seconds(5);
        case State::Climbing:    return std::chrono::seconds(30);
        case State::Landing:     return std::chrono::seconds(60);

        case State::Done:
        case State::Failed:
        case State::WaitVehicle: 
            return std::nullopt;
    }

    return std::nullopt;
}
}

FlightSequence::FlightSequence(
    CommandClient& command_client,
    const VehicleState& vehicle,
    float takeoff_alt_m
) : command_client_(command_client), vehicle_(vehicle), takeoff_alt_m_(takeoff_alt_m)
{
}

void FlightSequence::tick(std::chrono::steady_clock::time_point now)
{

    if (state_deadline_ && now >= *state_deadline_) {
        std::printf("[seq] %s timed out\n", stateName(state_));
        setState(State::Failed);
         
        return;
    }

    if (const auto result = command_client_.takeResult()) {
        onCommandResult(*result);
    }

    switch (state_) {
        case State::WaitVehicle:
         // Wait until the autopilot is found AND has finished booting
            if (vehicle_.known && vehicle_.system_status == MAV_STATE_STANDBY) {
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
            if (!command_client_.busy() && !vehicle_.armed) {
                setState(State::Done);
            }
            break;

        case State::SetGuided:
            // ACK only says "accepted"; move on when the heartbeat confirms the mode
            if (!command_client_.busy() && vehicle_.custom_mode == COPTER_MODE_GUIDED) {
                setState(State::Arming);
            }
            break;

        case State::Arming:
            if (!command_client_.busy() && vehicle_.armed) {
                setState(State::Takeoff);
            }
            break;

        default:
            break;
    }
}


std::optional<FlightSequence::State> FlightSequence::nextStateOnAck(State curr_state)
{
    switch (curr_state) {
        case State::Takeoff:  
            return State::Climbing;

        case State::SetGuided: 
        case State::Landing:
        case State::Arming:
            return std::nullopt;  // mode switched; tick() waits for disarm
        default:              
            return std::nullopt;  // ACK we did not expect in this state
    }
}

void FlightSequence::setState(State next)
{
    std::printf("[seq] %s -> %s\n", stateName(state_), stateName(next));
    state_ = next;
    command_client_.cancel(); 
    expected_command_.reset();
    const auto limit = stateTimeout(state_);
    if (limit){
        state_deadline_ = std::chrono::steady_clock::now() + *limit;
    } else {
        state_deadline_.reset();   // no limit for this state
    }

    switch (state_) {
        case State::SetGuided:
            issue(MAV_CMD_DO_SET_MODE,
                        {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_GUIDED, 0, 0, 0, 0, 0});
            break;

        case State::Arming:
            issue(MAV_CMD_COMPONENT_ARM_DISARM, {1, 0, 0, 0, 0, 0, 0});
            break;

        case State::Takeoff:
            issue(MAV_CMD_NAV_TAKEOFF, {0, 0, 0, 0, 0, 0, takeoff_alt_m_});
            break;

        case State::Landing:
            issue(MAV_CMD_DO_SET_MODE,
                        {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_LAND, 0, 0, 0, 0, 0});
            break;

        case State::WaitVehicle:
        case State::Climbing:
        case State::Done:
        case State::Failed:
            break;  // nothing to send, we just wait
    }
}

 void FlightSequence::issue(uint16_t command, const std::array<float, 7>& params)
  {
      if (!command_client_.send(command, params)) {
          std::printf("[seq] command client busy, cannot send %u\n",
                      static_cast<unsigned>(command));
          setState(State::Failed);
          return;
      }
      expected_command_ = command;
  }

void FlightSequence::onCommandResult(const CommandClient::Result& result)
{
    // Ignore results of commands this state did not issue
    if (!expected_command_ || result.command != *expected_command_) {
        return;
    }
    expected_command_.reset();

    switch (result.kind) {
        case CommandClient::Result::Kind::Accepted:
            if (const auto next = nextStateOnAck(state_)) {
                setState(*next);
            }
            break;

        case CommandClient::Result::Kind::Rejected:
            std::printf("[seq] command %u rejected: %s\n",
                        static_cast<unsigned>(result.command),
                        mavlink_utils::mavResultName(result.mav_result));
            setState(State::Failed);
            break;

        case CommandClient::Result::Kind::NoReply:
            std::printf("[seq] command %u: vehicle did not respond\n",
                        static_cast<unsigned>(result.command));
            setState(State::Failed);
            break;
    }
}