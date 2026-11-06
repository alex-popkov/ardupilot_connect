#pragma once
#include <ardupilotmega/mavlink.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

#include "mavlink/mavlink_sender.hpp"
#include "vehicle/vehicle_state.hpp"

// Delivers one COMMAND_LONG at a time: resends on silence, retries
// temporary refusals, and reports a single final result.
class CommandClient {
public:
    struct Result {
        enum class Kind { Accepted, Rejected, NoReply };
        Kind kind;
        uint16_t command;
        uint8_t mav_result; // MAV_RESULT_*; not meaningful for NoReply
    };

    CommandClient(MavlinkSender& sender, const VehicleState& vehicle);

    // Start a command. Returns false (and does nothing) if one is already active.
    [[nodiscard]] bool send(uint16_t command, const std::array<float, 7>& params);

    // Stop tracking the current command and drop any unread result
    void cancel();

    // True while a command is in flight or waiting for a retry
    bool busy() const { return active_command_.has_value(); }

    void onMessage(const mavlink_message_t& msg);         // COMMAND_ACK
    void tick(std::chrono::steady_clock::time_point now); // timeouts and retries

    // Final result of the last command; returned once, then empty
    std::optional<Result> takeResult();

private:
    struct ActiveCommand {
        enum class Phase { AwaitingAck, WaitingRetry };
        uint16_t command;
        std::array<float, 7> params;
        Phase phase;
        std::chrono::steady_clock::time_point deadline;
        uint8_t attempts;
    };

    void transmit(uint8_t confirmation);                // send active_ to the vehicle
    void finish(Result::Kind kind, uint8_t mav_result); // store result, clear active_
    void onCommandAck(const mavlink_command_ack_t& ack);
    void onCommandAckAccept(uint8_t result);
    void onCommandAckInProgress();
    void onCommandAckRetryLater();
    void onCommandAckFail(uint8_t result);

    MavlinkSender& sender_;
    const VehicleState& vehicle_;
    std::optional<ActiveCommand> active_command_;
    std::optional<Result> result_;
};