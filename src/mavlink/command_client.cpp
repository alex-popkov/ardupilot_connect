#include <cstdio>

#include "mavlink/mavlink_utils.hpp"
#include "mavlink/command_client.hpp"


namespace {
constexpr auto kAckTimeout = std::chrono::seconds(1);
constexpr auto kRetryDelay = std::chrono::seconds(3);
constexpr uint8_t kMaxAttempts = 3;

enum class AckAction { 
  Accept, 
  InProgress, 
  RetryLater,
  Fail 
};

AckAction classifyAck(uint16_t command, uint8_t result)
{
  switch(result) {
    case MAV_RESULT_ACCEPTED:                   
      return AckAction::Accept;

    case MAV_RESULT_IN_PROGRESS:                
      return AckAction::InProgress;

    case MAV_RESULT_TEMPORARILY_REJECTED:       
      return AckAction::RetryLater;

    case MAV_RESULT_FAILED: {
      if(command == MAV_CMD_COMPONENT_ARM_DISARM) {
        return AckAction::RetryLater;
      }

      return AckAction::Fail;
    }
        
    default: return AckAction::Fail;
  }
}
}


CommandClient::CommandClient(MavlinkSender& sender, const VehicleState& vehicle): sender_(sender), vehicle_(vehicle)
{}

bool CommandClient::send(uint16_t command, const std::array<float, 7>& params)
{
  if (active_command_) {
    return false;   // one command at a time
  }

  active_command_ = ActiveCommand{command, params, ActiveCommand::Phase::AwaitingAck, std::chrono::steady_clock::now() + kAckTimeout, 1};
  transmit(0);

  return true;
}

void CommandClient::cancel()
{
  active_command_.reset();
  result_.reset();
}

void CommandClient::onMessage(const mavlink_message_t& msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
        return;
    }
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    onCommandAck(ack);
}

void CommandClient::tick(std::chrono::steady_clock::time_point now)
{
    if (!active_command_ || now < active_command_->deadline) {
        return;
    }

    switch (active_command_->phase) {
      case ActiveCommand::Phase::AwaitingAck:
        if (active_command_->attempts >= kMaxAttempts) {
          std::printf("[cmd] command %u: no ACK after %u attempts\n",
                        static_cast<unsigned>(active_command_->command),
                        static_cast<unsigned>(active_command_->attempts));
          finish(Result::Kind::NoReply, 0); // here the result is created
          return;
        }
        transmit(active_command_->attempts); // retransmission, confirmation = attempts
        ++active_command_->attempts;
        active_command_->deadline = now + kAckTimeout;
        std::printf("[cmd] command %u: no ACK, resend %u\n",
                        static_cast<unsigned>(active_command_->command),
                        static_cast<unsigned>(active_command_->attempts));

        break;

      case ActiveCommand::Phase::WaitingRetry:
        transmit(0); // fresh attempt after a refusal
        active_command_->attempts = 1;
        active_command_->phase = ActiveCommand::Phase::AwaitingAck;
        active_command_->deadline = now + kAckTimeout;
        std::printf("[cmd] command %u: retrying after rejection\n",
                        static_cast<unsigned>(active_command_->command));

        break;
    }
}

std::optional<CommandClient::Result> CommandClient::takeResult()
{
  auto r = result_;
  result_.reset(); 
  
  return r;
}

void CommandClient::transmit(uint8_t confirmation)
{
    sender_.sendCommandLong(
      vehicle_.sysid, 
      vehicle_.compid,
      active_command_->command,
      active_command_->params,
      confirmation
    );
}

void CommandClient::finish(CommandClient::Result::Kind kind, uint8_t mav_result)
{
  result_ = CommandClient::Result{
    .kind = kind,
    .command = active_command_->command,
    .mav_result = mav_result
  };

  active_command_.reset();
} 

void CommandClient::onCommandAck(const mavlink_command_ack_t& ack)
{
    // Not for us, or not the command we are waiting for
    if (ack.target_system != sender_.sysid() || !active_command_ || ack.command != active_command_->command) {
        return;
    }

    if (active_command_->phase == ActiveCommand::Phase::WaitingRetry) {
        return;
    }

    const AckAction ack_action = classifyAck(ack.command, ack.result);
    switch (ack_action){
        case AckAction::Accept: {
            onCommandAckAccept(ack.result);
            break;
        }

        case AckAction::InProgress: {
            onCommandAckInProgress();
            break;
        }

        case AckAction::RetryLater: {
            onCommandAckRetryLater();
            std::printf("[cmd] rejected: %s, retry in %lld s\n",
                mavlink_utils::mavResultName(ack.result),
                static_cast<long long>(kRetryDelay.count())
            );
            break;
        }

        case AckAction::Fail: {
            onCommandAckFail(ack.result);
            std::printf("[cmd] command %u rejected: %s\n",
                    static_cast<unsigned>(ack.command), mavlink_utils::mavResultName(ack.result));

            break;
        }
    }
}

void CommandClient::onCommandAckAccept(uint8_t result)
{
  finish(Result::Kind::Accepted, result);
}

void CommandClient::onCommandAckInProgress()
{
    const auto now = std::chrono::steady_clock::now();
    active_command_->deadline = now + kAckTimeout;
    active_command_->attempts = 0;
}

void CommandClient::onCommandAckRetryLater()
{
    const auto now = std::chrono::steady_clock::now();
    active_command_->phase = ActiveCommand::Phase::WaitingRetry;
    active_command_->deadline = now + kRetryDelay;
    active_command_->attempts = 0;
}

void CommandClient::onCommandAckFail(uint8_t result)
{
  finish(Result::Kind::Rejected, result);
}