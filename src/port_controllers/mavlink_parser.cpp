#include "port_controllers/mavlink_parser.hpp"
#include <stdexcept>
#include <utility>

MavlinkParser::MavlinkParser(mavlink_channel_t chan, Handler onMessage) : chan_(chan), onMessage_(std::move(onMessage))
{
  if (!onMessage_) {
     throw std::runtime_error("empty message handler");
  }
}

void MavlinkParser::parse_buf(std::span<const std::uint8_t> buf) {
  for (const uint8_t byte : buf) {
      if (mavlink_parse_char(chan_, byte, &msg_, &status_)) {
        onMessage_(msg_);
      }
  }
}
