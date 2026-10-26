#include "mavlink/mavlink_parser.hpp"
#include <stdexcept>
#include <utility>

MavlinkParser::MavlinkParser(mavlink_channel_t chan, OnMessage on_message) : chan_(chan), on_message_(std::move(on_message))
{
  if (!on_message_) {
     throw std::runtime_error("empty message handler");
  }
}

void MavlinkParser::feed(std::span<const std::uint8_t> buf) {
  for (const uint8_t byte : buf) {
      if (mavlink_parse_char(chan_, byte, &msg_, &status_)) {
        on_message_(msg_);
      }
  }
}
