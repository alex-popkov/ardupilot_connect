#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <span>
#include <ardupilotmega/mavlink.h>

class MavlinkParser {

public:
  using Handler = std::function<void(const mavlink_message_t&)>;
  
  explicit MavlinkParser(mavlink_channel_t chan, Handler onMessage);

  void parse_buf(std::span<const std::uint8_t> buf);

private:
  mavlink_message_t msg_;
  mavlink_status_t status_;
  mavlink_channel_t chan_;
  Handler onMessage_;
};