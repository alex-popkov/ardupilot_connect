#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <span> 
#include <ardupilotmega/mavlink.h>
#include <transport/udp_server.hpp>

class MavlinkSender {
public:
  MavlinkSender(
    UdpServer& transport,
    uint8_t sysid,
    uint8_t compid, 
    mavlink_channel_t chan
  );
  bool sendHeartbeat();
  bool sendCommandLong(
    uint8_t targetSys,
    uint8_t targetComp,
    uint16_t command,
    const std::array<float, 7>& params,
    uint8_t confirmation = 0
  );
  
private:
  bool sendMessage(const mavlink_message_t& msg);   // серіалізувати + transport.send

  UdpServer& transport_;
  uint8_t sysid_;
  uint8_t compid_;
  mavlink_channel_t chan_;
};