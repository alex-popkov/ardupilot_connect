#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <span> 
#include <ardupilotmega/mavlink.h>
#include <transport/udp_server.hpp>

class MavlinkSender {
public:
  MavlinkSender(UdpServer& transport, uint8_t sysid, uint8_t compid, mavlink_channel_t chan);
  bool sendHeartbeat();
private:
  bool sendMessage(const mavlink_message_t& msg);   // серіалізувати + transport.send

  UdpServer& transport_;
  uint8_t sysid_;
  uint8_t compid_;
  mavlink_channel_t chan_;
};