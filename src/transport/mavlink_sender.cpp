#include <transport/mavlink_sender.hpp>


MavlinkSender::MavlinkSender(UdpServer& transport, uint8_t sysid, uint8_t compid, mavlink_channel_t chan)
 : transport_(transport), sysid_(sysid), compid_(compid), chan_(chan)
{
}


bool MavlinkSender::sendHeartbeat()
{
  mavlink_message_t msg{};
  mavlink_msg_heartbeat_pack_chan(sysid_, compid_, chan_, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  
  return sendMessage(msg);
}

bool MavlinkSender::sendCommandLong(
  uint8_t targetSys,
  uint8_t targetComp,
  uint16_t command,
  const std::array<float, 7>& params,
  uint8_t confirmation
)
{
  mavlink_message_t msg{};
  mavlink_msg_command_long_pack_chan(
    sysid_,
    compid_,
    chan_,
    &msg,
    targetSys,
    targetComp,
    command,
    confirmation,
    params[0], params[1], params[2], params[3],
    params[4], params[5], params[6]
  );

  return sendMessage(msg);
}

bool MavlinkSender::sendMessage(const mavlink_message_t& msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  auto n = mavlink_msg_to_send_buffer(buf, &msg);

  return transport_.send(buf, n);
}



