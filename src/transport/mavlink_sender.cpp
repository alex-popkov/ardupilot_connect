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

bool MavlinkSender::sendMessage(const mavlink_message_t& msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  auto n = mavlink_msg_to_send_buffer(buf, &msg);

  return transport_.send(buf, n);
}



