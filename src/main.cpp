#include "transport/udp_server.hpp"
#include "mavlink/mavlink_parser.hpp"
#include "mavlink/mavlink_sender.hpp"
#include "mavlink/mavlink_names.hpp"
#include <cstdio> 
#include <cstdint>

int main () {
    const uint16_t SERVER_PORT = 14551;
    const uint8_t SENDER_SYSID = 254;
    try { 
        UdpServer server(SERVER_PORT);
        uint8_t buf[2048];
        uint8_t vehicleSys;
        uint8_t vehicleComp;
        bool vehicleKnown = false; 

        MavlinkParser::Handler msg_handler = [&](const mavlink_message_t& msg){
            switch (msg.msgid){
                case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                         mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
                            break;
                        }

                        if (!vehicleKnown) {
                            vehicleSys = msg.sysid;
                            vehicleComp = msg.compid;
                            vehicleKnown = true;
                        }
                        const bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

                        std::printf("mode %-9s | %-8s | status %s\n",
                            mavlink_names::copterModeName(hb.custom_mode),
                            armed ? "ARMED" : "DISARMED",
                            mavlink_names::mavStateName(hb.system_status)
                        );

                        break;
                    }
                   
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    {
                       mavlink_command_ack_t command_ack;
                        mavlink_msg_command_ack_decode(&msg, &command_ack);

                        if (command_ack.target_system != SENDER_SYSID) { 
                            break;
                        }

                        std::printf("ACK command %u result %s\n",
                            static_cast<unsigned>(command_ack.command),
                            mavlink_names::mavResultName(command_ack.result)
                        );
                        break; 
                    }
                        
                default:
                    break;
            }
        };

        MavlinkParser parser{MAVLINK_COMM_0, std::move(msg_handler)};
        MavlinkSender sender{server, SENDER_SYSID, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_0};
        auto nextHeartbeat = std::chrono::steady_clock::now();
        bool guidedSent = false;

        while(1) {
            if (server.wait_readable(100)) {

                // recieve heartbeat
                const size_t n = server.recv(buf, sizeof(buf));
                if (n > 0) {
                    parser.parse_buf({buf, n});
                }

                //send command guided
                if (!guidedSent && vehicleKnown) {
                    guidedSent = sender.sendCommandLong(
                        vehicleSys,
                        vehicleComp,
                        MAV_CMD_DO_SET_MODE,
                        {MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_GUIDED, 0, 0, 0, 0, 0},
                        0
                    );
                }

                //send heartbeat
                const auto now = std::chrono::steady_clock::now();
                if (now >= nextHeartbeat) {
                    sender.sendHeartbeat();
                    nextHeartbeat += std::chrono::seconds(1);
                    if (now > nextHeartbeat) {
                        nextHeartbeat = now + std::chrono::seconds(1);
                    }
                }
            }
        }

        return 0;
    } catch (const std::exception& e) { 
        std::fprintf(stderr, "Error: %s\n", e.what()); 
        
        return 1; 
    }
}