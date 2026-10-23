#include "transport/udp_server.hpp"
#include "transport/mavlink_parser.hpp"
#include "transport/mavlink_sender.hpp"
#include <cstdio> 
#include <cstdint>

int main () {
    const uint16_t SERVER_PORT = 14551;
    const uint8_t SENDER_SYSID = 254;
    try { 
        UdpServer server(SERVER_PORT);
        uint8_t buf[2048];
        MavlinkParser::Handler msg_handler = [](const mavlink_message_t& msg){
            switch (msg.msgid){
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
                        break;
                    }
                    const bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

                    std::printf("custom_mode %3u | %-8s | system_status %u\n",
                        static_cast<unsigned>(hb.custom_mode),
                        armed ? "ARMED" : "DISARMED",
                        static_cast<unsigned>(hb.system_status));

                    break;
            }
        };

        MavlinkParser parser{MAVLINK_COMM_0, std::move(msg_handler)};
        MavlinkSender sender{server, SENDER_SYSID, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_0};
        auto nextHeartbeat = std::chrono::steady_clock::now();                      

        while(1) {
            if (server.wait_readable(100)) {
                const size_t n = server.recv(buf, sizeof(buf));
                if (n > 0) {
                    parser.parse_buf({buf, n});
                }

                const auto now = std::chrono::steady_clock::now();
                if (now >= nextHeartbeat) {
                    sender.sendHeartbeat();
                    nextHeartbeat += std::chrono::seconds(1);
                    if (nextHeartbeat < now) {            // відстали більше ніж на період —
                        nextHeartbeat = now + std::chrono::seconds(1);   // не доганяємо, а починаємо заново
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