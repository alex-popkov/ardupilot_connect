#include "transport/udp_server.hpp"
#include "mavlink/mavlink_parser.hpp"
#include "mavlink/mavlink_sender.hpp"
#include "vehicle/telemetry_handler.hpp"
#include <cstdio> 
#include <cstdint>

int main () {
    const uint16_t SERVER_PORT = 14551;
    const uint8_t SENDER_SYSID = 254;
    try { 
        UdpServer server(SERVER_PORT);
        uint8_t buf[2048];
        TelemetryHandler telemetry_handler{SENDER_SYSID};

        MavlinkParser::Handler msg_handler = [&](const mavlink_message_t& msg){
           telemetry_handler.onMessage(msg);
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
                if (!guidedSent && telemetry_handler.state().known) {
                    guidedSent = sender.sendCommandLong(
                        telemetry_handler.state().sysid,
                        telemetry_handler.state().compid,
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