#include "transport/udp_server.hpp"
#include "mavlink/mavlink_parser.hpp"
#include "mavlink/mavlink_sender.hpp"
#include "vehicle/telemetry_handler.hpp"
#include "vehicle/flight_sequence.hpp"
#include <cstdio> 
#include <cstdint>

int main () {
    const uint16_t kServerPort = 14551;
    const uint8_t kSenderSysid = 254;
    const float kTakeoffAltM = 10.0f;
    try { 
        UdpServer server(kServerPort);
        uint8_t buf[2048];
        TelemetryHandler telemetry_handler;
        MavlinkSender sender{server, kSenderSysid, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_0};
        FlightSequence flight_sequence{sender, telemetry_handler.state(), kSenderSysid, kTakeoffAltM};

        MavlinkParser::OnMessage msg_handler = [&](const mavlink_message_t& msg){
           telemetry_handler.onMessage(msg);
           flight_sequence.onMessage(msg); 
        };

        MavlinkParser parser{MAVLINK_COMM_0, std::move(msg_handler)};
        auto next_heartbeat = std::chrono::steady_clock::now();
        std::optional<std::chrono::steady_clock::time_point> exit_at;
        
        while(1) {
            if (server.wait_readable(100)) {

                // recieve heartbeat
                const size_t n = server.recv(buf, sizeof(buf));
                if (n > 0) {
                    parser.feed({buf, n});
                }

                //send heartbeat
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_heartbeat) {
                    sender.sendHeartbeat();
                    next_heartbeat += std::chrono::seconds(1);
                    if (now > next_heartbeat) {
                        next_heartbeat = now + std::chrono::seconds(1);
                    }
                }

                flight_sequence.tick(now); 

                 const bool finished = flight_sequence.state() == FlightSequence::State::Done ||
                          flight_sequence.state() == FlightSequence::State::Failed;

                // Keep the loop running a bit longer to catch late STATUSTEXT messages
                if (finished && !exit_at) {
                    exit_at = now + std::chrono::seconds(2);
                }
                if (exit_at && now >= *exit_at) {
                    break;
                }
            }
        }

        return 0;
    } catch (const std::exception& e) { 
        std::fprintf(stderr, "Error: %s\n", e.what()); 
        
        return 1; 
    }
}


