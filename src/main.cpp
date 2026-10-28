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
        FlightSequence flight_fequence{sender, telemetry_handler.state(), kSenderSysid, kTakeoffAltM};

        MavlinkParser::OnMessage msg_handler = [&](const mavlink_message_t& msg){
           telemetry_handler.onMessage(msg);
           flight_fequence.onMessage(msg); 
        };

        MavlinkParser parser{MAVLINK_COMM_0, std::move(msg_handler)};
        auto nextHeartbeat = std::chrono::steady_clock::now();
        
        while(1) {
            if (server.wait_readable(100)) {

                // recieve heartbeat
                const size_t n = server.recv(buf, sizeof(buf));
                if (n > 0) {
                    parser.feed({buf, n});
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

                flight_fequence.tick(now); 

                if (flight_fequence.state() == FlightSequence::State::Done ||
                    flight_fequence.state() == FlightSequence::State::Failed) {
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


