#include "port_controllers/udp_server.hpp"
#include "port_controllers/mavlink_parser.hpp"
#include <cstdio> 
#include <cstdint>

int main () {
    try { 
        UdpServer server(14551);
        uint8_t buf[2048];
        MavlinkParser::Handler msg_handler = [](const mavlink_message_t& msg){
            std::printf("seq %3u | sysid %3u | compid %3u | msgid %5u\n",
                static_cast<unsigned>(msg.seq),
                static_cast<unsigned>(msg.sysid),
                static_cast<unsigned>(msg.compid),
                static_cast<unsigned>(msg.msgid)
            );
        };
        MavlinkParser parser{MAVLINK_COMM_0, std::move(msg_handler)};


        while(1) {
            if (server.wait_readable(100)) {
                const size_t n = server.recv(buf, sizeof(buf));
                if (n > 0) {
                    parser.parse_buf({buf, n});
                }
            }
        }

        return 0;
    } catch (const std::exception& e) { 
        std::fprintf(stderr, "Error: %s\n", e.what()); 
        
        return 1; 
    }
}