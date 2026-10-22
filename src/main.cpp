#include "port_controllers/udp_server.hpp"
#include <cstdio> 
#include <cstdint>
#include <ardupilotmega/mavlink.h>

int main () {
    try { 
        UdpServer server(14551);
        uint8_t buf[2048];

        while(1) {
            if (server.waitReadable(100)) {
                const size_t n = server.recv(buf, sizeof(buf));

                if (n > 0) {
                    std::printf("received %zu bytes \n", n);
                    mavlink_message_t msg;
                    mavlink_status_t status;
                    for (size_t i = 0; i < n; ++i) {
                        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                            std::printf("seq %3u | sysid %3u | compid %3u | msgid %5u\n",
                                static_cast<unsigned>(msg.seq),
                                static_cast<unsigned>(msg.sysid),
                                static_cast<unsigned>(msg.compid),
                                static_cast<unsigned>(msg.msgid)
                            );
                        }
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