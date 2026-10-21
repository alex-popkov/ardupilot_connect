#include "port_controllers/udp_server.hpp"
#include <cstdio> 
#include <cstdint>

int main () {
    try { 
        UdpServer server(14551);
        uint8_t buf[2048];

        while(1) {
            if (server.waitReadable(100)) {
                const size_t n = server.recv(buf, sizeof(buf));

                if (n > 0) {
                    std::printf("received %zu bytes\n", n);
                }
            }
        }

        return 0;
    } catch (const std::exception& e) { 
        std::fprintf(stderr, "Error: %s\n", e.what()); 
        
        return 1; 
    }
}