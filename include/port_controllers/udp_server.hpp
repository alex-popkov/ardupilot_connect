#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <netinet/in.h>

class UdpServer {
public:
  explicit UdpServer(uint16_t port, const std::string& bindAddr = "0.0.0.0");
  ~UdpServer();
  UdpServer(const UdpServer&) = delete;  
  UdpServer& operator=(const UdpServer&) = delete;
  UdpServer& operator=(UdpServer&&) = delete;

  void send(const uint8_t* buf, size_t byteCount;
  void recv(uint8_t* buf, size_t max);

private:
  void UdpServer::failAndClose(const char* what)

  int fd{-1};
  sockaddr_in peer{};
  bool hasPeer{false};
};