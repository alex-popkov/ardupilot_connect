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

  bool send(const uint8_t* buf, size_t byteCount);
  size_t recv(uint8_t* buf, size_t maxByteCount);
  bool waitReadable(int timeoutMs);

private:
  [[noreturn]] void failAndClose(const char* what);

  int fd{-1};
  sockaddr_in peer{};
  bool hasPeer{false};
};