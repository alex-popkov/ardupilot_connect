#include <arpa/inet.h>   // htons, htonl
#include <fcntl.h>       // fcntl, O_NONBLOCK
#include <netinet/in.h>  // sockaddr_in, INADDR_ANY
#include <sys/socket.h>  // socket, bind
#include <unistd.h>      // close

#include <cerrno>
#include <cstring>       // strerror
#include <stdexcept>
#include <string>
#include <poll.h>
#include "transport/udp_server.hpp"

UdpServer::UdpServer(uint16_t port, const std::string& bindAddr)
{
   // 1. create UDP-socket: AF_INET = IPv4, SOCK_DGRAM = datagrams (UDP)
  fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }

  // 2. Local address
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);                // order of bytes (endianes)
  if (::inet_pton(AF_INET, bindAddr.c_str(), &addr.sin_addr) != 1) {
    // inet_pton return 0 in case of error,
    ::close(fd);
    fd = -1;
    throw std::runtime_error("bad bind address: " + bindAddr);
  }

  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
    fail_and_close("bind");
  }

  // 3. non blocking recvfrom() 
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    fail_and_close("fcntl");
  } 
}

UdpServer::~UdpServer()
{
  if (fd >= 0) {
    close(fd);
  }
}

bool UdpServer::send(const uint8_t* buf, size_t byteCount)
{
  if (!has_peer) {
     return false;
  }

  const ssize_t n = ::sendto(
    fd, 
    buf, 
    byteCount, 
    0,
    reinterpret_cast<const sockaddr*>(&peer),
    sizeof(peer)
  );

  if (n < 0) {
    const int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
      
      return false;
    }

    throw std::runtime_error(std::string("sendt?o: ") + std::strerror(err));
  }

  return true;
}

size_t UdpServer::recv(uint8_t* buf, size_t maxByteCount)
{
  sockaddr_in from{};
  socklen_t fromLen = sizeof(from);

  const ssize_t n = ::recvfrom(
    fd, 
    buf, 
    maxByteCount, 
    0, 
    reinterpret_cast<sockaddr*>(&from), 
    &fromLen
  );

  if (n < 0) {
    const int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
      
      return 0;
    }

    throw std::runtime_error(std::string("recvfrom: ") + std::strerror(err));
  }

  if (n > 0) {
    peer = from;
    has_peer = true;
  }

  return static_cast<size_t>(n);
}

bool UdpServer::wait_readable(int timeoutMs) {
  pollfd pfd{fd, POLLIN, 0};
  const int n = ::poll(&pfd, 1, timeoutMs);

  if (n == 0 ) {
    return false;
  }

  if (n < 0) {
    const int err = errno;
    if (err == EINTR) {
      
      return false;
    }

    throw std::runtime_error(std::string("poll: ") + std::strerror(err));
  }

  return (pfd.revents & POLLIN) != 0;
}

// private helper: close fd and throw error
[[noreturn]] void UdpServer::fail_and_close(const char* what)
{
  const int err = errno;   // save before close(), closecan rewrite errno
  ::close(fd);
  fd = -1;
  throw std::runtime_error(std::string(what) + ": " + std::strerror(err));
}