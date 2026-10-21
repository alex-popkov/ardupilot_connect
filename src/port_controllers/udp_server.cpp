#include <arpa/inet.h>   // htons, htonl
#include <fcntl.h>       // fcntl, O_NONBLOCK
#include <netinet/in.h>  // sockaddr_in, INADDR_ANY
#include <sys/socket.h>  // socket, bind
#include <unistd.h>      // close

#include <cerrno>
#include <cstring>       // strerror
#include <stdexcept>
#include <string>
#include "port_controllers/udp_server.hpp"

UdpServer::UdpServer(uint16_t port, const std::string& bindAddr = "0.0.0.0")
{
   // Створюємо UDP-сокет: AF_INET = IPv4, SOCK_DGRAM = датаграми (UDP)
  fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }

  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &dest.sin_addr) != 1) {
    close(fd);
    throw std::runtime_error("Bad UDP host: " + host + "\n");
  }

   // Локальна адреса, на якій слухаємо
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);                // порядок байтів хоста -> мережевий
  addr.sin_addr.s_addr = htonl(INADDR_ANY);   // 0.0.0.0, усі інтерфейси

  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
    failAndClose("bind");
  }

  // Неблокуючий режим: recvfrom() повертає одразу, навіть якщо даних немає
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    failAndClose("fcntl");
  }  
}

UdpServer::~UdpServer()
{
  if (fd >= 0) {
    close(fd);
  }
}

auto UdpServer::send(const uint8_t* buf, size_t byteCount) -> voida
{
  ::sendto(fd, buf, byteCount, 0, (const sockaddr*)&dest, sizeof(dest));
}

auto UdpServer::recv(uint8_t* buf, size_t maxByteCount) -> int
{
  return ::recvfrom(fd, buf, maxByteCount, 0, nullptr, nullptr);
}

// private helper: закрити fd і кинути виключення з текстом помилки
[[noreturn]] void UdpServer::failAndClose(const char* what)
{
  const int err = errno;   // зберегти ДО close(), бо close може перезаписати errno
  ::close(fd);
  fd = -1;
  throw std::runtime_error(std::string(what) + ": " + std::strerror(err));
}