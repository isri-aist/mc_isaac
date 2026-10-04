#include "Net.h"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace mc_isaac
{

namespace
{

struct FdCloser
{
  int fd;
  ~FdCloser()
  {
    if(fd >= 0) { ::close(fd); }
  }
};

std::runtime_error net_error(const std::string & what)
{
  return std::runtime_error(what + ": " + std::strerror(errno));
}

} // namespace

int tcp_connect(const std::string & host, int port, double timeout_s, bool no_delay)
{
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * res = nullptr;
  if(int err = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res); err != 0)
  {
    throw std::runtime_error("cannot resolve " + host + ": " + gai_strerror(err));
  }
  timeval tv{};
  tv.tv_sec = static_cast<time_t>(timeout_s);
  tv.tv_usec = static_cast<suseconds_t>((timeout_s - std::floor(timeout_s)) * 1e6);
  int fd = -1;
  for(addrinfo * ai = res; ai != nullptr; ai = ai->ai_next)
  {
    fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if(fd < 0) { continue; }
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if(::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) { break; }
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  if(fd < 0) { throw net_error("cannot connect to " + host + ":" + std::to_string(port)); }
  if(no_delay)
  {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  }
  return fd;
}

void send_all(int fd, const char * data, size_t size)
{
  while(size > 0)
  {
    ssize_t n = ::send(fd, data, size, MSG_NOSIGNAL);
    if(n < 0)
    {
      if(errno == EINTR) { continue; }
      throw net_error("send failed");
    }
    data += n;
    size -= static_cast<size_t>(n);
  }
}

void recv_exact(int fd, char * data, size_t size)
{
  while(size > 0)
  {
    ssize_t n = ::recv(fd, data, size, 0);
    if(n == 0) { throw std::runtime_error("connection closed by the Isaac server"); }
    if(n < 0)
    {
      if(errno == EINTR) { continue; }
      throw net_error("receive failed");
    }
    data += n;
    size -= static_cast<size_t>(n);
  }
}

HttpResponse http_request(const std::string & host,
                          int port,
                          const std::string & method,
                          const std::string & path,
                          const std::string & body,
                          const std::string & content_type,
                          double timeout_s)
{
  FdCloser fd{tcp_connect(host, port, timeout_s, false)};
  std::string request = method + " " + path + " HTTP/1.0\r\nHost: " + host + "\r\nConnection: close\r\n";
  if(!body.empty() || method == "POST" || method == "PUT")
  {
    request += "Content-Type: " + content_type + "\r\nContent-Length: " + std::to_string(body.size()) + "\r\n";
  }
  request += "\r\n";
  send_all(fd.fd, request.data(), request.size());
  send_all(fd.fd, body.data(), body.size());

  // the server closes the connection after the response (HTTP/1.0)
  std::string raw;
  char buffer[65536];
  while(true)
  {
    ssize_t n = ::recv(fd.fd, buffer, sizeof(buffer), 0);
    if(n == 0) { break; }
    if(n < 0)
    {
      if(errno == EINTR) { continue; }
      throw net_error("HTTP " + method + " " + path + " failed");
    }
    raw.append(buffer, static_cast<size_t>(n));
  }
  HttpResponse response;
  auto space = raw.find(' ');
  auto header_end = raw.find("\r\n\r\n");
  if(space == std::string::npos || raw.size() < space + 4)
  {
    throw std::runtime_error("invalid HTTP response to " + method + " " + path);
  }
  response.status = std::stoi(raw.substr(space + 1, 3));
  if(header_end != std::string::npos) { response.body = raw.substr(header_end + 4); }
  return response;
}

} // namespace mc_isaac
