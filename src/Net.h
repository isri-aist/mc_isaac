#pragma once

#include <string>

namespace mc_isaac
{

/** Blocking TCP connection with send/receive timeouts. Throws std::runtime_error on failure. */
int tcp_connect(const std::string & host, int port, double timeout_s, bool no_delay);

void send_all(int fd, const char * data, size_t size);

void recv_exact(int fd, char * data, size_t size);

struct HttpResponse
{
  int status = 0;
  std::string body;
};

/** Minimal HTTP/1.0 client for the mc_isaac server API (localhost, JSON or binary bodies). */
HttpResponse http_request(const std::string & host,
                          int port,
                          const std::string & method,
                          const std::string & path,
                          const std::string & body = "",
                          const std::string & content_type = "application/json",
                          double timeout_s = 5.0);

} // namespace mc_isaac
