#include "Bridge.h"

#include "Net.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace mc_isaac
{

namespace
{

bool little_endian()
{
  const uint16_t one = 1;
  return *reinterpret_cast<const uint8_t *>(&one) == 1;
}

} // namespace

Bridge::~Bridge()
{
  close();
}

void Bridge::connect(const std::string & host, int port, double timeout_s)
{
  if(!little_endian()) { throw std::runtime_error("mc_isaac bridge only supports little-endian hosts"); }
  close();
  fd_ = tcp_connect(host, port, timeout_s, true);
}

void Bridge::close()
{
  if(fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

mc_rtc::Configuration Bridge::request(const mc_rtc::Configuration & header,
                                      const std::vector<double> & payload,
                                      std::vector<double> & reply_payload)
{
  if(fd_ < 0) { throw std::runtime_error("bridge not connected"); }
  const std::string raw = header.dump();
  const size_t payload_bytes = payload.size() * sizeof(double);
  std::string frame(8 + raw.size() + payload_bytes, '\0');
  const uint32_t sizes[2] = {htonl(static_cast<uint32_t>(raw.size())), htonl(static_cast<uint32_t>(payload_bytes))};
  std::memcpy(&frame[0], sizes, 8);
  std::memcpy(&frame[8], raw.data(), raw.size());
  if(payload_bytes) { std::memcpy(&frame[8 + raw.size()], payload.data(), payload_bytes); }
  send_all(fd_, frame.data(), frame.size());

  uint32_t reply_sizes[2];
  recv_exact(fd_, reinterpret_cast<char *>(reply_sizes), 8);
  std::string reply_raw(ntohl(reply_sizes[0]), '\0');
  recv_exact(fd_, &reply_raw[0], reply_raw.size());
  const size_t reply_bytes = ntohl(reply_sizes[1]);
  reply_payload.resize(reply_bytes / sizeof(double));
  if(reply_bytes) { recv_exact(fd_, reinterpret_cast<char *>(reply_payload.data()), reply_bytes); }

  auto reply = mc_rtc::Configuration::fromData(reply_raw);
  if(!reply("ok", false))
  {
    throw std::runtime_error("Isaac server error: " + reply("error", std::string("unknown error")));
  }
  return reply;
}

} // namespace mc_isaac
