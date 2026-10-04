#pragma once

#include <mc_rtc/Configuration.h>

#include <string>
#include <vector>

namespace mc_isaac
{

/** Lockstep link to the mc_isaac server.
 *
 * Frame: uint32 header length, uint32 payload length (network order), JSON header, float64 little-endian payload.
 */
class Bridge
{
public:
  ~Bridge();

  void connect(const std::string & host, int port, double timeout_s);
  void close();
  bool connected() const { return fd_ >= 0; }

  /** Send a request and wait for the reply. Throws std::runtime_error on transport or server error. */
  mc_rtc::Configuration request(const mc_rtc::Configuration & header,
                                const std::vector<double> & payload,
                                std::vector<double> & reply_payload);

private:
  int fd_ = -1;
};

} // namespace mc_isaac
