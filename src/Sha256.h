#pragma once

#include <string>

namespace mc_isaac
{

/** SHA-256 of a byte buffer as lowercase hex (used as asset cache key on the Isaac server). */
std::string sha256_hex(const std::string & data);

} // namespace mc_isaac
