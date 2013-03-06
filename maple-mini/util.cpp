
#include "util.h"

namespace util {

// See Adler-32 as described in RFC 1950.
uint32_t
adler32(const uint8_t * const data, const size_t data_len, const uint32_t seed)
{
    const uint32_t ADLER_BASE = 65521;
    uint32_t s1 = seed & 0xffff;
    uint32_t s2 = (seed >> 16) & 0xffff;

    for (size_t i = 0; i < data_len; i++) {
        s1 = (s1 + data[i]) % ADLER_BASE;
        s2 = (s2 + s1)      % ADLER_BASE;
    }

    return (s2 << 16) + s1;
}

}   // namespace util

