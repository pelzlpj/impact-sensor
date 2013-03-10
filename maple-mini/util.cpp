
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

// Integer square root via bisection search.
uint32_t
sqrt_uint32(const uint32_t val)
{
	const uint64_t val64 = val;

    uint32_t lo = 0;
    uint32_t hi = (val / 2) + 1;
    while (lo + 1 < hi) {
        const uint64_t mid  = (lo + hi) / 2;
		const uint64_t mid2 = mid * mid;
        if (mid2 < val64) {
            lo = mid;
        } else if (mid2 > val64) {
            hi = mid;
        } else {
			return mid;
		}
    }

	return (lo * lo) == val ? lo : hi;
}


}   // namespace util

