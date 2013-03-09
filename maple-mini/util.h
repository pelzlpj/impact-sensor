
#ifndef INCLUDE_GUARD_6dce317f_d4b2_4df3_b055_969edfb2edc0
#define INCLUDE_GUARD_6dce317f_d4b2_4df3_b055_969edfb2edc0

#include <stdint.h>
#include <stddef.h>

#define ARRAY_COUNT(arr) (sizeof(arr)/sizeof(arr[0]))

namespace util {

const uint32_t ADLER32_SEED = 1;

// See Adler-32 as described in RFC 1950.
uint32_t
adler32(const uint8_t * data, size_t data_len, uint32_t seed = ADLER32_SEED);

}

#endif // INCLUDE_GUARD_6dce317f_d4b2_4df3_b055_969edfb2edc0

