// Tests whether a DBC signal lies entirely inside a received CAN payload.
// A layout can describe more fields than were transmitted. Reading a field
// from an incomplete payload can turn missing bytes into a plausible value.
//
// Intel starts at the least significant bit and advances to higher bit
// indices. Motorola starts at the most significant bit, descends within a
// byte, and continues at the next byte's most significant bit.

#pragma once

#include <cstddef>
#include <string>

namespace mqtt_consumer {

// The recognized little-endian labels select Intel bit order. Other labels
// select Motorola bit order.
inline bool is_intel_byte_order(const std::string& bo) {
    return !bo.empty()
        && (bo == "intel" || bo == "little" || bo == "le" || bo == "little_endian");
}

inline bool signal_fits_frame(bool intel, int start_bit, int bit_length,
                              std::size_t frame_bytes) {
    if (bit_length <= 0 || start_bit < 0) return false;
    if (frame_bytes == 0) return false;

    if (intel) {
        const auto last_bit = static_cast<unsigned long long>(start_bit) + bit_length - 1;
        return last_bit / 8 < frame_bytes;
    }

    // Motorola: bits available in the first byte, from the MSB position
    // down to bit 0 of that byte.
    const int first_byte = start_bit / 8;
    const int bits_in_first_byte = (start_bit % 8) + 1;
    const int remaining = bit_length - bits_in_first_byte;
    const auto extra_bytes = remaining <= 0 ? 0ULL :
        (static_cast<unsigned long long>(remaining) + 7) / 8;
    const auto last_byte = static_cast<unsigned long long>(first_byte) + extra_bytes;
    return last_byte < frame_bytes;
}

}  // namespace mqtt_consumer
