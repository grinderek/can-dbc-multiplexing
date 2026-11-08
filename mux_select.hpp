// Dependency-free helpers for selecting the active page of a multiplexed
// CAN message. A child refers to its selector by ID. Its parse plan can carry
// the selector layout even when the selector is absent from that plan.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "frame_bounds.hpp"

namespace mqtt_consumer {

// Bit layout of a multiplexer selector signal.
struct MuxSelector {
    std::int64_t param_id = 0;
    std::string  byte_order;  // "intel"/"little"/... or "motorola"/anything else
    int          start_bit = 0;
    int          bit_length = 0;
};

// Fill `pd.mux_selector` for every multiplexed child of one identifier's
// parse plan. Duck-typed on ParamT: needs `param_id`, `byte_order`,
// `start_bit`, `bit_length`, `multiplexer_parameter_id`
// (std::optional<int64>) and `mux_selector` (std::optional<MuxSelector>).
//
// A carried layout is used when it matches the child's current selector ID.
// Otherwise an entry with that ID in the plan supplies the layout. An entry
// without a selector reference has any stale layout cleared.
template <class ParamT>
inline void resolve_mux_selectors(std::vector<ParamT>& params) {
    for (auto& pd : params) {
        if (!pd.multiplexer_parameter_id.has_value()) {
            pd.mux_selector.reset();
            continue;
        }
        if (pd.mux_selector.has_value()
            && pd.mux_selector->param_id == *pd.multiplexer_parameter_id
            && pd.mux_selector->bit_length > 0) {
            continue;
        }
        pd.mux_selector.reset();
        const std::int64_t wanted = *pd.multiplexer_parameter_id;
        for (const auto& cand : params) {
            if (cand.param_id == wanted && cand.bit_length > 0) {
                pd.mux_selector = MuxSelector{cand.param_id, cand.byte_order,
                                              cand.start_bit, cand.bit_length};
                break;
            }
        }
    }
}

// All-ones raw for a given width (J1939 "not available" on the selector).
inline std::uint64_t mux_all_ones(int bits) {
    if (bits <= 0) return 0ULL;
    if (bits >= 64) return ~0ULL;
    return (1ULL << bits) - 1ULL;
}

// Per-frame memo of decoded selector values keyed by selector param_id, so a
// frame whose pages reference the same selector reads it once. One instance
// per CAN frame; do not reuse across frames.
class MuxFrame {
public:
    // `read(byte_order, start_bit, bit_length) -> std::uint64_t` extracts raw
    // bits from the frame payload; `frame_bytes` is the payload length.
    // The reader receives selectors between 1 and 64 bits wide that fit the
    // supplied payload. An unreadable selector or an all-ones value has no
    // active page.
    template <class ReadBits>
    std::optional<std::int64_t> raw_for(const MuxSelector& sel, std::size_t frame_bytes,
                                        ReadBits&& read) {
        auto it = memo_.find(sel.param_id);
        if (it != memo_.end()) return it->second;

        std::optional<std::int64_t> out;
        if (sel.bit_length <= 64 &&
            signal_fits_frame(is_intel_byte_order(sel.byte_order),
                              sel.start_bit, sel.bit_length, frame_bytes)) {
            const std::uint64_t raw = read(sel.byte_order, sel.start_bit, sel.bit_length);
            if (raw != mux_all_ones(sel.bit_length)) {
                out = static_cast<std::int64_t>(raw);
            }
        }
        memo_.emplace(sel.param_id, out);
        return out;
    }

private:
    std::unordered_map<std::int64_t, std::optional<std::int64_t>> memo_;
};

// A multiplexed child decodes only when the selector was readable AND its
// raw value equals the child's declared page.
inline bool mux_child_active(const std::optional<std::int64_t>& decoded_mux,
                             const std::optional<std::int64_t>& wanted_page) {
    return decoded_mux.has_value() && wanted_page.has_value()
        && *decoded_mux == *wanted_page;
}

}  // namespace mqtt_consumer
