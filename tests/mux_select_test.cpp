// Selector resolution and page selection for synthetic CAN messages.

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mux_select.hpp"

namespace {

using mqtt_consumer::MuxFrame;
using mqtt_consumer::MuxSelector;
using mqtt_consumer::mux_child_active;
using mqtt_consumer::resolve_mux_selectors;

// Minimal signal definition for the test parse plans.
struct Param {
    std::int64_t param_id = 0;
    std::string byte_order = "intel";
    int start_bit = 0;
    int bit_length = 0;
    bool is_multiplexer = false;
    std::optional<std::int64_t> multiplexer_parameter_id = std::nullopt;
    std::optional<std::int64_t> multiplexer_value = std::nullopt;
    std::optional<MuxSelector> mux_selector = std::nullopt;
};

// Little-endian reader used by the test callbacks.
std::uint64_t read_intel(const std::vector<std::uint8_t>& bytes, int start_bit, int bit_len) {
    std::uint64_t val = 0;
    for (int b = 0; b < bit_len; ++b) {
        const int idx = start_bit + b;
        const int byte_idx = idx / 8;
        if (byte_idx >= 0 && static_cast<std::size_t>(byte_idx) < bytes.size()) {
            val |= static_cast<std::uint64_t>((bytes[byte_idx] >> (idx % 8)) & 1u) << b;
        }
    }
    return val;
}

auto reader_for(const std::vector<std::uint8_t>& bytes) {
    return [&bytes](const std::string& /*byte_order*/, int start_bit, int bit_len) {
        return read_intel(bytes, start_bit, bit_len);
    };
}

// ---- resolve_mux_selectors ------------------------------------------------

TEST(ResolveMuxSelectors, CarriedLayoutWinsEvenWhenSelectorNotSubscribed) {
    // The plan contains only the child. Keep its matching selector layout.
    std::vector<Param> plan{
        Param{.param_id = 102, .start_bit = 8, .bit_length = 24,
              .multiplexer_parameter_id = 101, .multiplexer_value = 1,
              .mux_selector = MuxSelector{101, "intel", 0, 8}},
    };
    resolve_mux_selectors(plan);
    ASSERT_TRUE(plan[0].mux_selector.has_value());
    EXPECT_EQ(plan[0].mux_selector->param_id, 101);
    EXPECT_EQ(plan[0].mux_selector->start_bit, 0);
    EXPECT_EQ(plan[0].mux_selector->bit_length, 8);
}

TEST(ResolveMuxSelectors, FallsBackToSubscribedParamById_IgnoringIsMultiplexerFlag) {
    // The plan supplies the selector by ID even after its flag changes.
    std::vector<Param> plan{
        Param{.param_id = 201, .start_bit = 0, .bit_length = 8, .is_multiplexer = false},
        Param{.param_id = 202, .start_bit = 8, .bit_length = 24,
              .multiplexer_parameter_id = 201, .multiplexer_value = 2},
    };
    resolve_mux_selectors(plan);
    ASSERT_TRUE(plan[1].mux_selector.has_value());
    EXPECT_EQ(plan[1].mux_selector->param_id, 201);
    EXPECT_EQ(plan[1].mux_selector->bit_length, 8);
    EXPECT_FALSE(plan[0].mux_selector.has_value()) << "selector itself is not multiplexed";
}

TEST(ResolveMuxSelectors, TwoSelectorGenerationsCoexistIndependently) {
    // Old-generation child gated by 201, new-generation child gated by 101,
    // both on the same identifier. Each must resolve to its own selector.
    std::vector<Param> plan{
        Param{.param_id = 201, .start_bit = 0, .bit_length = 8},
        Param{.param_id = 202, .start_bit = 8, .bit_length = 24,
              .multiplexer_parameter_id = 201, .multiplexer_value = 2},
        Param{.param_id = 103, .start_bit = 8, .bit_length = 24,
              .multiplexer_parameter_id = 101, .multiplexer_value = 2,
              .mux_selector = MuxSelector{101, "intel", 0, 8}},
    };
    resolve_mux_selectors(plan);
    ASSERT_TRUE(plan[1].mux_selector.has_value());
    EXPECT_EQ(plan[1].mux_selector->param_id, 201);
    ASSERT_TRUE(plan[2].mux_selector.has_value());
    EXPECT_EQ(plan[2].mux_selector->param_id, 101);
}

TEST(ResolveMuxSelectors, DanglingReferenceStaysUnresolved) {
    std::vector<Param> plan{
        Param{.param_id = 1, .start_bit = 8, .bit_length = 8,
              .multiplexer_parameter_id = 999, .multiplexer_value = 0},
    };
    resolve_mux_selectors(plan);
    EXPECT_FALSE(plan[0].mux_selector.has_value());
}

TEST(ResolveMuxSelectors, NonMultiplexedParamNeverGetsSelector) {
    std::vector<Param> plan{
        Param{.param_id = 1, .start_bit = 0, .bit_length = 8,
              .mux_selector = MuxSelector{5, "intel", 0, 8}},  // stale garbage
    };
    resolve_mux_selectors(plan);
    EXPECT_FALSE(plan[0].mux_selector.has_value());
}

TEST(ResolveMuxSelectors, CarriedLayoutForDifferentSelectorIsReplaced) {
    // Child was re-pointed to selector 101 but still carries 201's layout
    // (stale): the carried layout must not be trusted.
    std::vector<Param> plan{
        Param{.param_id = 101, .start_bit = 0, .bit_length = 8},
        Param{.param_id = 202, .start_bit = 8, .bit_length = 24,
              .multiplexer_parameter_id = 101, .multiplexer_value = 2,
              .mux_selector = MuxSelector{201, "motorola", 7, 8}},
    };
    resolve_mux_selectors(plan);
    ASSERT_TRUE(plan[1].mux_selector.has_value());
    EXPECT_EQ(plan[1].mux_selector->param_id, 101);
    EXPECT_EQ(plan[1].mux_selector->byte_order, "intel");
    EXPECT_EQ(plan[1].mux_selector->start_bit, 0);
}

TEST(ResolveMuxSelectors, StaleCarriedLayoutWithNoFallbackIsDropped) {
    std::vector<Param> plan{
        Param{.param_id = 202, .start_bit = 8, .bit_length = 24,
              .multiplexer_parameter_id = 101, .multiplexer_value = 2,
              .mux_selector = MuxSelector{201, "intel", 0, 8}},
    };
    resolve_mux_selectors(plan);
    EXPECT_FALSE(plan[0].mux_selector.has_value());
}

TEST(ResolveMuxSelectors, ZeroLengthCarriedLayoutIsReplacedByFallback) {
    std::vector<Param> plan{
        Param{.param_id = 10, .start_bit = 0, .bit_length = 8},
        Param{.param_id = 11, .start_bit = 8, .bit_length = 8,
              .multiplexer_parameter_id = 10, .multiplexer_value = 0,
              .mux_selector = MuxSelector{10, "intel", 0, 0}},
    };
    resolve_mux_selectors(plan);
    ASSERT_TRUE(plan[1].mux_selector.has_value());
    EXPECT_EQ(plan[1].mux_selector->bit_length, 8);
}

// ---- MuxFrame::raw_for + mux_child_active ---------------------------------

TEST(MuxFrame, DecodesPageZeroFromSyntheticFrame) {
    const std::vector<std::uint8_t> bytes{0x00, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE};
    MuxFrame frame;
    const MuxSelector sel{101, "intel", 0, 8};
    const auto raw = frame.raw_for(sel, bytes.size(), reader_for(bytes));
    ASSERT_TRUE(raw.has_value());
    EXPECT_EQ(*raw, 0);
    EXPECT_TRUE(mux_child_active(raw, 0));
    EXPECT_FALSE(mux_child_active(raw, 1));
}

TEST(MuxFrame, DecodesNonzeroPageFromSyntheticFrame) {
    const std::vector<std::uint8_t> bytes{0x2A, 0x87, 0x65, 0x43, 0x21, 0x10, 0x24, 0x68};
    MuxFrame frame;
    const auto raw = frame.raw_for(MuxSelector{301, "intel", 0, 8}, bytes.size(), reader_for(bytes));
    ASSERT_TRUE(raw.has_value());
    EXPECT_EQ(*raw, 42);
    EXPECT_TRUE(mux_child_active(raw, 42));
    EXPECT_FALSE(mux_child_active(raw, 43));
}

TEST(MuxFrame, AllOnesSelectorActivatesNoPage) {
    const std::vector<std::uint8_t> bytes{0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    MuxFrame frame;
    const auto raw = frame.raw_for(MuxSelector{1, "intel", 0, 8}, bytes.size(), reader_for(bytes));
    EXPECT_FALSE(raw.has_value());
    EXPECT_FALSE(mux_child_active(raw, 255));
    EXPECT_FALSE(mux_child_active(raw, 0));
}

TEST(MuxFrame, UnsupportedSelectorWidthsNeverReachReader) {
    // A 65-bit selector fits in nine bytes but exceeds the reader's return type.
    int reads = 0;
    const auto reader = [&reads](const std::string&, int, int) -> std::uint64_t {
        ++reads;
        return 5;
    };
    MuxFrame frame;
    EXPECT_FALSE(frame.raw_for(MuxSelector{1, "intel", 0, 0}, 9, reader).has_value());
    EXPECT_FALSE(frame.raw_for(MuxSelector{2, "intel", 0, 65}, 9, reader).has_value());
    EXPECT_EQ(reads, 0);
}

TEST(MuxFrame, MemoisesPerSelectorAndReadsEachOnce) {
    const std::vector<std::uint8_t> bytes{0x03, 0x10, 0, 0, 0, 0, 0, 0};
    int reads = 0;
    auto counting = [&](const std::string&, int start_bit, int bit_len) {
        ++reads;
        return read_intel(bytes, start_bit, bit_len);
    };
    MuxFrame frame;
    const MuxSelector a{1, "intel", 0, 8};
    const MuxSelector b{2, "intel", 8, 8};
    EXPECT_EQ(frame.raw_for(a, bytes.size(), counting), std::optional<std::int64_t>{3});
    EXPECT_EQ(frame.raw_for(a, bytes.size(), counting), std::optional<std::int64_t>{3});
    EXPECT_EQ(frame.raw_for(b, bytes.size(), counting), std::optional<std::int64_t>{16});
    EXPECT_EQ(frame.raw_for(b, bytes.size(), counting), std::optional<std::int64_t>{16});
    EXPECT_EQ(reads, 2);
}

TEST(MuxFrame, SelectorPastFrameEndIsUnreadable) {
    // Selector declared in byte 1 (bits 8..15) but the device sent a 1-byte
    // frame: the reader would zero-fill and return raw 0, which would activate
    // page 0. The selector must instead count as unreadable (no page active).
    const std::vector<std::uint8_t> bytes{0x02};
    MuxFrame frame;
    const MuxSelector sel{1, "intel", 8, 8};
    const auto raw = frame.raw_for(sel, bytes.size(), reader_for(bytes));
    EXPECT_FALSE(raw.has_value());
    EXPECT_FALSE(mux_child_active(raw, 0));
}

TEST(MuxFrame, SelectorInsideFrameStillDecodes) {
    const std::vector<std::uint8_t> bytes{0x02, 0x07};
    MuxFrame frame;
    const auto raw = frame.raw_for(MuxSelector{1, "intel", 8, 8}, bytes.size(), reader_for(bytes));
    ASSERT_TRUE(raw.has_value());
    EXPECT_EQ(*raw, 7);
}

TEST(MuxChildActive, RequiresBothSidesAndEquality) {
    EXPECT_FALSE(mux_child_active(std::nullopt, 1));
    EXPECT_FALSE(mux_child_active(1, std::nullopt));
    EXPECT_FALSE(mux_child_active(std::nullopt, std::nullopt));
    EXPECT_FALSE(mux_child_active(1, 2));
    EXPECT_TRUE(mux_child_active(17, 17));
}

}  // namespace
