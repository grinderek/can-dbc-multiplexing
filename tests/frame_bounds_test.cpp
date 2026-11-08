#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "frame_bounds.hpp"

using mqtt_consumer::signal_fits_frame;

// Illustrative DM1 layout with two lamp bytes followed by up to five
// four-byte diagnostic trouble codes (DTCs) in Intel bit order. A single
// frame can carry one complete DTC and two pad bytes.
namespace {
constexpr bool kIntel = true;
constexpr bool kMotorola = false;
constexpr int kDtcBits = 32;
constexpr int kDtc1 = 16;
constexpr int kDtc2 = 48;
constexpr int kDtc3 = 80;
constexpr int kDtc4 = 112;
constexpr int kDtc5 = 144;
constexpr std::size_t kSingleFrame = 8;
}  // namespace

// ---- Single-frame DM1 padding --------------------------------------------

TEST(FrameBoundsTest, Dtc1FitsSingleFrameDm1) {
    EXPECT_TRUE(signal_fits_frame(kIntel, kDtc1, kDtcBits, kSingleFrame));
}

TEST(FrameBoundsTest, Dtc2DoesNotFitSingleFrameDm1) {
    // bits 48..79 against a 64-bit frame: bytes 6-7 are the 0xFF pad,
    // bytes 8-9 do not exist -> must be skipped, not zero-filled to 65535
    EXPECT_FALSE(signal_fits_frame(kIntel, kDtc2, kDtcBits, kSingleFrame));
}

TEST(FrameBoundsTest, Dtc3To5DoNotFitSingleFrameDm1) {
    EXPECT_FALSE(signal_fits_frame(kIntel, kDtc3, kDtcBits, kSingleFrame));
    EXPECT_FALSE(signal_fits_frame(kIntel, kDtc4, kDtcBits, kSingleFrame));
    EXPECT_FALSE(signal_fits_frame(kIntel, kDtc5, kDtcBits, kSingleFrame));
}

// --- BAM-reassembled frames (buffer trimmed to total_size) -------------------

TEST(FrameBoundsTest, TwoDtcBamFrameFitsExactlyTwoSlots) {
    constexpr std::size_t two_dtc = 2 + 2 * 4;  // 10 bytes
    EXPECT_TRUE(signal_fits_frame(kIntel, kDtc1, kDtcBits, two_dtc));
    EXPECT_TRUE(signal_fits_frame(kIntel, kDtc2, kDtcBits, two_dtc));
    EXPECT_FALSE(signal_fits_frame(kIntel, kDtc3, kDtcBits, two_dtc));
}

TEST(FrameBoundsTest, FiveDtcBamFrameFitsAllSlots) {
    constexpr std::size_t five_dtc = 2 + 5 * 4;  // 22 bytes
    EXPECT_TRUE(signal_fits_frame(kIntel, kDtc5, kDtcBits, five_dtc));
}

// --- intel boundary arithmetic ----------------------------------------------

TEST(FrameBoundsTest, IntelSignalEndingOnLastBitFits) {
    EXPECT_TRUE(signal_fits_frame(kIntel, 32, 32, 8));   // bits 32..63
    EXPECT_TRUE(signal_fits_frame(kIntel, 63, 1, 8));    // last bit only
    EXPECT_TRUE(signal_fits_frame(kIntel, 0, 64, 8));    // whole frame
}

TEST(FrameBoundsTest, IntelSignalOneBitPastEndDoesNotFit) {
    EXPECT_FALSE(signal_fits_frame(kIntel, 33, 32, 8));  // bits 33..64
    EXPECT_FALSE(signal_fits_frame(kIntel, 64, 1, 8));
    EXPECT_FALSE(signal_fits_frame(kIntel, 0, 65, 8));
}

// --- motorola (DBC sawtooth: start_bit is the MSB position) ------------------

TEST(FrameBoundsTest, MotorolaSignalWithinFirstByteFits) {
    // MSB at bit 3 of byte 0, 4 bits: bits 3..0 of byte 0
    EXPECT_TRUE(signal_fits_frame(kMotorola, 3, 4, 1));
    // MSB at bit 7, 8 bits: whole byte 0
    EXPECT_TRUE(signal_fits_frame(kMotorola, 7, 8, 1));
}

TEST(FrameBoundsTest, MotorolaSignalSpillingIntoNextByteNeedsThatByte) {
    // MSB at bit 3 of byte 0, 5 bits: 4 bits from byte 0, 1 bit from byte 1
    EXPECT_FALSE(signal_fits_frame(kMotorola, 3, 5, 1));
    EXPECT_TRUE(signal_fits_frame(kMotorola, 3, 5, 2));
}

TEST(FrameBoundsTest, MotorolaSixteenBitSignalInLastTwoBytesFits) {
    // MSB at bit 7 of byte 6 (sawtooth 55), 16 bits: bytes 6..7
    EXPECT_TRUE(signal_fits_frame(kMotorola, 55, 16, 8));
    EXPECT_FALSE(signal_fits_frame(kMotorola, 55, 24, 8));  // would need byte 8
    EXPECT_FALSE(signal_fits_frame(kMotorola, 55, 16, 7));
}

TEST(FrameBoundsTest, MotorolaByteAlignedMsbUsesExactlyOneBitOfItsByte) {
    // MSB at bit 0 of a byte (start_bit % 8 == 0): the first byte contributes
    // exactly ONE bit, so a 1-bit flag `24|1@0+` lives entirely in byte 3
    EXPECT_TRUE(signal_fits_frame(kMotorola, 24, 1, 4));
    EXPECT_FALSE(signal_fits_frame(kMotorola, 24, 1, 3));
    // ...and anything longer spills into the NEXT byte immediately
    EXPECT_FALSE(signal_fits_frame(kMotorola, 0, 2, 1));
    EXPECT_TRUE(signal_fits_frame(kMotorola, 0, 2, 2));
    // 1 bit from byte 2, then 8 more bits = byte 3
    EXPECT_FALSE(signal_fits_frame(kMotorola, 16, 9, 3));
    EXPECT_TRUE(signal_fits_frame(kMotorola, 16, 9, 4));
}

TEST(FrameBoundsTest, MotorolaStartBitPastFrameDoesNotFit) {
    EXPECT_FALSE(signal_fits_frame(kMotorola, 71, 8, 8));  // byte 8
}

// --- degenerate inputs --------------------------------------------------------

TEST(FrameBoundsTest, ExtremeInputsDoNotOverflow) {
    const auto max_width = std::numeric_limits<int>::max();
    const auto max_size = std::numeric_limits<std::size_t>::max();
    EXPECT_FALSE(signal_fits_frame(kIntel, 0, max_width, 8));
    EXPECT_FALSE(signal_fits_frame(kMotorola, 0, max_width, 8));
    EXPECT_TRUE(signal_fits_frame(kIntel, 0, 1, max_size));
    EXPECT_TRUE(signal_fits_frame(kMotorola, 0, 1, max_size));
}

TEST(FrameBoundsTest, EmptyFrameFitsNothing) {
    EXPECT_FALSE(signal_fits_frame(kIntel, 0, 1, 0));
    EXPECT_FALSE(signal_fits_frame(kMotorola, 7, 1, 0));
}

TEST(FrameBoundsTest, NonPositiveLengthOrNegativeStartNeverFits) {
    EXPECT_FALSE(signal_fits_frame(kIntel, 0, 0, 8));
    EXPECT_FALSE(signal_fits_frame(kIntel, 0, -1, 8));
    EXPECT_FALSE(signal_fits_frame(kIntel, -1, 8, 8));
    EXPECT_FALSE(signal_fits_frame(kMotorola, -1, 8, 8));
}
