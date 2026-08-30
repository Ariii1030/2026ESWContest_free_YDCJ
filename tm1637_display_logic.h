#ifndef TM1637_DISPLAY_LOGIC_H
#define TM1637_DISPLAY_LOGIC_H

#include <stdint.h>

// Pure TM1637 frame logic. No Arduino, GPIO, delay(), millis(), or display
// library calls are allowed in this header.

enum Tm1637Glyph : uint8_t {
  TM_GLYPH_0 = 0,
  TM_GLYPH_1,
  TM_GLYPH_2,
  TM_GLYPH_3,
  TM_GLYPH_4,
  TM_GLYPH_5,
  TM_GLYPH_6,
  TM_GLYPH_7,
  TM_GLYPH_8,
  TM_GLYPH_9,
  TM_GLYPH_A,
  TM_GLYPH_C,
  TM_GLYPH_E,
  TM_GLYPH_L,
  TM_GLYPH_N,
  TM_GLYPH_O,
  TM_GLYPH_R,
  TM_GLYPH_T,
  TM_GLYPH_DASH,
  TM_GLYPH_BLANK
};

enum Tm1637Segment : uint8_t {
  TM_SEG_A = 0x01U,
  TM_SEG_B = 0x02U,
  TM_SEG_C = 0x04U,
  TM_SEG_D = 0x08U,
  TM_SEG_E = 0x10U,
  TM_SEG_F = 0x20U,
  TM_SEG_G = 0x40U,
  TM_SEG_DP = 0x80U
};

struct Tm1637LogicalFrame {
  Tm1637Glyph glyphs[4] = {TM_GLYPH_BLANK, TM_GLYPH_BLANK,
                            TM_GLYPH_BLANK, TM_GLYPH_BLANK};
  // Bit 0 controls digit 0's decimal point, bit 1 digit 1, and so on.
  uint8_t decimalPointMask = 0U;
};

struct Tm1637DisplayConfig {
  uint8_t brightness = 4U;  // TM1637Display library range: 0..7.
  unsigned long blinkIntervalMs = 500UL;
};

inline bool isTm1637DisplayConfigValid(
    const Tm1637DisplayConfig& config) {
  return config.brightness <= 7U && config.blinkIntervalMs > 0UL;
}

inline uint8_t encodeTm1637Glyph(Tm1637Glyph glyph) {
  static const uint8_t digits[10] = {
      TM_SEG_A | TM_SEG_B | TM_SEG_C | TM_SEG_D | TM_SEG_E | TM_SEG_F,
      TM_SEG_B | TM_SEG_C,
      TM_SEG_A | TM_SEG_B | TM_SEG_D | TM_SEG_E | TM_SEG_G,
      TM_SEG_A | TM_SEG_B | TM_SEG_C | TM_SEG_D | TM_SEG_G,
      TM_SEG_B | TM_SEG_C | TM_SEG_F | TM_SEG_G,
      TM_SEG_A | TM_SEG_C | TM_SEG_D | TM_SEG_F | TM_SEG_G,
      TM_SEG_A | TM_SEG_C | TM_SEG_D | TM_SEG_E | TM_SEG_F | TM_SEG_G,
      TM_SEG_A | TM_SEG_B | TM_SEG_C,
      TM_SEG_A | TM_SEG_B | TM_SEG_C | TM_SEG_D | TM_SEG_E | TM_SEG_F |
          TM_SEG_G,
      TM_SEG_A | TM_SEG_B | TM_SEG_C | TM_SEG_D | TM_SEG_F | TM_SEG_G};
  if (glyph >= TM_GLYPH_0 && glyph <= TM_GLYPH_9) {
    return digits[static_cast<uint8_t>(glyph)];
  }
  switch (glyph) {
    case TM_GLYPH_A:
      return TM_SEG_A | TM_SEG_B | TM_SEG_C | TM_SEG_E | TM_SEG_F |
             TM_SEG_G;
    case TM_GLYPH_C:
      return TM_SEG_A | TM_SEG_D | TM_SEG_E | TM_SEG_F;
    case TM_GLYPH_E:
      return TM_SEG_A | TM_SEG_D | TM_SEG_E | TM_SEG_F | TM_SEG_G;
    case TM_GLYPH_L:
      return TM_SEG_D | TM_SEG_E | TM_SEG_F;
    case TM_GLYPH_N:
      return TM_SEG_C | TM_SEG_E | TM_SEG_G;
    case TM_GLYPH_O:
      return TM_SEG_C | TM_SEG_D | TM_SEG_E | TM_SEG_G;
    case TM_GLYPH_R:
      return TM_SEG_E | TM_SEG_G;
    case TM_GLYPH_T:
      return TM_SEG_D | TM_SEG_E | TM_SEG_F | TM_SEG_G;
    case TM_GLYPH_DASH:
      return TM_SEG_G;
    case TM_GLYPH_BLANK:
    default:
      return 0U;
  }
}

inline void encodeTm1637Frame(const Tm1637LogicalFrame& frame,
                              uint8_t segments[4]) {
  if (segments == 0) return;
  for (uint8_t i = 0U; i < 4U; ++i) {
    segments[i] = encodeTm1637Glyph(frame.glyphs[i]);
    if ((frame.decimalPointMask & (1U << i)) != 0U) {
      segments[i] |= TM_SEG_DP;
    }
  }
}

struct Tm1637DisplayController {
  bool blinkTracking = false;
  bool blinkVisible = true;
  unsigned long blinkStartMs = 0UL;
  bool hasLastOutput = false;
  bool lastEnabled = false;
  uint8_t lastBrightness = 0U;
  uint8_t lastSegments[4] = {0U, 0U, 0U, 0U};
};

struct Tm1637DisplayOutput {
  bool enabled = false;
  bool configValid = false;
  bool changed = false;
  uint8_t brightness = 0U;
  uint8_t segments[4] = {0U, 0U, 0U, 0U};
};

inline bool areTm1637SegmentsEqual(const uint8_t left[4],
                                   const uint8_t right[4]) {
  if (left == 0 || right == 0) return false;
  for (uint8_t i = 0U; i < 4U; ++i) {
    if (left[i] != right[i]) return false;
  }
  return true;
}

inline Tm1637DisplayOutput updateTm1637Display(
    Tm1637DisplayController& controller,
    const Tm1637LogicalFrame& frame,
    bool displayEnabled,
    bool blinkRequested,
    unsigned long nowMs,
    const Tm1637DisplayConfig& config) {
  Tm1637DisplayOutput output;
  output.configValid = isTm1637DisplayConfigValid(config);
  output.brightness = output.configValid ? config.brightness : 0U;

  if (!output.configValid || !displayEnabled) {
    controller.blinkTracking = false;
    controller.blinkVisible = true;
  } else if (!blinkRequested) {
    controller.blinkTracking = false;
    controller.blinkVisible = true;
  } else if (!controller.blinkTracking) {
    controller.blinkTracking = true;
    controller.blinkVisible = true;
    controller.blinkStartMs = nowMs;
  } else {
    const unsigned long elapsed = nowMs - controller.blinkStartMs;
    const unsigned long intervals = elapsed / config.blinkIntervalMs;
    if (intervals > 0UL) {
      if ((intervals & 1UL) != 0UL) {
        controller.blinkVisible = !controller.blinkVisible;
      }
      controller.blinkStartMs += intervals * config.blinkIntervalMs;
    }
  }

  output.enabled = output.configValid && displayEnabled;
  if (output.enabled && controller.blinkVisible) {
    encodeTm1637Frame(frame, output.segments);
  }

  output.changed = !controller.hasLastOutput ||
                   output.enabled != controller.lastEnabled ||
                   output.brightness != controller.lastBrightness ||
                   !areTm1637SegmentsEqual(output.segments,
                                           controller.lastSegments);

  controller.hasLastOutput = true;
  controller.lastEnabled = output.enabled;
  controller.lastBrightness = output.brightness;
  for (uint8_t i = 0U; i < 4U; ++i) {
    controller.lastSegments[i] = output.segments[i];
  }
  return output;
}

#endif  // TM1637_DISPLAY_LOGIC_H
