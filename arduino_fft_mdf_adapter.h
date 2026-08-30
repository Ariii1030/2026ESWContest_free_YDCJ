#ifndef ARDUINO_FFT_MDF_ADAPTER_H
#define ARDUINO_FFT_MDF_ADAPTER_H

#include <Arduino.h>
#include <arduinoFFT.h>
#include <atomic>
#include <math.h>

#include "fatigue_logic.h"

// arduinoFFT 2.x adapter for fatigue_logic.h 1.2.9.
// Revision 3 shares one workspace across translation units, rejects a
// concurrent call, and supports power-of-two windows through 1024 samples.
#define ARDUINO_FFT_MDF_ADAPTER_REVISION 3

// Supports the existing 256-sample fast window and the project's historical
// 1024-sample, approximately one-second window at 1024 Hz. The two shared
// float work buffers reserve 8192 bytes in total; they are static rather than
// task-stack allocations.
constexpr int ARDUINO_FFT_MDF_MAX_SAMPLES = 1024;

#if !defined(FFT_LIB_REV) || ((FFT_LIB_REV & 0xF0) != 0x20)
#error "arduino_fft_mdf_adapter requires arduinoFFT 2.x"
#endif

namespace arduino_fft_mdf_detail {

// Function-local statics in an external-linkage inline function are shared by
// every translation unit. This avoids the per-.cpp buffers created by a
// namespace-level static variable in the first adapter revision.
inline float* realBuffer() {
  static float buffer[ARDUINO_FFT_MDF_MAX_SAMPLES] = {};
  return buffer;
}

inline float* imagBuffer() {
  static float buffer[ARDUINO_FFT_MDF_MAX_SAMPLES] = {};
  return buffer;
}

inline std::atomic_flag& calculationFlag() {
  static std::atomic_flag flag = ATOMIC_FLAG_INIT;
  return flag;
}

inline bool isPowerOfTwo(int value) {
  return value >= 2 && (value & (value - 1)) == 0;
}

class CalculationGuard {
 public:
  CalculationGuard()
      : flag_(calculationFlag()),
        acquired_(!flag_.test_and_set(std::memory_order_acquire)) {}

  ~CalculationGuard() {
    if (acquired_) flag_.clear(std::memory_order_release);
  }

  bool acquired() const { return acquired_; }

 private:
  std::atomic_flag& flag_;
  bool acquired_;

  CalculationGuard(const CalculationGuard&) = delete;
  CalculationGuard& operator=(const CalculationGuard&) = delete;
};

}  // namespace arduino_fft_mdf_detail

// Exact MDFCalculatorFunction signature used by fatigue_logic.h.
// Invalid input and a concurrent call return NaN, so the project wrapper
// produces valid=false instead of accepting a misleading MDF value.
inline float calculateMDFWithArduinoFFT(const float* samples,
                                        int n,
                                        float sampleRate) {
  using namespace arduino_fft_mdf_detail;

  if (samples == 0 || !isPowerOfTwo(n) ||
      n > ARDUINO_FFT_MDF_MAX_SAMPLES ||
      !isfinite(sampleRate) || sampleRate <= 0.0f) {
    return NAN;
  }

  CalculationGuard guard;
  if (!guard.acquired()) return NAN;

  float* const realValues = realBuffer();
  float* const imagValues = imagBuffer();

  double mean = 0.0;
  for (int i = 0; i < n; ++i) {
    if (!isfinite(samples[i])) return NAN;
    mean += static_cast<double>(samples[i]);
  }
  mean /= static_cast<double>(n);

  double centeredPower = 0.0;
  for (int i = 0; i < n; ++i) {
    const float centered = samples[i] - static_cast<float>(mean);
    realValues[i] = centered;
    imagValues[i] = 0.0f;
    centeredPower +=
        static_cast<double>(centered) * static_cast<double>(centered);
  }
  if (!isfinite(centeredPower) || centeredPower <= 0.0) return NAN;

  ArduinoFFT<float> fft(realValues, imagValues,
                        static_cast<uint_fast16_t>(n), sampleRate);
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward, false);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  const int nyquistBin = n / 2;
  double totalPower = 0.0;
  for (int bin = 1; bin <= nyquistBin; ++bin) {
    const float magnitude = realValues[bin];
    if (!isfinite(magnitude) || magnitude < 0.0f) return NAN;

    double binPower =
        static_cast<double>(magnitude) * static_cast<double>(magnitude);
    if (bin < nyquistBin) binPower *= 2.0;
    totalPower += binPower;
  }
  if (!isfinite(totalPower) || totalPower <= 0.0) return NAN;

  const double halfPower = totalPower * 0.5;
  double cumulativePower = 0.0;
  for (int bin = 1; bin <= nyquistBin; ++bin) {
    const double magnitude = static_cast<double>(realValues[bin]);
    double binPower = magnitude * magnitude;
    if (bin < nyquistBin) binPower *= 2.0;
    cumulativePower += binPower;
    if (cumulativePower >= halfPower) {
      const float frequency =
          static_cast<float>(bin) * sampleRate / static_cast<float>(n);
      return isfinite(frequency) ? frequency : NAN;
    }
  }

  return NAN;
}

inline MDFCalculatorFunction getArduinoFftMdfCalculator() {
  MDFCalculatorFunction calculator = &calculateMDFWithArduinoFFT;
  return calculator;
}

#endif
