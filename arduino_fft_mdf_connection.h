#ifndef ARDUINO_FFT_MDF_CONNECTION_H
#define ARDUINO_FFT_MDF_CONNECTION_H

#include "arduino_fft_mdf_adapter.h"

// Reusable connection component. It deliberately has no setup() or loop(), so
// it can be added to the final IMU/RMS sketch without duplicate definitions.
//
// 두 가지 경로를 의도적으로 구분한다.
//   1) process(...): 다른 StaticSegmentGate가 없는 단독 구성에서만 사용한다.
//      이 컴포넌트가 IMU 디바운스와 최소 정지시간을 직접 판정한다.
//   2) processCombined(...): 결합 스케치가 이미 RMS와 IMU 게이트를 소유할
//      때 사용한다. 두 조건의 AND와 FFT 분석주기 제한을 이곳에서 처리한다.
// processEligible()은 호환성을 위해 유지하지만, 전달 bool은 정지조건만이
// 아니라 최종 결합값(isContracting && isStaticEligible)이어야 한다.
class ArduinoFftMdfConnection {
 public:
  ArduinoFftMdfConnection()
      : processor_(), eligibilityProcessor_(), config_(),
        calculator_(getArduinoFftMdfCalculator()) {
    config_.segment.minStaticDurationMs = 5000UL;
    config_.segment.motionDebounceMs = 300UL;
    config_.analysisIntervalMs = 5000UL;
  }

  void begin() { reset(); }

  void configure(unsigned long minStaticDurationMs,
                 unsigned long motionDebounceMs,
                 unsigned long analysisIntervalMs) {
    config_.segment.minStaticDurationMs = minStaticDurationMs;
    config_.segment.motionDebounceMs = motionDebounceMs;
    config_.analysisIntervalMs = analysisIntervalMs;
    reset();
  }

  void reset() {
    resetStaticMDFProcessor(processor_);
    resetEligibilityGatedMDFProcessor(eligibilityProcessor_);
  }

  // [자체 게이트 방식] 정지판정을 이 클래스가 직접 한다. combined
  // 스케치처럼 다른 곳에 이미 같은 판정이 있는 상황에서는 쓰지 말 것
  // (위 클래스 설명 참고).
  StaticMDFResult process(bool isImuStaticNow,
                          unsigned long nowMs,
                          const float* emgSamples,
                          int sampleCount,
                          float sampleRateHz) {
    return processStaticMDF(
        processor_, isImuStaticNow, nowMs, emgSamples, sampleCount,
        sampleRateHz, calculator_, config_);
  }

  // Preferred hardware path. A communication, data, or configuration error
  // is not ordinary motion noise: discard the complete static history so the
  // recovered IMU must establish a fresh debounce + minimum-static interval.
  // [자체 게이트 방식] 위 process()와 마찬가지로 자체 게이트를 쓴다.
  StaticMDFResult process(const ImuStaticEvaluation& imuEvaluation,
                          unsigned long nowMs,
                          const float* emgSamples,
                          int sampleCount,
                          float sampleRateHz) {
    if (imuEvaluation.status != IMU_OK) {
      reset();
      return StaticMDFResult();
    }
    return process(imuEvaluation.isStaticNow, nowMs, emgSamples,
                   sampleCount, sampleRateHz);
  }

  // 호환 경로: isMdfEligible은 이미 RMS 수축과 IMU 정지를 AND한 값이다.
  StaticMDFResult processEligible(bool isMdfEligible,
                                  unsigned long nowMs,
                                  const float* emgSamples,
                                  int sampleCount,
                                  float sampleRateHz) {
    return processMdfForEligibleSegment(
        eligibilityProcessor_, isMdfEligible, nowMs, emgSamples,
        sampleCount, sampleRateHz, calculator_, config_.analysisIntervalMs);
  }

  // 결합 스케치의 권장 경로. AND 연산을 컴포넌트 내부에 두어 비수축
  // 정지 구간에서 호출자가 실수로 FFT를 실행하는 것을 막는다.
  StaticMDFResult processCombined(bool isContracting,
                                  bool isStaticEligible,
                                  unsigned long nowMs,
                                  const float* emgSamples,
                                  int sampleCount,
                                  float sampleRateHz) {
    const bool isMdfEligible =
        isMdfEligibleForFatigueAnalysis(isContracting, isStaticEligible);
    return processEligible(isMdfEligible, nowMs, emgSamples, sampleCount,
                           sampleRateHz);
  }

  // 보드 통합 권장 overload. isStaticEligible은 이번 창의 MDF 사용 가능
  // 여부이고, isStaticSegmentConfirmed는 StaticSegmentGate::isStatic의
  // 스냅샷입니다. 300ms 미만 원시 움직임에서는 전자는 false, 후자는 true라
  // 오염 창만 차단하고 기존 5초 분석주기 타이머는 보존합니다. 수축 종료,
  // 확정 움직임, IMU 오류에서는 타이머를 초기화합니다.
  StaticMDFResult processCombined(bool isContracting,
                                  bool isStaticEligible,
                                  bool isStaticSegmentConfirmed,
                                  unsigned long nowMs,
                                  const float* emgSamples,
                                  int sampleCount,
                                  float sampleRateHz) {
    const bool isMdfEligible = isMdfEligibleForFatigueAnalysis(
        isContracting, isStaticEligible && isStaticSegmentConfirmed);
    const bool resetAnalysisStateWhenIneligible =
        !isContracting || !isStaticSegmentConfirmed;
    return processMdfForEligibleSegment(
        eligibilityProcessor_, isMdfEligible,
        resetAnalysisStateWhenIneligible, nowMs, emgSamples, sampleCount,
        sampleRateHz, calculator_, config_.analysisIntervalMs);
  }

  const StaticMDFProcessingConfig& config() const { return config_; }

 private:
  StaticMDFProcessor processor_;
  EligibilityGatedMDFProcessor eligibilityProcessor_;
  StaticMDFProcessingConfig config_;
  MDFCalculatorFunction calculator_;
};

#endif
