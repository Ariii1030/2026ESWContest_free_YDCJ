/*
  ===================================================================
  fatigue_logic.h
  ===================================================================
  근피로 감지·이완 밴드 - "진짜" 로직 코드만 모아놓은 파일.

  이 파일 안에는 가짜 데이터, 테스트, 출력(printf) 코드가 전혀 없음.
  순수하게 "입력을 받아서 계산/판단만 하는 함수들"만 있음.

  -> 지금은 test_fatigue_logic.cpp 에서 가짜 데이터를 넣어 테스트하고,
  -> 나중에 하드웨어(ESP32) 완성되면 Arduino .ino 파일에서
     #include "fatigue_logic.h" 로 그대로 불러와서
     analogRead(), DS18B20 값, 실제 millis() 등 "진짜 데이터"를 넣어주면
     이 파일은 단 한 줄도 안 고쳐도 됨.
  ===================================================================
*/

#ifndef FATIGUE_LOGIC_H
#define FATIGUE_LOGIC_H

#define FATIGUE_LOGIC_VERSION_MAJOR 1
#define FATIGUE_LOGIC_VERSION_MINOR 2
#define FATIGUE_LOGIC_VERSION_PATCH 9

// ---------------------------------------------------------------
// v1.2.9 변경사항 (보드 가정 FFT/MDF 끝단 시뮬레이션)
// ---------------------------------------------------------------
// - 300ms 미만의 짧은 원시 움직임은 해당 창의 MDF만 차단하고, 확정 정지
//   구간이 유지되는 동안에는 5초 분석주기 타이머를 보존할 수 있는 overload
//   추가. 기존 API는 호환성을 위해 그대로 유지됩니다.
//
// ---------------------------------------------------------------
// v1.2.8 변경사항 (arduino_fft_mdf_* 연결 교차검증)
// ---------------------------------------------------------------
// - processMdfForEligibleSegment() / EligibilityGatedMDFProcessor 신규
// - 최종 MDF 적격값(isContracting && isStaticEligible)을 직접 받아 FFT가
//   비수축 구간에서 실행되는 호출자 실수를 차단할 수 있게 함.
//
// v1.2.7: 캘리브레이션 경계·수치 안정성 수정
// - 표준편차 0인 양수 베이스라인과 음수 minMadRatio 설정을 fail-safe 거부
// - 품질 통계를 double/표본 표준편차로 계산하고 정렬 인덱스를 명시적으로 보호
//
// ---------------------------------------------------------------
// v1.2.6 변경사항 (imu_rms_combined_gate 리뷰에서 발견된 문제 수정)
// ---------------------------------------------------------------
// - checkCalibQuality() 신규 추가: RMS 베이스라인 캘리브레이션 구간에서
//   모은 블록별 값들의 이상치/변동계수를 검사해 접촉불량·순간노이즈로
//   오염된 베이스라인을 걸러냄 (기존 mdf_realtime_plot 계열 .ino에 있던
//   같은 이름의 함수를 헤더로 이식, 로직은 동일)
// - 그 외 IMU 정지판정 임계값 미설정, 자이로 바이어스 보정 누락 문제는
//   이 헤더가 아니라 .ino(어댑터) 쪽 문제라 통합 스케치에서 수정함

// ---------------------------------------------------------------
// 실측 후 "값만" 조정하면 되는 항목 (코드 수정 아님) - TODO 목록
// ---------------------------------------------------------------
// - ImuStaticDetectionConfig::accelStaticDeviationG / gyroStaticThresholdDps
//   (실제 IMU 노이즈 특성을 보고 정지/동작 임계값 조정)
// - StaticSegmentConfig::motionDebounceMs / minStaticDurationMs
// - RmsContractionConfig::startStdMult / endStdMult / debounceBlocks
// 위 세 그룹 모두 .ino에서 config 구조체 값만 바꾸면 되고, 이 헤더의
// 함수 로직 자체는 손댈 필요가 없습니다.

#include <math.h>
#include <stddef.h>
// 주의: <cmath> + std:: 대신 <math.h>의 전역 함수(sqrt, cos, sin, round)를 씁니다.
// avr-gcc 기반 아두이노(Uno 등)는 <cmath>의 std:: 네임스페이스 함수 일부를 지원하지 않아
// ESP32에서는 되던 코드가 다른 보드에서는 컴파일 에러가 날 수 있기 때문입니다.
// math.h의 전역 함수는 AVR/ESP32/SAMD 등 모든 아두이노 계열 보드에서 공통으로 동작합니다.

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------
// STEP 1. RMS 계산
// ---------------------------------------------------------------
inline float calculateRMS(const float* samples, int n) {
  if (samples == 0 || n <= 0) return 0.0f;

  float sumSq = 0.0f;
  for (int i = 0; i < n; i++) {
    if (!isfinite(samples[i])) return 0.0f;
    sumSq += samples[i] * samples[i];
  }
  if (!isfinite(sumSq)) return 0.0f;
  return sqrtf(sumSq / n);
}

// ---------------------------------------------------------------
// STEP 2. 중앙주파수(MDF) 계산 (간이 DFT 기반)
// ---------------------------------------------------------------
// 전체 0~Nyquist 대역을 계산합니다. 메모리를 아끼기 위해 Parseval 관계로
// 전체 전력을 구하고 주파수 bin을 순서대로 계산합니다.
//
// 주의: 이 구현은 O(n^2) 브루트포스 DFT라서 SAMPLES=1024 기준으로 실시간
// 처리에 못 씁니다. 테스트(test_fatigue_logic.cpp) 검증용으로만 의도된
// 코드이며, 실제 .ino로 옮길 때는 이 함수를 그대로 쓰지 말고 arduinoFFT 같은
// 라이브러리로 교체해야 합니다.

inline float calculateMDF(const float* samples, int n, float sampleRate) {
  if (samples == 0 || n < 2 || sampleRate <= 0.0f || !isfinite(sampleRate)) {
    return 0.0f;
  }

  float timeDomainPower = 0.0f;
  for (int t = 0; t < n; t++) {
    if (!isfinite(samples[t])) return 0.0f;
    timeDomainPower += samples[t] * samples[t];
  }
  if (timeDomainPower <= 0.0f || !isfinite(timeDomainPower)) return 0.0f;

  // With a one-sided spectrum, interior-bin power is doubled. Parseval's
  // relation then gives the total as n * sum(samples[t]^2).
  const float totalPower = (float)n * timeDomainPower;
  if (!isfinite(totalPower)) return 0.0f;

  const int lastBin = n / 2;
  float cumulative = 0.0f;

  for (int k = 0; k <= lastBin; k++) {
    float real = 0.0f, imag = 0.0f;
    for (int t = 0; t < n; t++) {
      float angle = 2.0f * (float)M_PI * (float)k * (float)t / (float)n;
      real += samples[t] * cosf(angle);
      imag -= samples[t] * sinf(angle);
    }
    float binPower = real * real + imag * imag;
    const bool isDc = (k == 0);
    const bool isNyquist = ((n % 2) == 0 && k == lastBin);
    if (!isDc && !isNyquist) binPower *= 2.0f;
    cumulative += binPower;

    if (cumulative >= totalPower / 2.0f) {
      return (float)k * sampleRate / n;
    }
  }
  return 0.0f;
}

// ---------------------------------------------------------------
// STEP 3. 기준값 대비 변화율 계산
// ---------------------------------------------------------------
// baselineValue가 0에 아주 가까우면(부동소수점 오차 수준) 나눗셈 결과가
// 비정상적으로 커질 수 있어 최소 크기 하한을 둡니다.
#ifndef MIN_BASELINE_MAGNITUDE
#define MIN_BASELINE_MAGNITUDE 1e-6f
#endif

inline float calculateChangeRate(float currentValue, float baselineValue) {
  if (!isfinite(currentValue) || !isfinite(baselineValue) ||
      fabsf(baselineValue) < MIN_BASELINE_MAGNITUDE) {
    return 0.0f;
  }
  const float changeRate = (currentValue - baselineValue) / baselineValue;
  return isfinite(changeRate) ? changeRate : 0.0f;
}

// ---------------------------------------------------------------
// STEP 4. 피로 판정 로직
// ---------------------------------------------------------------
struct FatigueThresholds {
  float rmsIncreaseThreshold;
  float mdfDecreaseThreshold;
};

inline bool isFatigued(float rmsChangeRate, float mdfChangeRate, const FatigueThresholds& th) {
  if (!isfinite(rmsChangeRate) || !isfinite(mdfChangeRate) ||
      !isfinite(th.rmsIncreaseThreshold) || !isfinite(th.mdfDecreaseThreshold) ||
      th.rmsIncreaseThreshold < 0.0f || th.mdfDecreaseThreshold < 0.0f) {
    return false;
  }
  bool rmsUp = (rmsChangeRate >= th.rmsIncreaseThreshold);
  bool mdfDown = (mdfChangeRate <= -th.mdfDecreaseThreshold);
  return rmsUp && mdfDown;
}

// ---------------------------------------------------------------
// STEP 4-0. IMU 원신호 -> 정지 여부(isStaticNow) 판정 + 센서 오류 처리
// ---------------------------------------------------------------
// updateStaticSegmentGate()에 넘길 isStaticNow(bool)를 IMU 원신호(가속도/
// 각속도)로부터 만들어내는 부분만 따로 뗀 함수들입니다. 임계값은 전부
// 실측 전 잠정값이며, .ino에서 ImuStaticDetectionConfig 값만 바꾸면 되고
// 이 함수들 자체는 손댈 필요가 없습니다.
#ifndef DEFAULT_IMU_ACCEL_STATIC_DEV_G
#define DEFAULT_IMU_ACCEL_STATIC_DEV_G 0.05f  // 잠정값. 실측 후 조정
#endif
#ifndef DEFAULT_IMU_GYRO_STATIC_DPS
#define DEFAULT_IMU_GYRO_STATIC_DPS 5.0f      // 잠정값. 실측 후 조정
#endif
#ifndef DEFAULT_IMU_MAX_VALID_ACCEL_G
#define DEFAULT_IMU_MAX_VALID_ACCEL_G 16.0f
#endif
#ifndef DEFAULT_IMU_MAX_VALID_GYRO_DPS
#define DEFAULT_IMU_MAX_VALID_GYRO_DPS 2000.0f
#endif

struct ImuStaticDetectionConfig {
  float gravityG = 1.0f;
  // 가속도 크기가 중력(gravityG)에서 이 이상 벗어나면 "움직이는 중"으로 봄.
  float accelStaticDeviationG = DEFAULT_IMU_ACCEL_STATIC_DEV_G;
  // 각속도 크기가 이 이상이면 "움직이는 중"으로 봄.
  float gyroStaticThresholdDps = DEFAULT_IMU_GYRO_STATIC_DPS;
  // 이 범위를 벗어나는 값은 통신 오류/글리치로 보고 무효 처리함.
  float maxValidAccelG = DEFAULT_IMU_MAX_VALID_ACCEL_G;
  float maxValidGyroDps = DEFAULT_IMU_MAX_VALID_GYRO_DPS;
};

// DS18B20 쪽 TempSafetyStatus와 같은 목적: IMU 통신 실패/비정상 값을
// "정지 아님(false)"과 구분해서 로그·상태로 남길 수 있게 함. 기존 세
// 상태의 숫자값 호환성을 유지하기 위해 IMU_CONFIG_ERROR는 끝에 추가합니다.
enum ImuSafetyStatus {
  IMU_OK,
  IMU_COMM_ERROR,
  IMU_DATA_INVALID,
  IMU_CONFIG_ERROR
};

inline bool isImuStaticDetectionConfigValid(
    const ImuStaticDetectionConfig& config) {
  if (!isfinite(config.gravityG) ||
      !isfinite(config.accelStaticDeviationG) ||
      !isfinite(config.gyroStaticThresholdDps) ||
      !isfinite(config.maxValidAccelG) ||
      !isfinite(config.maxValidGyroDps)) {
    return false;
  }

  if (config.gravityG <= 0.0f ||
      config.accelStaticDeviationG < 0.0f ||
      config.accelStaticDeviationG >= config.gravityG ||
      config.gyroStaticThresholdDps < 0.0f ||
      config.maxValidAccelG <= 0.0f ||
      config.maxValidGyroDps <= 0.0f ||
      config.gravityG > config.maxValidAccelG ||
      config.accelStaticDeviationG >
          config.maxValidAccelG - config.gravityG ||
      config.gyroStaticThresholdDps > config.maxValidGyroDps) {
    return false;
  }

  return true;
}

inline float calculateAccelMagnitudeG(float axG, float ayG, float azG) {
  if (!isfinite(axG) || !isfinite(ayG) || !isfinite(azG)) return -1.0f;
  return sqrtf(axG * axG + ayG * ayG + azG * azG);
}

inline float calculateGyroMagnitudeDps(float gxDps, float gyDps, float gzDps) {
  if (!isfinite(gxDps) || !isfinite(gyDps) || !isfinite(gzDps)) return -1.0f;
  return sqrtf(gxDps * gxDps + gyDps * gyDps + gzDps * gzDps);
}

// commOk: I2C 등 이번 프레임의 통신 성공 여부를 .ino에서 판단해 전달.
inline ImuSafetyStatus getImuSafetyStatus(bool commOk,
                                          float accelMagnitudeG,
                                          float gyroMagnitudeDps,
                                          const ImuStaticDetectionConfig& config) {
  if (!commOk) return IMU_COMM_ERROR;
  if (!isImuStaticDetectionConfigValid(config)) return IMU_CONFIG_ERROR;
  if (accelMagnitudeG < 0.0f || gyroMagnitudeDps < 0.0f ||
      !isfinite(accelMagnitudeG) || !isfinite(gyroMagnitudeDps) ||
      accelMagnitudeG > config.maxValidAccelG ||
      gyroMagnitudeDps > config.maxValidGyroDps) {
    return IMU_DATA_INVALID;
  }
  return IMU_OK;
}

struct ImuStaticEvaluation {
  bool isStaticNow = false;
  ImuSafetyStatus status = IMU_DATA_INVALID;
};

// 온도 쪽 isSensorErrorTemp와 같은 원칙: 통신 오류/무효 데이터는 절대
// "정지"로 판정하지 않음 (안전한 쪽인 "동작 중"으로 취급해 MDF 계산을 막음).
inline ImuStaticEvaluation evaluateImuStaticNow(bool commOk,
                                                float accelMagnitudeG,
                                                float gyroMagnitudeDps,
                                                const ImuStaticDetectionConfig& config) {
  ImuStaticEvaluation result;
  result.status = getImuSafetyStatus(commOk, accelMagnitudeG, gyroMagnitudeDps, config);
  if (result.status != IMU_OK) {
    result.isStaticNow = false;
    return result;
  }
  const bool accelStill =
      fabsf(accelMagnitudeG - config.gravityG) <= config.accelStaticDeviationG;
  const bool gyroStill = gyroMagnitudeDps <= config.gyroStaticThresholdDps;
  result.isStaticNow = accelStill && gyroStill;
  return result;
}

inline ImuStaticEvaluation evaluateImuStaticNow(bool commOk,
                                                float accelMagnitudeG,
                                                float gyroMagnitudeDps) {
  const ImuStaticDetectionConfig config;
  return evaluateImuStaticNow(commOk, accelMagnitudeG, gyroMagnitudeDps, config);
}

// ---------------------------------------------------------------
// STEP 4-1. Minimum static-segment duration gate
// ---------------------------------------------------------------
// Keep MDF/static fatigue analysis disabled until a continuous static segment
// reaches this duration. The caller may pass a different threshold.
#ifndef DEFAULT_MIN_STATIC_SEGMENT_MS
#define DEFAULT_MIN_STATIC_SEGMENT_MS 5000UL
#endif

// 정지<->동작 판정이 바뀌려면(양방향 모두) isStaticNow가 이 시간 이상
// "끊기지 않고" 새 상태를 유지해야 합니다. 이보다 짧은 flicker(센서
// 노이즈로 한두 프레임만 반대 상태로 튀는 경우)는 무시되며, 전환 시도
// 중이라도 원래 상태의 샘플이 다시 들어오면 그 즉시 취소됩니다.
#ifndef DEFAULT_MOTION_DEBOUNCE_MS
#define DEFAULT_MOTION_DEBOUNCE_MS 300UL
#endif

// 보드·IMU가 바뀌어도 헤더를 수정하지 않고 .ino에서 설정만 전달합니다.
struct StaticSegmentConfig {
  unsigned long minStaticDurationMs = DEFAULT_MIN_STATIC_SEGMENT_MS;
  unsigned long motionDebounceMs = DEFAULT_MOTION_DEBOUNCE_MS;
};

struct StaticSegmentGate {
  bool isStatic = false;
  unsigned long staticStartMs = 0;
  // isStaticNow와 확정된 isStatic이 다를 때(상태 전환 후보) 그 불일치가
  // 얼마나 오래 "끊기지 않고" 이어지는지 재는 대기(pending) 타이머입니다.
  // 정지->동작, 동작->정지 양쪽 전환에 동일하게 적용됩니다(대칭 디바운스).
  bool hasPending = false;
  bool pendingState = false;
  unsigned long pendingStartMs = 0;
};

// Update this once per motion-state decision. Returns true only while the
// current static segment is long enough for MDF/static logic. Both
// transitions (static->moving and moving->static) require isStaticNow to
// disagree with the confirmed state continuously for motionDebounceMs before
// committing; a single differing sample (sensor noise) does not flip the
// state, and any partial progress toward a flip is cancelled the moment a
// sample matching the confirmed state reappears. A raw moving frame always
// returns false so MDF is never calculated from motion-contaminated data,
// even while the confirmed static state is retained during exit debounce.
inline bool updateStaticSegmentGate(StaticSegmentGate& gate,
                                    bool isStaticNow,
                                    unsigned long currentTimeMs,
                                    unsigned long minStaticDurationMs = DEFAULT_MIN_STATIC_SEGMENT_MS,
                                    unsigned long motionDebounceMs = DEFAULT_MOTION_DEBOUNCE_MS) {
  if (isStaticNow == gate.isStatic) {
    // 현재 확정 상태와 같은 샘플이 들어오면, 반대 방향으로 진행 중이던
    // 전환 시도(있다면)는 노이즈였던 것이므로 취소합니다.
    gate.hasPending = false;
  } else {
    if (!gate.hasPending || gate.pendingState != isStaticNow) {
      gate.hasPending = true;
      gate.pendingState = isStaticNow;
      gate.pendingStartMs = currentTimeMs;
    }
    // Unsigned subtraction also follows the millis() wrap-around convention.
    if ((currentTimeMs - gate.pendingStartMs) >= motionDebounceMs) {
      gate.isStatic = isStaticNow;
      gate.hasPending = false;
      if (gate.isStatic) {
        gate.staticStartMs = currentTimeMs;
      }
    }
  }

  // A short moving flicker does not reset the confirmed static segment, but
  // MDF/static analysis must still be blocked on every raw moving frame.
  if (!isStaticNow) return false;

  if (gate.isStatic) {
    return (currentTimeMs - gate.staticStartMs) >= minStaticDurationMs;
  }
  return false;
}

inline bool updateStaticSegmentGate(StaticSegmentGate& gate,
                                    bool isStaticNow,
                                    unsigned long currentTimeMs,
                                    const StaticSegmentConfig& config) {
  return updateStaticSegmentGate(gate, isStaticNow, currentTimeMs,
                                 config.minStaticDurationMs,
                                 config.motionDebounceMs);
}

inline void resetStaticSegmentGate(StaticSegmentGate& gate) {
  gate.isStatic = false;
  gate.staticStartMs = 0;
  gate.hasPending = false;
  gate.pendingState = false;
  gate.pendingStartMs = 0;
}

struct StaticMDFResult {
  bool valid = false;
  float mdfHz = 0.0f;
};

// PC에서는 기본 DFT를, ESP32에서는 FFT 구현을 전달할 수 있습니다.
// 이렇게 하면 실제 보드 성능에 맞춰 계산기만 .ino에서 교체하고 이 헤더는
// 수정하지 않아도 됩니다.
typedef float (*MDFCalculatorFunction)(const float* samples,
                                       int n,
                                       float sampleRate);

// 게이트 상태와 무관하게, "이미 eligible하다고 확인된" 샘플로 MDF만 계산
// 합니다. updateStaticSegmentGate를 호출하지 않으므로, 호출자가 같은
// 프레임에서 게이트를 이미 갱신했다면(예: processStaticMDF) 이 함수를 써서
// 게이트가 한 프레임에 두 번 갱신되는 것을 피할 수 있습니다.
inline StaticMDFResult calculateMDFForEligibleSamples(
    const float* samples,
    int n,
    float sampleRate,
    MDFCalculatorFunction mdfCalculator) {
  StaticMDFResult result;

  if (samples == 0 || n < 2 || sampleRate <= 0.0f ||
      !isfinite(sampleRate) || mdfCalculator == 0) {
    return result;
  }

  float signalPower = 0.0f;
  for (int i = 0; i < n; i++) {
    if (!isfinite(samples[i])) return result;
    signalPower += samples[i] * samples[i];
  }
  if (signalPower <= 0.0f || !isfinite(signalPower)) return result;

  const float mdfHz = mdfCalculator(samples, n, sampleRate);
  if (!isfinite(mdfHz) || mdfHz < 0.0f || mdfHz > sampleRate / 2.0f) {
    return result;
  }

  result.mdfHz = mdfHz;
  result.valid = true;
  return result;
}

inline StaticMDFResult calculateStaticMDFWithBackendIfEligible(
    StaticSegmentGate& gate,
    bool isStaticNow,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    MDFCalculatorFunction mdfCalculator,
    unsigned long minStaticDurationMs = DEFAULT_MIN_STATIC_SEGMENT_MS,
    unsigned long motionDebounceMs = DEFAULT_MOTION_DEBOUNCE_MS) {
  if (!updateStaticSegmentGate(gate, isStaticNow, currentTimeMs,
                               minStaticDurationMs, motionDebounceMs)) {
    return StaticMDFResult();
  }
  return calculateMDFForEligibleSamples(samples, n, sampleRate, mdfCalculator);
}

inline StaticMDFResult calculateStaticMDFWithBackendIfEligible(
    StaticSegmentGate& gate,
    bool isStaticNow,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    MDFCalculatorFunction mdfCalculator,
    const StaticSegmentConfig& config) {
  return calculateStaticMDFWithBackendIfEligible(
      gate, isStaticNow, currentTimeMs, samples, n, sampleRate,
      mdfCalculator, config.minStaticDurationMs, config.motionDebounceMs);
}

// 정지시간 게이트와 MDF 계산을 한 함수로 묶은 권장 API입니다.
// valid가 false인 결과는 MDF 히스토리나 피로 판정에 사용하면 안 됩니다.
inline StaticMDFResult calculateStaticMDFIfEligible(
    StaticSegmentGate& gate,
    bool isStaticNow,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    unsigned long minStaticDurationMs = DEFAULT_MIN_STATIC_SEGMENT_MS,
    unsigned long motionDebounceMs = DEFAULT_MOTION_DEBOUNCE_MS) {
  return calculateStaticMDFWithBackendIfEligible(
      gate, isStaticNow, currentTimeMs, samples, n, sampleRate,
      calculateMDF, minStaticDurationMs, motionDebounceMs);
}

inline StaticMDFResult calculateStaticMDFIfEligible(
    StaticSegmentGate& gate,
    bool isStaticNow,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    const StaticSegmentConfig& config) {
  return calculateStaticMDFIfEligible(
      gate, isStaticNow, currentTimeMs, samples, n, sampleRate,
      config.minStaticDurationMs, config.motionDebounceMs);
}

// 실제 loop()에서 권장하는 주기 제한형 MDF 처리기입니다. 정지 구간이 처음
// 유효해졌을 때 한 번 계산하고, 이후 analysisIntervalMs마다만 다시 계산합니다.
struct StaticMDFProcessingConfig {
  StaticSegmentConfig segment;
  unsigned long analysisIntervalMs = 5000UL;
};

struct StaticMDFProcessor {
  StaticSegmentGate gate;
  bool hasPreviousAnalysis = false;
  unsigned long lastAnalysisMs = 0;
};

inline void resetStaticMDFProcessor(StaticMDFProcessor& processor) {
  resetStaticSegmentGate(processor.gate);
  processor.hasPreviousAnalysis = false;
  processor.lastAnalysisMs = 0;
}

inline StaticMDFResult processStaticMDF(
    StaticMDFProcessor& processor,
    bool isStaticNow,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    MDFCalculatorFunction mdfCalculator,
    const StaticMDFProcessingConfig& config) {
  StaticMDFResult result;
  if (config.analysisIntervalMs == 0) return result;

  const bool wasStatic = processor.gate.isStatic;
  const bool eligible = updateStaticSegmentGate(
      processor.gate, isStaticNow, currentTimeMs, config.segment);

  // 연속 동작으로 구간이 실제 종료되면 새 정지 구간에서 첫 분석을 다시
  // 수행할 수 있도록 분석 주기 상태도 초기화합니다.
  if (wasStatic && !processor.gate.isStatic) {
    processor.hasPreviousAnalysis = false;
    processor.lastAnalysisMs = 0;
  }

  if (!eligible) return result;
  if (processor.hasPreviousAnalysis &&
      (currentTimeMs - processor.lastAnalysisMs) < config.analysisIntervalMs) {
    return result;
  }

  // 이번 프레임의 게이트 갱신은 위 updateStaticSegmentGate 호출로 이미
  // 끝났으므로, calculateStaticMDFWithBackendIfEligible(내부에서 게이트를
  // 다시 갱신함)을 쓰지 않고 MDF 계산만 하는 함수를 직접 호출합니다.
  result = calculateMDFForEligibleSamples(samples, n, sampleRate, mdfCalculator);
  if (result.valid) {
    processor.hasPreviousAnalysis = true;
    processor.lastAnalysisMs = currentTimeMs;
  }
  return result;
}

inline StaticMDFResult processStaticMDF(
    StaticMDFProcessor& processor,
    bool isStaticNow,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    const StaticMDFProcessingConfig& config) {
  return processStaticMDF(processor, isStaticNow, currentTimeMs,
                          samples, n, sampleRate, calculateMDF, config);
}

// ---------------------------------------------------------------
// STEP 4-2. RMS 수축(contraction) 판정 게이트
// ---------------------------------------------------------------
// IMU의 StaticSegmentConfig/Gate와 같은 패턴입니다: 임계값을 상수 대신
// config로 빼두고, 캘리브레이션 평균/표준편차 기반 이력현상(hysteresis)
// + 블록 단위 디바운스로 켜짐/꺼짐을 판정합니다. (mdf_realtime_plot 계열
// .ino에서 쓰던 START_STD_MULT/END_STD_MULT/DEBOUNCE_BLOCKS와 같은 개념을
// 여기서는 config 구조체로 통일한 것뿐, 판정 방식 자체는 동일합니다.)
#ifndef DEFAULT_RMS_START_STD_MULT
#define DEFAULT_RMS_START_STD_MULT 3.0f  // 잠정값. 실측 후 조정
#endif
#ifndef DEFAULT_RMS_END_STD_MULT
#define DEFAULT_RMS_END_STD_MULT 1.5f    // 잠정값. START보다 낮아야 이력현상 성립
#endif
#ifndef DEFAULT_RMS_DEBOUNCE_BLOCKS
#define DEFAULT_RMS_DEBOUNCE_BLOCKS 2
#endif

struct RmsContractionConfig {
  float startStdMult = DEFAULT_RMS_START_STD_MULT;
  float endStdMult = DEFAULT_RMS_END_STD_MULT;
  int debounceBlocks = DEFAULT_RMS_DEBOUNCE_BLOCKS;
};

inline bool isRmsContractionConfigValid(
    const RmsContractionConfig& config) {
  return isfinite(config.startStdMult) &&
         isfinite(config.endStdMult) &&
         config.startStdMult >= 0.0f &&
         config.endStdMult >= 0.0f &&
         config.startStdMult > config.endStdMult &&
         config.debounceBlocks > 0;
}

struct RmsContractionGate {
  bool isContracting = false;
  int aboveCount = 0;
  int belowCount = 0;
};

inline void resetRmsContractionGate(RmsContractionGate& gate) {
  gate.isContracting = false;
  gate.aboveCount = 0;
  gate.belowCount = 0;
}

// currentRms를 캘리브레이션 평균/표준편차 기반 시작/종료 임계값과 비교해,
// debounceBlocks번 연속으로 조건을 충족해야 실제 상태(isContracting)가
// 바뀝니다. 현재 RMS나 캘리브레이션 값이 유효하지 않으면 오래된 true
// 상태가 MDF 분석에 사용되지 않도록 게이트를 안전하게 해제합니다.
inline bool updateRmsContractionGate(RmsContractionGate& gate,
                                     float currentRms,
                                     float baselineMean,
                                     float baselineStd,
                                     const RmsContractionConfig& config) {
  // 역전되거나 유효하지 않은 히스테리시스 설정은 같은 RMS 값에서 상태가
  // 반복 전환될 수 있으므로, 설정 오류 시 수축 상태를 안전하게 해제합니다.
  if (!isRmsContractionConfigValid(config)) {
    resetRmsContractionGate(gate);
    return false;
  }

  // baselineStd == 0은 수학적으로는 계산 가능하지만(시작=종료=평균이 되어
  // 이력현상 폭이 0으로 붕괴), 실제로는 캘리브레이션이 아직 제대로 된
  // 분산 데이터를 못 모은 상태일 가능성이 큽니다. RMS 계열 값은 음수가
  // 될 수 없으며, NaN/Inf를 포함한 무효 입력에서는 fail-safe로 해제합니다.
  if (!isfinite(currentRms) || !isfinite(baselineMean) ||
      !isfinite(baselineStd) || currentRms < 0.0f ||
      baselineMean < 0.0f || baselineStd <= 0.0f) {
    resetRmsContractionGate(gate);
    return false;
  }

  const float startThreshold = baselineMean + config.startStdMult * baselineStd;
  const float endThreshold = baselineMean + config.endStdMult * baselineStd;
  if (!isfinite(startThreshold) || !isfinite(endThreshold)) {
    resetRmsContractionGate(gate);
    return false;
  }

  if (!gate.isContracting) {
    gate.belowCount = 0;
    if (currentRms >= startThreshold) {
      gate.aboveCount++;
      if (gate.aboveCount >= config.debounceBlocks) {
        gate.isContracting = true;
        gate.aboveCount = 0;
      }
    } else {
      gate.aboveCount = 0;
    }
  } else {
    gate.aboveCount = 0;
    if (currentRms <= endThreshold) {
      gate.belowCount++;
      if (gate.belowCount >= config.debounceBlocks) {
        gate.isContracting = false;
        gate.belowCount = 0;
      }
    } else {
      gate.belowCount = 0;
    }
  }

  return gate.isContracting;
}

// ---------------------------------------------------------------
// STEP 4-2b. RMS 베이스라인 캘리브레이션 품질 검사
// ---------------------------------------------------------------
// 캘리브레이션 구간(예: 5~10초) 동안 모은 블록별 RMS 값 배열을 받아
// 중앙값 기반 이상치 비율과 변동계수(CV)로 품질을 판정합니다. 접촉불량이나
// 순간적인 움직임/노이즈로 한두 블록만 크게 튄 상태를 그대로 기준값(평균/
// 표준편차)으로 채택하는 것을 막기 위함입니다. mdf_realtime_plot 계열
// .ino에 있던 checkCalibQuality()와 동일한 방식(중앙값+MAD 기반 이상치
// 탐지)을 이 헤더로 이식한 것이며, 판정 로직 자체는 바뀌지 않았습니다.
#ifndef DEFAULT_CALIB_MAX_CV
#define DEFAULT_CALIB_MAX_CV 0.5f  // 표준편차/평균 이 값을 넘으면 불안정으로 봄
#endif
#ifndef DEFAULT_CALIB_MAX_OUTLIER_COUNT
#define DEFAULT_CALIB_MAX_OUTLIER_COUNT 1  // 표본 중 이상치 개수가 이보다 많으면 실패
#endif
#ifndef DEFAULT_CALIB_MIN_MAD_RATIO
#define DEFAULT_CALIB_MIN_MAD_RATIO 0.005f  // 중앙값 대비 MAD 최소 비율(하한)
#endif
#ifndef CALIB_MAX_SAMPLES
#define CALIB_MAX_SAMPLES 64  // 이 함수 내부 정렬 버퍼 크기. 호출자의 n은 이 이하여야 함
#endif

struct BaselineQualityConfig {
  float maxCoefficientOfVariation = DEFAULT_CALIB_MAX_CV;
  int maxOutlierCount = DEFAULT_CALIB_MAX_OUTLIER_COUNT;
  float minMadRatio = DEFAULT_CALIB_MIN_MAD_RATIO;
};

struct BaselineQualityResult {
  bool ok = false;
  float mean = 0.0f;
  float std = 0.0f;
  int outlierCount = 0;
};

// data/n: 캘리브레이션 구간에서 모은 블록별 RMS 값들. n은 CALIB_MAX_SAMPLES
// 이하여야 하며(내부 스택 배열 고정크기), 그 이상은 호출자가 잘라서 넘겨야
// 합니다. 반환된 result.ok가 false면 mean/std를 기준값으로 쓰면 안 됩니다.
inline BaselineQualityResult checkCalibQuality(const float* data, int n,
                                               const BaselineQualityConfig& config) {
  BaselineQualityResult result;
  if (data == 0 || n <= 0 || n > CALIB_MAX_SAMPLES) return result;
  if (!isfinite(config.maxCoefficientOfVariation) ||
      !isfinite(config.minMadRatio) ||
      config.maxCoefficientOfVariation < 0.0f ||
      config.minMadRatio < 0.0f ||
      config.maxOutlierCount < 0) {
    return result;
  }

  for (int i = 0; i < n; i++) {
    if (!isfinite(data[i]) || data[i] < 0.0f) return result;
  }

  const size_t count = static_cast<size_t>(n);
  float sorted[CALIB_MAX_SAMPLES] = {};
  for (size_t i = 0; i < count; i++) sorted[i] = data[i];
  for (size_t i = 1; i < count; i++) {
    float key = sorted[i];
    size_t j = i;
    while (j > 0U && sorted[j - 1U] > key) {
      sorted[j] = sorted[j - 1U];
      --j;
    }
    sorted[j] = key;
  }
  const size_t lowerMedianIndex = (count - 1U) / 2U;
  const size_t upperMedianIndex = count / 2U;
  const float median = 0.5f * sorted[lowerMedianIndex] +
                       0.5f * sorted[upperMedianIndex];

  float sortedDev[CALIB_MAX_SAMPLES] = {};
  for (size_t i = 0; i < count; i++) {
    sortedDev[i] = fabsf(data[i] - median);
  }
  for (size_t i = 1; i < count; i++) {
    float key = sortedDev[i];
    size_t j = i;
    while (j > 0U && sortedDev[j - 1U] > key) {
      sortedDev[j] = sortedDev[j - 1U];
      --j;
    }
    sortedDev[j] = key;
  }
  float mad = (0.5f * sortedDev[lowerMedianIndex] +
               0.5f * sortedDev[upperMedianIndex]) * 1.4826f;
  const float minMad = fabsf(median) * config.minMadRatio;
  if (mad < minMad) mad = minMad;
  if (mad < 1e-6f) mad = 1e-6f;

  int outlierCount = 0;
  for (int i = 0; i < n; i++) {
    if (fabsf(data[i] - median) > 3.0f * mad) outlierCount++;
  }

  double sum = 0.0, sumSq = 0.0;
  for (int i = 0; i < n; i++) {
    const double value = static_cast<double>(data[i]);
    sum += value;
    sumSq += value * value;
  }
  const double meanDouble = sum / static_cast<double>(n);
  double varianceDouble = 0.0;
  if (n > 1) {
    varianceDouble =
        (sumSq - (sum * sum) / static_cast<double>(n)) /
        static_cast<double>(n - 1);
    if (varianceDouble < 0.0 &&
        varianceDouble > -1e-12 * (sumSq + 1.0)) {
      varianceDouble = 0.0;
    }
  }
  const double stdDouble = varianceDouble > 0.0
      ? sqrt(varianceDouble) : 0.0;
  const float mean = static_cast<float>(meanDouble);
  const float std = static_cast<float>(stdDouble);
  const bool statisticsValid =
      isfinite(mean) && mean > 0.0f && isfinite(std) && std > 0.0f;
  const float cv = statisticsValid ? (std / mean) : 999.0f;

  result.mean = mean;
  result.std = std;
  result.outlierCount = outlierCount;
  result.ok = statisticsValid &&
              (cv <= config.maxCoefficientOfVariation) &&
              (outlierCount <= config.maxOutlierCount);
  return result;
}

inline BaselineQualityResult checkCalibQuality(const float* data, int n) {
  const BaselineQualityConfig config;
  return checkCalibQuality(data, n, config);
}

// ---------------------------------------------------------------
// STEP 4-2c. 이미 확정된 최종 MDF 적격 판정을 그대로 쓰는 처리기
// ---------------------------------------------------------------
// processStaticMDF()(위)는 내부에 자기 소유의 StaticSegmentGate를 두고
// 호출될 때마다 디바운스+최소정지시간 판정을 처음부터 다시 한다. 시스템에
// 이미 다른 곳에서 RMS 수축과 IMU 정지 판정을 끝냈다면, 두 값을
// isMdfEligibleForFatigueAnalysis()로 결합한 최종 bool을 이 함수에 넘깁니다.
// 정지 여부만 넘기면 비수축 구간에서도 FFT가 실행되므로 금지합니다.
//
// 여기서는 정지-판정 자체(StaticSegmentGate/디바운스)를 하지 않고, 이미
// eligible로 확정된 호출자의 결과만 받아 "분석 주기(analysisIntervalMs)
// 제한"만 처리한다. eligible이 false가 되면 다음 정지 구간에서 새로
// 분석하도록 상태를 초기화한다(연속된 다른 정지 구간의 데이터가 섞이지
// 않게).
struct EligibilityGatedMDFProcessor {
  bool hasPreviousAnalysis = false;
  unsigned long lastAnalysisMs = 0;
};

inline void resetEligibilityGatedMDFProcessor(
    EligibilityGatedMDFProcessor& processor) {
  processor.hasPreviousAnalysis = false;
  processor.lastAnalysisMs = 0;
}

inline StaticMDFResult processMdfForEligibleSegment(
    EligibilityGatedMDFProcessor& processor,
    bool isMdfEligible,
    bool resetAnalysisStateWhenIneligible,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    MDFCalculatorFunction mdfCalculator,
    unsigned long analysisIntervalMs) {
  StaticMDFResult result;
  if (analysisIntervalMs == 0) return result;

  if (!isMdfEligible) {
    // 원시 움직임 한 프레임처럼 현재 창만 부적격이고 확정 정지 구간은
    // 유지되는 경우에는 타이머를 보존합니다. 수축 종료, 확정 움직임 또는
    // 센서 오류처럼 실제 적격 구간이 끝난 경우에만 호출자가 true를 줍니다.
    if (resetAnalysisStateWhenIneligible) {
      processor.hasPreviousAnalysis = false;
      processor.lastAnalysisMs = 0;
    }
    return result;
  }

  if (processor.hasPreviousAnalysis &&
      (currentTimeMs - processor.lastAnalysisMs) < analysisIntervalMs) {
    return result;
  }

  result = calculateMDFForEligibleSamples(samples, n, sampleRate, mdfCalculator);
  if (result.valid) {
    processor.hasPreviousAnalysis = true;
    processor.lastAnalysisMs = currentTimeMs;
  }
  return result;
}

// 기존 호출자 호환 경로: false를 완전한 적격 구간 종료로 취급합니다.
inline StaticMDFResult processMdfForEligibleSegment(
    EligibilityGatedMDFProcessor& processor,
    bool isMdfEligible,
    unsigned long currentTimeMs,
    const float* samples,
    int n,
    float sampleRate,
    MDFCalculatorFunction mdfCalculator,
    unsigned long analysisIntervalMs) {
  return processMdfForEligibleSegment(
      processor, isMdfEligible, true, currentTimeMs, samples, n, sampleRate,
      mdfCalculator, analysisIntervalMs);
}

// ---------------------------------------------------------------
// STEP 4-3. RMS 게이트 + IMU 게이트 결합
// ---------------------------------------------------------------
// "이 순간 MDF를 피로 분석에 써도 되는가"는 각 게이트의 세부 구현과 무관
// 하게 결과 bool 2개만으로 결정됩니다. RMS/IMU 판정 로직이 나중에 바뀌어도
// 이 함수는 그대로 씁니다.
//   isContracting    : RmsContractionGate가 판정한 "지금 수축 중인가"
//   isStaticEligible : updateStaticSegmentGate()/processStaticMDF()가
//                      돌려주는 "지금 IMU 기준으로 MDF를 써도 되는가"
//                      (단순 정지 여부(isStatic)가 아니라
//                      minStaticDurationMs까지 충족된 값을 넣어야 함)
//
// 주의(실측 전 인지만 해두면 됨, 코드 수정 사항 아님): 두 게이트를 AND로
// 묶으면 실제로 이 함수가 true를 내려면 "IMU 기준 5초+ 연속 정지"와
// "RMS 기준 수축 판정"이 동시에 성립해야 합니다. 즉 최소 충족 시간은
// 두 게이트 중 더 늦게 켜지는 쪽 기준이라, IMU 쪽 minStaticDurationMs
// (기본 5초, 진입 디바운스 포함하면 5.3초)만 보고 튜닝하면 실제 반응
// 시간을 과소평가하게 됩니다.
inline bool isMdfEligibleForFatigueAnalysis(bool isContracting, bool isStaticEligible) {
  return isContracting && isStaticEligible;
}

// ---------------------------------------------------------------
// STEP 4-4. 시리얼 DATA 라인 필드 레이아웃 (I/O 없음, 필드 정의만)
// ---------------------------------------------------------------
// 실제 Serial.print()/파이썬 파싱은 .ino와 파이썬 쪽에서 담당합니다(이
// 파일은 파일 맨 위 설명대로 "가짜 데이터/테스트/출력 코드 없음" 원칙을
// 지킵니다). RMS 필드를 나중에 추가/삭제하면서 파이썬 파서와 계속 안
// 맞았던 문제가 있었으므로, IMU 관련 필드도 처음부터 자리를 잡아 이
// 구조체 하나로 고정해둡니다. 필드를 추가/삭제할 때는 이 구조체와
// .ino의 Serial.print(), 파이썬 쪽 파서 세 곳을 항상 같이 바꿔야 합니다.
struct SerialDataFrame {
  unsigned long timestampMs = 0;
  float rms = 0.0f;
  bool isContracting = false;   // RMS 게이트 판정 (RmsContractionGate)
  // 이름에 "Now"를 붙여 순간값(evaluateImuStaticNow의 isStaticNow)임을
  // 명시합니다. gate.isStatic(디바운스로 확정된 값)이 아니고, MDF 계산
  // 가능 여부 판단에도 이 필드가 아니라 아래 isMdfEligible을 써야 합니다.
  bool isImuStaticNow = false;
  bool isMdfEligible = false;   // isMdfEligibleForFatigueAnalysis() 결과
  float mdfHz = 0.0f;           // isMdfEligible == false면 무의미한 값(0.0f)
  ImuSafetyStatus imuStatus = IMU_OK;
};

// ---------------------------------------------------------------
// STEP 5. 5초 평균 버퍼
// ---------------------------------------------------------------
#define BUFFER_SIZE 5

struct AverageBuffer {
  float values[BUFFER_SIZE];
  int count = 0;
  int index = 0;
};

inline bool addToBuffer(AverageBuffer& buf, float newValue) {
  if (!isfinite(newValue)) return false;
  buf.values[buf.index] = newValue;
  buf.index = (buf.index + 1) % BUFFER_SIZE;
  if (buf.count < BUFFER_SIZE) buf.count++;
  return (buf.count == BUFFER_SIZE);
}

inline float getBufferAverage(const AverageBuffer& buf) {
  if (buf.count == 0) return 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < buf.count; i++) sum += buf.values[i];
  return sum / buf.count;
}

// ---------------------------------------------------------------
// STEP 6. 상태 머신 (측정 모드 <-> 개입 모드)
// ---------------------------------------------------------------
enum SystemState { STATE_MEASURING, STATE_INTERVENING };

struct StateMachine {
  SystemState state = STATE_MEASURING;
  unsigned long interventionStartMs = 0;
};

// currentTimeMs: 실전(Arduino)에서는 millis() 값을 그대로 넣어주면 됨
inline void updateStateMachine(StateMachine& sm, bool fatigueDetected, unsigned long currentTimeMs, unsigned long interventionDurationMs) {
  if (sm.state == STATE_MEASURING) {
    if (fatigueDetected) {
      sm.state = STATE_INTERVENING;
      sm.interventionStartMs = currentTimeMs;
    }
  } else {
    if (currentTimeMs - sm.interventionStartMs >= interventionDurationMs) {
      sm.state = STATE_MEASURING;
    }
  }
}

// ---------------------------------------------------------------
// STEP 7. 온도 on-off 제어 로직
// ---------------------------------------------------------------
// DS18B20은 변환 실패/초기화 직후 값을 못 받으면 스크래치패드 기본값인
// 85.0000C를 그대로 반환합니다. 이 값은 "진짜 과열"이 아니라 "센서 읽기
// 실패"이므로 로그/상태는 구분해서 남겨야 합니다. (히터 자체는 currentTemp
// >= 45.0f 조건으로 이미 안전하게 꺼지므로 shouldHeatOn의 동작은 그대로입니다.)
#ifndef DS18B20_ERROR_TEMP
#define DS18B20_ERROR_TEMP 85.0f
#endif
#ifndef DS18B20_ERROR_TEMP_MARGIN
#define DS18B20_ERROR_TEMP_MARGIN 1.0f
#endif

enum TempSafetyStatus { TEMP_OK, TEMP_SENSOR_ERROR, TEMP_EMERGENCY_OVERHEAT };

// 센서나 설정온도 범위가 바뀌어도 헤더를 수정하지 않고 .ino에서 설정합니다.
struct TemperatureSafetyConfig {
  float minSensorTempC = -55.0f;
  float maxSensorTempC = 125.0f;
  float minSetTempC = 38.0f;
  float maxSetTempC = 43.0f;
  float emergencyTempC = 45.0f;
  float sensorErrorTempC = DS18B20_ERROR_TEMP;
  float sensorErrorMarginC = DS18B20_ERROR_TEMP_MARGIN;
  float disconnectedTempC = -127.0f;
  float disconnectedMarginC = 1.0f;
};

inline bool isTemperatureSafetyConfigValid(const TemperatureSafetyConfig& config) {
  return isfinite(config.minSensorTempC) && isfinite(config.maxSensorTempC) &&
         isfinite(config.minSetTempC) && isfinite(config.maxSetTempC) &&
         isfinite(config.emergencyTempC) && isfinite(config.sensorErrorTempC) &&
         isfinite(config.sensorErrorMarginC) && isfinite(config.disconnectedTempC) &&
         isfinite(config.disconnectedMarginC) &&
         config.minSensorTempC < config.maxSensorTempC &&
         config.minSetTempC <= config.maxSetTempC &&
         config.minSetTempC >= config.minSensorTempC &&
         config.maxSetTempC < config.emergencyTempC &&
         config.emergencyTempC <= config.maxSensorTempC &&
         config.sensorErrorMarginC >= 0.0f && config.disconnectedMarginC >= 0.0f;
}

inline bool isSensorErrorTemp(float currentTemp,
                              const TemperatureSafetyConfig& config) {
  if (!isTemperatureSafetyConfigValid(config) || !isfinite(currentTemp) ||
      currentTemp < config.minSensorTempC || currentTemp > config.maxSensorTempC) {
    return true;
  }

  const bool startupError =
      fabsf(currentTemp - config.sensorErrorTempC) <= config.sensorErrorMarginC;
  const bool disconnectedError =
      fabsf(currentTemp - config.disconnectedTempC) <= config.disconnectedMarginC;
  return startupError || disconnectedError;
}

inline bool isSensorErrorTemp(float currentTemp) {
  const TemperatureSafetyConfig config;
  return isSensorErrorTemp(currentTemp, config);
}

inline TempSafetyStatus getTempSafetyStatus(
    float currentTemp, const TemperatureSafetyConfig& config) {
  if (isSensorErrorTemp(currentTemp, config)) {
    return TEMP_SENSOR_ERROR;
  }
  if (currentTemp >= config.emergencyTempC) {
    return TEMP_EMERGENCY_OVERHEAT;
  }
  return TEMP_OK;
}

inline TempSafetyStatus getTempSafetyStatus(float currentTemp) {
  const TemperatureSafetyConfig config;
  return getTempSafetyStatus(currentTemp, config);
}

inline bool shouldHeatOn(float currentTemp, float setTemp,
                         const TemperatureSafetyConfig& config) {
  if (getTempSafetyStatus(currentTemp, config) != TEMP_OK ||
      !isfinite(setTemp) || setTemp < config.minSetTempC ||
      setTemp > config.maxSetTempC) {
    return false;
  }
  return currentTemp < setTemp;
}

inline bool shouldHeatOn(float currentTemp, float setTemp) {
  const TemperatureSafetyConfig config;
  return shouldHeatOn(currentTemp, setTemp, config);
}

// ---------------------------------------------------------------
// STEP 8. 안전 범위 클램프
// ---------------------------------------------------------------
inline float clampSetTemp(float requestedTemp,
                          const TemperatureSafetyConfig& config) {
  if (!isTemperatureSafetyConfigValid(config)) return 38.0f;
  if (!isfinite(requestedTemp)) return config.minSetTempC;
  if (requestedTemp < config.minSetTempC) return config.minSetTempC;
  if (requestedTemp > config.maxSetTempC) return config.maxSetTempC;
  return requestedTemp;
}

inline float clampSetTemp(float requestedTemp) {
  const TemperatureSafetyConfig config;
  return clampSetTemp(requestedTemp, config);
}

inline bool isEmergencyOverheat(float currentTemp,
                                const TemperatureSafetyConfig& config) {
  return getTempSafetyStatus(currentTemp, config) == TEMP_EMERGENCY_OVERHEAT;
}

inline bool isEmergencyOverheat(float currentTemp) {
  const TemperatureSafetyConfig config;
  return isEmergencyOverheat(currentTemp, config);
}

inline bool isHeatingBlocked(float currentTemp,
                             const TemperatureSafetyConfig& config) {
  return getTempSafetyStatus(currentTemp, config) != TEMP_OK;
}

struct HeaterControlConfig {
  float hysteresisC = 0.5f;
  // 0이면 안전을 위해 히터를 켜지 않습니다. 실제 최대 연속 가열시간을
  // .ino에서 반드시 지정해야 합니다.
  unsigned long maxContinuousOnMs = 0;
};

struct HeaterController {
  bool heaterOn = false;
  bool timeoutLatched = false;
  unsigned long heaterOnStartMs = 0;
};

enum HeaterControlStatus {
  HEATER_OFF,
  HEATER_ON,
  HEATER_CONFIG_ERROR,
  HEATER_SENSOR_ERROR,
  HEATER_OVERHEAT,
  HEATER_TIMEOUT
};

struct HeaterControlResult {
  bool heaterOn = false;
  HeaterControlStatus status = HEATER_OFF;
};

inline void resetHeaterController(HeaterController& controller) {
  controller.heaterOn = false;
  controller.timeoutLatched = false;
  controller.heaterOnStartMs = 0;
}

inline HeaterControlResult updateHeaterController(
    HeaterController& controller,
    float currentTemp,
    float setTemp,
    unsigned long currentTimeMs,
    const TemperatureSafetyConfig& temperatureConfig,
    const HeaterControlConfig& heaterConfig) {
  HeaterControlResult result;

  if (!isTemperatureSafetyConfigValid(temperatureConfig) ||
      !isfinite(heaterConfig.hysteresisC) || heaterConfig.hysteresisC < 0.0f ||
      heaterConfig.maxContinuousOnMs == 0 || !isfinite(setTemp) ||
      setTemp < temperatureConfig.minSetTempC ||
      setTemp > temperatureConfig.maxSetTempC) {
    controller.heaterOn = false;
    result.status = HEATER_CONFIG_ERROR;
    return result;
  }

  const TempSafetyStatus temperatureStatus =
      getTempSafetyStatus(currentTemp, temperatureConfig);
  if (temperatureStatus == TEMP_SENSOR_ERROR) {
    controller.heaterOn = false;
    result.status = HEATER_SENSOR_ERROR;
    return result;
  }
  if (temperatureStatus == TEMP_EMERGENCY_OVERHEAT) {
    controller.heaterOn = false;
    result.status = HEATER_OVERHEAT;
    return result;
  }

  if (controller.timeoutLatched) {
    controller.heaterOn = false;
    result.status = HEATER_TIMEOUT;
    return result;
  }

  if (controller.heaterOn) {
    if ((currentTimeMs - controller.heaterOnStartMs) >=
        heaterConfig.maxContinuousOnMs) {
      controller.heaterOn = false;
      controller.timeoutLatched = true;
      result.status = HEATER_TIMEOUT;
      return result;
    }

    if (currentTemp >= setTemp) {
      controller.heaterOn = false;
      result.status = HEATER_OFF;
      return result;
    }

    result.heaterOn = true;
    result.status = HEATER_ON;
    return result;
  }

  if (currentTemp <= setTemp - heaterConfig.hysteresisC) {
    controller.heaterOn = true;
    controller.heaterOnStartMs = currentTimeMs;
    result.heaterOn = true;
    result.status = HEATER_ON;
  }
  return result;
}

// ---------------------------------------------------------------
// STEP 9. 버튼 디바운싱
// ---------------------------------------------------------------
struct ButtonState {
  bool lastRawState = false;
  bool debouncedState = false;
  unsigned long lastChangeMs = 0;
};

// rawState: 실전에서는 digitalRead(pin)==LOW 결과. currentTimeMs: 실전에서는 millis().
inline bool debounceButton(ButtonState& btn, bool rawState, unsigned long currentTimeMs, unsigned long debounceMs) {
  bool pressedEvent = false;
  if (rawState != btn.lastRawState) {
    btn.lastChangeMs = currentTimeMs;
    btn.lastRawState = rawState;
  }
  if ((currentTimeMs - btn.lastChangeMs) >= debounceMs) {
    if (btn.debouncedState != rawState) {
      btn.debouncedState = rawState;
      if (rawState == true) pressedEvent = true;
    }
  }
  return pressedEvent;
}

// ---------------------------------------------------------------
// STEP 10. 4자리 표시 포맷 (TM1637에 나중에 그대로 넘길 값)
// ---------------------------------------------------------------
inline void formatTempToDigits(float temp, int digits[4]) {
  if (digits == 0) return;

  if (!isfinite(temp) || temp < 0.0f || temp >= 100.0f) {
    digits[0] = 0;
    digits[1] = 0;
    digits[2] = 0;
    digits[3] = 0;
    return;
  }

  int tempInt = (int)temp;
  // (temp - tempInt) * 10 만 쓰면 부동소수점 오차로 9.9 -> 8이 나오는 등
  // 실제로 값이 깎이는 버그가 있었음. round()로 반올림해서 보정.
  int tempDecimal = (int)roundf((temp - tempInt) * 10.0f);
  // round() 결과가 10이 될 수 있음 (예: 39.95 -> 소수부 0.95*10=9.5 -> 반올림하면 10)
  // 이 경우 소수 자리를 0으로 내리고 정수 자리를 1 올려줘야 함 (39.95 -> 40.0)
  if (tempDecimal >= 10) {
    tempDecimal = 0;
    tempInt += 1;
  }
  // 이 포맷은 정수부 두 자리만 표시하므로 99.95~99.99가 반올림되어
  // 100.0이 되는 경우 표시 가능한 최댓값 99.9로 제한합니다.
  if (tempInt >= 100) {
    tempInt = 99;
    tempDecimal = 9;
  }
  digits[0] = tempInt / 10;
  digits[1] = tempInt % 10;
  digits[2] = tempDecimal;
  digits[3] = 0;
}

// ---------------------------------------------------------------
// STEP 11. 30초 MDF/RMS 추세 기반 피로 판정
// ---------------------------------------------------------------
constexpr int FATIGUE_TREND_HISTORY_SIZE = 6;

struct FatigueTrendHistory {
  float mdfValues[FATIGUE_TREND_HISTORY_SIZE] = {};
  float rmsValues[FATIGUE_TREND_HISTORY_SIZE] = {};
  unsigned long timestamps[FATIGUE_TREND_HISTORY_SIZE] = {};
  int count = 0;
  int nextIndex = 0;
};

struct FatigueTrendResult {
  bool valid = false;
  bool fatigued = false;
  float mdfPercentChange = 0.0f;
  float rmsPercentChange = 0.0f;
};

inline void resetFatigueTrendHistory(FatigueTrendHistory& history) {
  for (int i = 0; i < FATIGUE_TREND_HISTORY_SIZE; ++i) {
    history.mdfValues[i] = 0.0f;
    history.rmsValues[i] = 0.0f;
    history.timestamps[i] = 0UL;
  }
  history.count = 0;
  history.nextIndex = 0;
}

inline void pushFatigueTrendSample(FatigueTrendHistory& history,
                                   float mdfHz, float rms,
                                   unsigned long timestampMs) {
  history.mdfValues[history.nextIndex] = mdfHz;
  history.rmsValues[history.nextIndex] = rms;
  history.timestamps[history.nextIndex] = timestampMs;
  history.nextIndex =
      (history.nextIndex + 1) % FATIGUE_TREND_HISTORY_SIZE;
  if (history.count < FATIGUE_TREND_HISTORY_SIZE) ++history.count;
}

inline FatigueTrendResult evaluateFatigueTrend(
    const FatigueTrendHistory& history) {
  FatigueTrendResult result;
  if (history.count != FATIGUE_TREND_HISTORY_SIZE ||
      history.nextIndex < 0 ||
      history.nextIndex >= FATIGUE_TREND_HISTORY_SIZE) {
    return result;
  }

  const int firstIndex = history.nextIndex;
  const int lastIndex =
      (history.nextIndex + FATIGUE_TREND_HISTORY_SIZE - 1) %
      FATIGUE_TREND_HISTORY_SIZE;
  const float firstMdf = history.mdfValues[firstIndex];
  const float lastMdf = history.mdfValues[lastIndex];
  const float firstRms = history.rmsValues[firstIndex];
  const float lastRms = history.rmsValues[lastIndex];
  if (!isfinite(firstMdf) || !isfinite(lastMdf) ||
      !isfinite(firstRms) || !isfinite(lastRms) ||
      firstMdf == 0.0f || firstRms == 0.0f) {
    return result;
  }

  result.mdfPercentChange =
      (lastMdf - firstMdf) / firstMdf * 100.0f;
  result.rmsPercentChange =
      (lastRms - firstRms) / firstRms * 100.0f;
  if (!isfinite(result.mdfPercentChange) ||
      !isfinite(result.rmsPercentChange)) {
    result.mdfPercentChange = 0.0f;
    result.rmsPercentChange = 0.0f;
    return result;
  }

  result.valid = true;
  result.fatigued = result.mdfPercentChange <= -15.0f &&
                    result.rmsPercentChange >= -20.0f;
  return result;
}

#endif // FATIGUE_LOGIC_H
