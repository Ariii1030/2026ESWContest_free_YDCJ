#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include "fatigue_logic.h"
#include "arduino_fft_mdf_connection.h"
#include "pipeline_stage5_bridge.h"

// Arduino가 자동 생성하는 함수 원형에서 아래 타입을 먼저 알 수 있게 한다.
struct MdfTaskInput;
struct MdfOutputState;

#ifndef STAGE5_SYSTEM_VIBRATION_PIN
#define STAGE5_SYSTEM_VIBRATION_PIN 25
#endif
#ifndef STAGE5_SYSTEM_HEATER_PIN
#define STAGE5_SYSTEM_HEATER_PIN 26
#endif
#ifndef STAGE5_SYSTEM_TM1637_CLK_PIN
#define STAGE5_SYSTEM_TM1637_CLK_PIN 18
#endif
#ifndef STAGE5_SYSTEM_TM1637_DIO_PIN
#define STAGE5_SYSTEM_TM1637_DIO_PIN 19
#endif
#ifndef TEMP_SENSOR_PIN
#define TEMP_SENSOR_PIN 4
#endif
#ifndef BTN_UP_PIN
#define BTN_UP_PIN 32
#endif
#ifndef BTN_DOWN_PIN
#define BTN_DOWN_PIN 33
#endif
#ifndef BTN_OK_PIN
#define BTN_OK_PIN 27
#endif

#if STAGE5_SYSTEM_TM1637_CLK_PIN >= 0 && \
    STAGE5_SYSTEM_TM1637_DIO_PIN >= 0
#include <TM1637Display.h>
TM1637Display stage5SystemTm1637(STAGE5_SYSTEM_TM1637_CLK_PIN,
                                 STAGE5_SYSTEM_TM1637_DIO_PIN);
#endif

OneWire oneWireTempBus(TEMP_SENSOR_PIN);
DallasTemperature temperatureSensor(&oneWireTempBus);

// ESP32 combined IMU + RMS + MDF adapter for fatigue_logic.h v1.2.9, with
// checklist item 5 (SerialDataFrame output) added.
//
// Builds on the VERIFIED imu_rms_combined_gate_v1_2_7.ino (item 3: the two
// gates combined through isMdfEligibleForFatigueAnalysis()) and on
// arduino_fft_mdf_connection.h (item 4: calculateMDF -> arduinoFFT via
// ArduinoFftMdfConnection). Part 1 and Part 2 below are that verified v1.2.7
// logic, UNCHANGED. This file only adds Part 3 (a dedicated mdfTask that
// runs the FFT and prints one SerialDataFrame per completed EMG window while
// the bounded handoff queue has capacity) and
// switches setup()/loop() accordingly.
//
// [이 파일이 새로 나온 이유]
// 이전에 업로드됐던 imu_rms_mdf_dataframe_v1_2_5.ino는 검증되지 않은 v1.2.5
// 베이스로 짜여 있어 (1) serialMutex를 선언보다 먼저 쓰는 컴파일 에러가
// 있었고, (2) v1.2.6~v1.2.7에서 고친 것들(IMU 임계값 설정, 자이로 바이어스
// 보정, RMS 캘리브레이션 품질검사, EMG 신호품질 검사, debounceBlocks=8)이
// 전부 빠져 있었다. 이 파일은 그 검증된 v1.2.7을 그대로 베이스로 쓰고,
// SerialDataFrame 출력만 새로 얹은 것이다.
//
// Connect the sensor's RAW EMG output (not an envelope-only output) to
// EMG_ADC_PIN, and the MPU6050 to the I2C pins below. Keep the muscle
// relaxed during the automatic RMS baseline calibration.
//
// [v1.2.6 수정, 리뷰에서 발견된 3가지 문제 대응 — Part 1/2에 그대로 포함됨]
// 1) imuConfig의 정지판정 임계값을 이전 세션 실측값(가속도편차 0.15g,
//    각속도 60dps — mdf_realtime_plot15.ino에서 이미 확인된 값)으로
//    setup()에서 명시적으로 채워넣음. 기존엔 헤더 기본값(accel 0.05g /
//    gyro 5dps)이 그대로 쓰여서, 실측 노이즈 수준(정지 시에도 자이로
//    ~48dps)보다 훨씬 낮은 임계값 때문에 정지 게이트가 거의 안 열릴
//    위험이 있었음. 이 값도 여전히 "이전 보드/세션 기준 잠정값"이라
//    이 보드로 다시 실측하면 재조정 필요.
// 2) 자이로 바이어스 보정(calibrateGyroBias) 추가 — 움직임 중 얻은 표본은
//    제외하고, 축별 표준편차가 너무 크거나 유효 표본이 45/50개 미만이면
//    보정 자체를 실패 처리한다(예전 값을 계속 쓰지 않음).
// 3) RMS 베이스라인 캘리브레이션에 품질 검사(fatigue_logic.h의
//    checkCalibQuality, MAD 기반 이상치+CV 검사) 연결 — 품질 검사를
//    통과 못 하면 RMS_BASELINE_QUALITY_FAILED 상태로 멈추고 'C' 재전송을
//    기다림.
// 4) plot14의 1000Hz/1024샘플 계산에서 약 2초 수축 디바운스를 사용.
// 5) EMG 신호품질 검사(isEmgWindowSignalQualityAcceptable) — 평탄선(피크
//    투피크 2카운트 미만)과 급격한 스파이크(파고율 8.0 초과)를 RMS 계산
//    전에 걸러냄.
//
// [이번에 새로 추가한 부분 — checklist item 4/5]
// - mdfTask: FFT와 Serial 출력 전용 3번째 태스크(core0). ArduinoFftMdfConnection
//   ::processCombined()을 통해 이미 확정된 isContracting/isStaticEligible을
//   그대로 받아쓰므로, imuStaticGate와 별개인 두 번째 정지판정 게이트가
//   생기지 않는다(지난 4번 코드 리뷰에서 고친 부분 그대로 재사용).
// - queueMdfInput()의 gateGeneration: 적격 상태가 바뀔 때마다 증가시켜서,
//   서로 다른 정지·수축 구간의 MDF/피로 이력이 섞이지 않게 mdfTask 쪽에서
//   매번 리셋 여부를 판단한다.
// - imuTask 우선순위를 mdfTask보다 높게(2 vs 1) 둬서, FFT 연산이 도는 몇 ms
//   동안 IMU의 100ms 주기 읽기가 밀리지 않게 한다(같은 core0을 공유하므로).
// - serialMutex는 IMU 헬퍼 함수들이 에러 경로에서 바로 쓰기 때문에 파일
//   앞부분에서 선언한다(v1.2.5 dataframe 파일의 컴파일 에러 원인이었음).
// - Wire.end() 호출은 #if defined(WIRE_HAS_END)로 감싸서, 이 함수가 없는
//   보드 패키지 버전에서도 컴파일되게 한다.

// ===================================================================
// Part 1: IMU (from mpu6050_imu_adapter_v1_2_5_fixed.ino, unchanged)
// ===================================================================

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000UL;
constexpr uint16_t I2C_TIMEOUT_MS = 50U;

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr uint8_t MPU6050_WHO_AM_I_VALUE = 0x68;
constexpr size_t MPU_FRAME_BYTES = 14U;
constexpr float ACCEL_LSB_PER_G = 4096.0f;   // plot14: +/-8 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // plot14: +/-500 dps
constexpr unsigned long IMU_READ_INTERVAL_MS = 100UL;
constexpr unsigned long IMU_REINITIALIZE_INTERVAL_MS = 1000UL;
constexpr uint8_t IMU_FAILURES_BEFORE_REINITIALIZE = 3U;

// Keep the safe default. If a physically verified compatible clone reports a
// different value, enable this option and enter that exact value. Arbitrary
// identities are never accepted, and reserved WHO_AM_I bits must still be 0.
#ifndef MPU6050_ENABLE_ALTERNATE_WHO_AM_I
#define MPU6050_ENABLE_ALTERNATE_WHO_AM_I 1
#endif
#ifndef MPU6050_ALTERNATE_WHO_AM_I_VALUE
#define MPU6050_ALTERNATE_WHO_AM_I_VALUE 0x70U  // MPU-6500 compatible mode
#endif

ImuStaticDetectionConfig imuConfig;  // Thresholds are set in setup() below, from prior on-hardware measurements.
bool wireReady = false;
bool imuReady = false;
uint8_t consecutiveReadFailures = 0U;

// [v1.2.6 신규] 자이로 바이어스 보정 — 이전엔 이 로직이 아예 없어서 원시
// 자이로 값을 그대로 썼음. imuTask 시작 시(및 재초기화 성공 시) 정지 상태를
// 가정하고 GYRO_BIAS_SAMPLES개를 평균 내 바이어스로 삼고, 이후 매 프레임
// readImuAndEvaluate()에서 이 값을 빼고 각속도 크기를 계산한다.
float gyroBiasDpsX = 0.0f, gyroBiasDpsY = 0.0f, gyroBiasDpsZ = 0.0f;
constexpr int GYRO_BIAS_SAMPLE_COUNT = 50;
constexpr int GYRO_BIAS_MIN_VALID_SAMPLES = 45;
constexpr float GYRO_BIAS_MAX_AXIS_STD_DPS = 5.0f;

// Gate 1: turns the IMU's instantaneous isStaticNow into a debounced,
// minimum-duration-confirmed "eligible" flag (checklist item 3 needs this,
// not the raw isStaticNow from evaluateImuStaticNow()).
StaticSegmentGate imuStaticGate;
StaticSegmentConfig imuSegmentConfig;
// Declared before the IMU helper functions because initialization error paths
// use it for non-interleaved Serial output.
SemaphoreHandle_t serialMutex = nullptr;

void discardWireInput() {
  while (Wire.available() > 0) {
    (void)Wire.read();
  }
}

void prepareWireForRestart() {
  discardWireInput();
#if defined(WIRE_HAS_END)
  if (wireReady) (void)Wire.end();
#endif
  wireReady = false;
}

bool writeMpuRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  const bool buffered =
      Wire.write(registerAddress) == 1U && Wire.write(value) == 1U;
  const uint8_t result = Wire.endTransmission(true);
  return buffered && result == 0U;
}

bool readMpuRegister(uint8_t registerAddress, uint8_t& value) {
  Wire.beginTransmission(MPU_ADDRESS);
  const bool addressBuffered = Wire.write(registerAddress) == 1U;
  const uint8_t transmissionResult = Wire.endTransmission(false);
  if (!addressBuffered || transmissionResult != 0U) {
    discardWireInput();
    return false;
  }

  const size_t received = Wire.requestFrom(
      MPU_ADDRESS, static_cast<uint8_t>(1U), true);
  if (received != 1U || Wire.available() < 1) {
    discardWireInput();
    return false;
  }

  const int byteValue = Wire.read();
  discardWireInput();
  if (byteValue < 0) return false;

  value = static_cast<uint8_t>(byteValue);
  return true;
}

bool readInt16BigEndian(int16_t& value) {
  if (Wire.available() < 2) return false;

  const int highByte = Wire.read();
  const int lowByte = Wire.read();
  if (highByte < 0 || lowByte < 0) return false;

  const uint16_t raw =
      (static_cast<uint16_t>(highByte) << 8) |
      static_cast<uint16_t>(lowByte);
  const int32_t signedValue =
      (raw & 0x8000U) != 0U
          ? static_cast<int32_t>(raw) - 0x10000L
          : static_cast<int32_t>(raw);
  value = static_cast<int16_t>(signedValue);
  return true;
}

bool isAcceptedWhoAmI(uint8_t identity) {
  const bool reservedBitsValid = (identity & 0x81U) == 0U;
  if (!reservedBitsValid || identity == 0U) return false;
  if (identity == MPU6050_WHO_AM_I_VALUE) return true;
#if MPU6050_ENABLE_ALTERNATE_WHO_AM_I
  return identity ==
      static_cast<uint8_t>(MPU6050_ALTERNATE_WHO_AM_I_VALUE);
#else
  return false;
#endif
}

bool initializeMpu6050() {
  if (!writeMpuRegister(REG_PWR_MGMT_1, 0x80U)) return false;
  delay(100);

  if (!writeMpuRegister(REG_PWR_MGMT_1, 0x01U) ||
      !writeMpuRegister(REG_PWR_MGMT_2, 0x00U) ||
      !writeMpuRegister(REG_CONFIG, 0x04U) ||          // plot14: ~21/20 Hz DLPF
      !writeMpuRegister(REG_SMPLRT_DIV, 9U) ||         // internal 100 Hz
      !writeMpuRegister(REG_GYRO_CONFIG, 0x08U) ||     // plot14: +/-500 dps
      !writeMpuRegister(REG_ACCEL_CONFIG, 0x10U)) {    // plot14: +/-8 g
    return false;
  }

  uint8_t identity = 0U;
  uint8_t power1 = 0U;
  uint8_t power2 = 0U;
  uint8_t filterConfig = 0U;
  uint8_t sampleDivider = 0U;
  uint8_t gyroConfig = 0U;
  uint8_t accelConfig = 0U;
  if (!readMpuRegister(REG_WHO_AM_I, identity) ||
      !readMpuRegister(REG_PWR_MGMT_1, power1) ||
      !readMpuRegister(REG_PWR_MGMT_2, power2) ||
      !readMpuRegister(REG_CONFIG, filterConfig) ||
      !readMpuRegister(REG_SMPLRT_DIV, sampleDivider) ||
      !readMpuRegister(REG_GYRO_CONFIG, gyroConfig) ||
      !readMpuRegister(REG_ACCEL_CONFIG, accelConfig)) {
    return false;
  }

  if (!isAcceptedWhoAmI(identity)) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.print("MPU6050_WHO_AM_I_REJECTED,0x");
      Serial.println(identity, HEX);
      xSemaphoreGive(serialMutex);
    }
    return false;
  }
  if (identity != MPU6050_WHO_AM_I_VALUE) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.print("MPU6050_ALTERNATE_WHO_AM_I_ACCEPTED,0x");
      Serial.println(identity, HEX);
      xSemaphoreGive(serialMutex);
    }
  }

  return power1 == 0x01U &&
         power2 == 0x00U &&
         filterConfig == 0x04U &&
         sampleDivider == 9U &&
         gyroConfig == 0x08U &&
         accelConfig == 0x10U;
}

bool readMpuFrame(int16_t& axRaw, int16_t& ayRaw, int16_t& azRaw,
                  int16_t& gxRaw, int16_t& gyRaw, int16_t& gzRaw) {
  Wire.beginTransmission(MPU_ADDRESS);
  const bool addressBuffered = Wire.write(REG_ACCEL_XOUT_H) == 1U;
  const uint8_t transmissionResult = Wire.endTransmission(false);
  if (!addressBuffered || transmissionResult != 0U) {
    discardWireInput();
    return false;
  }

  const size_t received = Wire.requestFrom(
      MPU_ADDRESS, static_cast<uint8_t>(MPU_FRAME_BYTES), true);
  if (received != MPU_FRAME_BYTES ||
      Wire.available() < static_cast<int>(MPU_FRAME_BYTES)) {
    discardWireInput();
    return false;
  }

  int16_t temperatureRaw = 0;
  const bool decoded =
      readInt16BigEndian(axRaw) &&
      readInt16BigEndian(ayRaw) &&
      readInt16BigEndian(azRaw) &&
      readInt16BigEndian(temperatureRaw) &&
      readInt16BigEndian(gxRaw) &&
      readInt16BigEndian(gyRaw) &&
      readInt16BigEndian(gzRaw);
  (void)temperatureRaw;
  discardWireInput();
  return decoded;
}

ImuStaticEvaluation communicationErrorEvaluation() {
  return evaluateImuStaticNow(false, 0.0f, 0.0f, imuConfig);
}

// [v1.2.6 신규] readMpuFrame()으로 GYRO_BIAS_SAMPLES회 읽어 평균을 바이어스로
// 채택한다. 호출 시점(imuTask 시작 직후/재초기화 직후)엔 착용자가 가만히
// 있다고 가정 — 실제로 움직이는 중에 호출되면 바이어스가 오염된다.
bool calibrateGyroBias() {
  double meanX = 0.0, meanY = 0.0, meanZ = 0.0;
  double m2X = 0.0, m2Y = 0.0, m2Z = 0.0;
  int gotSamples = 0;

  for (int i = 0; i < GYRO_BIAS_SAMPLE_COUNT; i++) {
    int16_t axRaw = 0, ayRaw = 0, azRaw = 0, gxRaw = 0, gyRaw = 0, gzRaw = 0;
    if (readMpuFrame(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
      const float axG = static_cast<float>(axRaw) / ACCEL_LSB_PER_G;
      const float ayG = static_cast<float>(ayRaw) / ACCEL_LSB_PER_G;
      const float azG = static_cast<float>(azRaw) / ACCEL_LSB_PER_G;
      const float accelMagnitudeG = calculateAccelMagnitudeG(axG, ayG, azG);
      // 움직임 중 얻은 값을 바이어스로 빼면 실제 회전을 정지로 오판할 수
      // 있으므로, 중력 크기 조건을 벗어난 프레임은 보정 표본에서 제외합니다.
      if (!isfinite(accelMagnitudeG) ||
          fabsf(accelMagnitudeG - imuConfig.gravityG) >
              imuConfig.accelStaticDeviationG) {
        delay(5);
        continue;
      }

      ++gotSamples;
      const double x = static_cast<double>(gxRaw);
      const double y = static_cast<double>(gyRaw);
      const double z = static_cast<double>(gzRaw);
      const double dx = x - meanX;
      const double dy = y - meanY;
      const double dz = z - meanZ;
      meanX += dx / static_cast<double>(gotSamples);
      meanY += dy / static_cast<double>(gotSamples);
      meanZ += dz / static_cast<double>(gotSamples);
      m2X += dx * (x - meanX);
      m2Y += dy * (y - meanY);
      m2Z += dz * (z - meanZ);
    }
    delay(5);
  }

  bool calibrationOk = gotSamples >= GYRO_BIAS_MIN_VALID_SAMPLES;
  float stdX = 0.0f, stdY = 0.0f, stdZ = 0.0f;
  if (calibrationOk) {
    const double divisor = static_cast<double>(gotSamples - 1);
    stdX = static_cast<float>(sqrt(m2X / divisor)) / GYRO_LSB_PER_DPS;
    stdY = static_cast<float>(sqrt(m2Y / divisor)) / GYRO_LSB_PER_DPS;
    stdZ = static_cast<float>(sqrt(m2Z / divisor)) / GYRO_LSB_PER_DPS;
    calibrationOk = isfinite(stdX) && isfinite(stdY) && isfinite(stdZ) &&
                    stdX <= GYRO_BIAS_MAX_AXIS_STD_DPS &&
                    stdY <= GYRO_BIAS_MAX_AXIS_STD_DPS &&
                    stdZ <= GYRO_BIAS_MAX_AXIS_STD_DPS;
  }

  if (calibrationOk) {
    gyroBiasDpsX = static_cast<float>(meanX) / GYRO_LSB_PER_DPS;
    gyroBiasDpsY = static_cast<float>(meanY) / GYRO_LSB_PER_DPS;
    gyroBiasDpsZ = static_cast<float>(meanZ) / GYRO_LSB_PER_DPS;
  } else {
    // 실패한 일부 표본이나 이전 센서 세션의 바이어스를 사용하지 않습니다.
    gyroBiasDpsX = 0.0f;
    gyroBiasDpsY = 0.0f;
    gyroBiasDpsZ = 0.0f;
  }

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (calibrationOk) {
      Serial.print("IMU_GYRO_BIAS,");
      Serial.print(gyroBiasDpsX, 2);
      Serial.print(",");
      Serial.print(gyroBiasDpsY, 2);
      Serial.print(",");
      Serial.print(gyroBiasDpsZ, 2);
      Serial.println("");
    } else {
      Serial.print("IMU_GYRO_BIAS_CALIBRATION_FAILED,samples=");
      Serial.println(gotSamples);
    }
    xSemaphoreGive(serialMutex);
  }
  return calibrationOk;
}

bool initializeWireAndMpu6050() {
  if (!wireReady) {
    wireReady = Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!wireReady) return false;

    Wire.setClock(I2C_CLOCK_HZ);
    if (static_cast<uint32_t>(Wire.getClock()) != I2C_CLOCK_HZ) {
      prepareWireForRestart();
      return false;
    }
    Wire.setTimeOut(I2C_TIMEOUT_MS);
  }

  if (!initializeMpu6050()) {
    prepareWireForRestart();
    return false;
  }
  return true;
}

ImuStaticEvaluation readImuAndEvaluate() {
  if (!imuReady) return communicationErrorEvaluation();

  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;
  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;
  if (!readMpuFrame(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
    if (consecutiveReadFailures < 0xFFU) ++consecutiveReadFailures;
    if (consecutiveReadFailures >= IMU_FAILURES_BEFORE_REINITIALIZE) {
      imuReady = false;
      prepareWireForRestart();
    }
    return communicationErrorEvaluation();
  }

  consecutiveReadFailures = 0U;

  const float axG = static_cast<float>(axRaw) / ACCEL_LSB_PER_G;
  const float ayG = static_cast<float>(ayRaw) / ACCEL_LSB_PER_G;
  const float azG = static_cast<float>(azRaw) / ACCEL_LSB_PER_G;
  // [v1.2.6 수정] 바이어스를 빼지 않은 원시값을 그대로 썼던 문제 수정.
  const float gxDps = static_cast<float>(gxRaw) / GYRO_LSB_PER_DPS - gyroBiasDpsX;
  const float gyDps = static_cast<float>(gyRaw) / GYRO_LSB_PER_DPS - gyroBiasDpsY;
  const float gzDps = static_cast<float>(gzRaw) / GYRO_LSB_PER_DPS - gyroBiasDpsZ;

  const float accelMagnitudeG = calculateAccelMagnitudeG(axG, ayG, azG);
  const float gyroMagnitudeDps =
      calculateGyroMagnitudeDps(gxDps, gyDps, gzDps);
  return evaluateImuStaticNow(
      true, accelMagnitudeG, gyroMagnitudeDps, imuConfig);
}

// ===================================================================
// Part 2: RMS (from rms_emg_adapter_v1_2_5_fixed.ino, unchanged)
// ===================================================================

constexpr int EMG_ADC_PIN = 34;  // Classic ESP32 ADC1 pin; change for another board.

constexpr bool configuredPinsConflict(int first, int second) {
  return first >= 0 && second >= 0 && first == second;
}

static_assert((STAGE5_SYSTEM_TM1637_CLK_PIN >= 0) ==
                  (STAGE5_SYSTEM_TM1637_DIO_PIN >= 0),
              "TM1637 CLK and DIO must both be configured or both disabled");
static_assert(!configuredPinsConflict(STAGE5_SYSTEM_VIBRATION_PIN,
                                      STAGE5_SYSTEM_HEATER_PIN),
              "Vibration and heater outputs cannot share a pin");
static_assert(!configuredPinsConflict(STAGE5_SYSTEM_TM1637_CLK_PIN,
                                      STAGE5_SYSTEM_TM1637_DIO_PIN),
              "TM1637 CLK and DIO cannot share a pin");
static_assert(!configuredPinsConflict(STAGE5_SYSTEM_VIBRATION_PIN,
                                      EMG_ADC_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_VIBRATION_PIN,
                                          I2C_SDA_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_VIBRATION_PIN,
                                          I2C_SCL_PIN),
              "Vibration output conflicts with an acquisition pin");
static_assert(!configuredPinsConflict(STAGE5_SYSTEM_HEATER_PIN,
                                      EMG_ADC_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_HEATER_PIN,
                                          I2C_SDA_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_HEATER_PIN,
                                          I2C_SCL_PIN),
              "Heater output conflicts with an acquisition pin");
static_assert(!configuredPinsConflict(STAGE5_SYSTEM_TM1637_CLK_PIN,
                                      EMG_ADC_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_TM1637_CLK_PIN,
                                          I2C_SDA_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_TM1637_CLK_PIN,
                                          I2C_SCL_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_TM1637_DIO_PIN,
                                          EMG_ADC_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_TM1637_DIO_PIN,
                                          I2C_SDA_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_TM1637_DIO_PIN,
                                          I2C_SCL_PIN),
              "TM1637 pins conflict with an acquisition pin");
static_assert(!configuredPinsConflict(STAGE5_SYSTEM_VIBRATION_PIN,
                                      STAGE5_SYSTEM_TM1637_CLK_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_VIBRATION_PIN,
                                          STAGE5_SYSTEM_TM1637_DIO_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_HEATER_PIN,
                                          STAGE5_SYSTEM_TM1637_CLK_PIN) &&
                  !configuredPinsConflict(STAGE5_SYSTEM_HEATER_PIN,
                                          STAGE5_SYSTEM_TM1637_DIO_PIN),
              "Stage-5 peripherals cannot share configured pins");
constexpr int EMG_ADC_BITS = 12;
constexpr int EMG_ADC_MAX_COUNT = (1 << EMG_ADC_BITS) - 1;
constexpr int EMG_SAMPLE_COUNT = 1024;
constexpr uint32_t EMG_SAMPLE_RATE_HZ = 1000UL;
constexpr uint32_t EMG_SAMPLE_PERIOD_US =
    1000000UL / EMG_SAMPLE_RATE_HZ;
constexpr uint32_t EMG_SAMPLE_PERIOD_REMAINDER =
    1000000UL % EMG_SAMPLE_RATE_HZ;
constexpr uint32_t EMG_MAX_SAMPLE_PERIOD_US =
    EMG_SAMPLE_PERIOD_US +
    (EMG_SAMPLE_PERIOD_REMAINDER == 0UL ? 0UL : 1UL);
constexpr uint32_t MAX_SAMPLE_LATENESS_US = 5000UL;  // plot14 resync limit
constexpr int BASELINE_BLOCK_COUNT = 40;
constexpr unsigned long BASELINE_CALIBRATION_MS = 10000UL;
constexpr int MAX_SATURATED_SAMPLES = EMG_SAMPLE_COUNT / 20;  // 5%.
// 평탄선과 레일에 닿지 않는 극단적 단일 접촉 스파이크를 차단하는 보수적
// 디지털 검사입니다. 실제 전극 임피던스 검사를 대신하지 않으며 실측 조정이
// 필요합니다.
constexpr int MIN_EMG_PEAK_TO_PEAK_COUNTS = 2;
constexpr float MAX_EMG_CREST_FACTOR = 8.0f;
constexpr int MAX_SERIAL_BYTES_PER_LOOP = 8;

enum RmsAdapterStatus {
  RMS_COLLECTING = 0,
  RMS_CALIBRATING,
  RMS_OK,
  RMS_TIMING_ERROR,
  RMS_ADC_ERROR,
  RMS_BASELINE_INVALID,
  RMS_CONFIG_ERROR,
  // [v1.2.6 신규] checkCalibQuality()가 이상치/CV 기준으로 베이스라인을
  // 거부했을 때. RMS_BASELINE_INVALID(값 자체가 NaN 등 수학적으로 무효)와
  // 구분되는 별도 상태 — 값은 유효하지만 "신뢰할 수 없는" 경우.
  RMS_BASELINE_QUALITY_FAILED,
  RMS_SIGNAL_QUALITY_ERROR
};

enum EmgSamplingEvent {
  EMG_NO_EVENT = 0,
  EMG_WINDOW_READY,
  EMG_TIMING_FAULT,
  EMG_ADC_FAULT,
  EMG_SIGNAL_QUALITY_FAULT
};

void resetEmgWindow();
void discardIncompleteBaselineCalibration();
void setRmsAcquisitionFault(RmsAdapterStatus status);
void restartRmsCalibration();
void beginRmsCalibrationCountdown(unsigned long nowMs, const char* source);
void updateRmsCalibrationCountdown(unsigned long nowMs);
bool addBaselineRms(float rms);
void processRmsWindow(float rms);
bool isEmgWindowSignalQualityAcceptable();
void removeWindowDcOffset();
void advanceNextSampleTime();
EmgSamplingEvent updateEmgSampling();
void printRmsState();
void handleSerialCommands();
// [신규] stage5의 자동 재캘리브레이션(온열 개입 종료 직후)이 진행 중인지
// 확인한다. handleSerialCommands()가 이걸 확인해서, 겹치는 동안엔 수동
// 'C'를 무시한다 - 아래 정의(stage5SystemController 선언 이후)와 실제
// 사용(handleSerialCommands, 이 함수보다 파일상 앞쪽) 사이의 순서를
// 맞추기 위한 전방선언.
bool isManualRmsRecalibrationBlocked();

float emgSamples[EMG_SAMPLE_COUNT];
int emgSampleIndex = 0;
int saturatedSampleCount = 0;
uint32_t nextSampleUs = 0UL;
uint32_t samplePeriodRemainder = 0UL;
bool sampleScheduleStarted = false;

RmsContractionGate rmsGate;
RmsContractionConfig rmsConfig;
// [v1.2.6 수정] Welford 누적 방식의 BaselineAccumulator를 없애고, 원본
// 블록별 RMS 값을 그대로 모아뒀다가 checkCalibQuality()(이상치/CV 검사
// 포함)로 한 번에 평균/표준편차를 구하는 방식으로 바꿈.
float baselineSamples[BASELINE_BLOCK_COUNT];
int baselineSampleCount = 0;
volatile bool mdfBaselineReady = false;
volatile bool mdfBaselineFailed = false;
volatile float baselineMdfHz = 0.0f;
unsigned long baselineCalibrationStartMs = 0UL;
double baselineOneSecondRmsSquareSum = 0.0;
uint8_t baselineQuarterSecondBlockCount = 0U;

// plot14's 60 Hz Q=10 time-domain notch. RMS and MDF both consume this
// filtered signal before DC removal.
constexpr double EMG_NOTCH_FREQ_HZ = 60.0;
constexpr double EMG_NOTCH_Q = 10.0;
double notchB0 = 1.0, notchB1 = 0.0, notchB2 = 0.0;
double notchA1 = 0.0, notchA2 = 0.0;
double notchX1 = 0.0, notchX2 = 0.0, notchY1 = 0.0, notchY2 = 0.0;

void initEmgNotchFilter() {
  const double w0 = 2.0 * 3.14159265358979323846 * EMG_NOTCH_FREQ_HZ /
                    static_cast<double>(EMG_SAMPLE_RATE_HZ);
  const double alpha = sin(w0) / (2.0 * EMG_NOTCH_Q);
  const double cosw0 = cos(w0);
  const double a0 = 1.0 + alpha;
  notchB0 = 1.0 / a0;
  notchB1 = (-2.0 * cosw0) / a0;
  notchB2 = 1.0 / a0;
  notchA1 = (-2.0 * cosw0) / a0;
  notchA2 = (1.0 - alpha) / a0;
}

float applyEmgNotchFilter(float input) {
  const double x0 = static_cast<double>(input);
  const double y0 = notchB0 * x0 + notchB1 * notchX1 + notchB2 * notchX2 -
                    notchA1 * notchY1 - notchA2 * notchY2;
  notchX2 = notchX1;
  notchX1 = x0;
  notchY2 = notchY1;
  notchY1 = y0;
  return static_cast<float>(y0);
}

float currentRms = 0.0f;
float baselineRmsMean = 0.0f;
float baselineRmsStd = 0.0f;
bool baselineReady = false;
bool baselineFailed = false;
uint8_t baselineSignalQualityErrorCount = 0U;
bool baselineContactWarningActive = false;
constexpr uint8_t BASELINE_CONTACT_WARNING_BLOCKS = 2U;
bool plot14ContactBad = false;
uint8_t plot14ContactBadCount = 0U;
uint8_t plot14ContactGoodCount = 0U;
bool isContracting = false;
RmsAdapterStatus rmsStatus = RMS_COLLECTING;
bool serialRecalibrationDrainActive = false;
volatile bool stage5FatigueInterventionPending = false;
bool heaterPausedByUser = false;
volatile bool heaterPauseTogglePending = false;
enum DirectInterventionState {
  DIRECT_INTERVENTION_IDLE = 0,
  DIRECT_INTERVENTION_VIBRATING,
  DIRECT_INTERVENTION_HEATING,
  DIRECT_INTERVENTION_COMPLETE
};
volatile DirectInterventionState directInterventionState =
    DIRECT_INTERVENTION_IDLE;
unsigned long directInterventionStateStartMs = 0UL;
bool directHeaterDemand = false;
// Owned by core 1. Version 0 means that no valid RMS baseline has ever been
// published. A successful initial calibration or atomic stage-5 replacement
// increments the version.
uint32_t rmsBaselineVersion = 0U;
bool rmsBaselineRecalibratedPulse = false;
constexpr unsigned long RMS_CALIBRATION_COUNTDOWN_MS = 3000UL;
bool rmsCalibrationCountdownActive = false;
unsigned long rmsCalibrationCountdownStartMs = 0UL;
int lastRmsCountdownValue = -1;
uint32_t manualFatigueSessionGeneration = 0U;

void resetEmgWindow() {
  emgSampleIndex = 0;
  saturatedSampleCount = 0;
  samplePeriodRemainder = 0UL;
  sampleScheduleStarted = false;
}

// [v1.2.6 수정] baselineFailed는 여기서 더 이상 자동으로 풀지 않는다 —
// 원래 이 함수는 setRmsAcquisitionFault()(타이밍/ADC 오류)에서도 호출되는데,
// 예전엔 baselineFailed까지 같이 초기화해버려서 품질검사 실패
// (RMS_BASELINE_QUALITY_FAILED) 이후 우연히 타이밍 오류 한 번만 나도
// 사용자가 'C'를 다시 보내지 않았는데도 조용히 새 베이스라인 수집이
// 시작되는 문제가 있었다. baselineFailed 해제는 이제 restartRmsCalibration()
// ('C' 명령 처리)에서만 일어난다.
void discardIncompleteBaselineCalibration() {
  if (baselineReady) return;
  baselineSampleCount = 0;
  baselineCalibrationStartMs = 0UL;
  baselineOneSecondRmsSquareSum = 0.0;
  baselineQuarterSecondBlockCount = 0U;
  baselineRmsMean = 0.0f;
  baselineRmsStd = 0.0f;
}

void setRmsAcquisitionFault(RmsAdapterStatus status) {
  if (!baselineReady && status == RMS_SIGNAL_QUALITY_ERROR) {
    if (baselineSignalQualityErrorCount < 255U) {
      ++baselineSignalQualityErrorCount;
    }
    if (!baselineContactWarningActive &&
        baselineSignalQualityErrorCount >=
            BASELINE_CONTACT_WARNING_BLOCKS) {
      baselineContactWarningActive = true;
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("접촉불량,전극 접촉 또는 EMG 신호를 확인하세요");
        xSemaphoreGive(serialMutex);
      }
    }
  }
  resetEmgWindow();
  // A noisy/flat EMG window is skipped, but already accepted baseline
  // windows are retained. Resetting all samples here caused a repeating
  // 1/40 -> 2/40 -> 1/40 loop whenever contact was intermittently bad.
  if (baselineReady && status != RMS_SIGNAL_QUALITY_ERROR) {
    discardIncompleteBaselineCalibration();
  }
  resetRmsContractionGate(rmsGate);
  currentRms = 0.0f;
  isContracting = false;
  rmsStatus = status;
}

void restartRmsCalibration() {
  baselineSampleCount = 0;
  baselineCalibrationStartMs = 0UL;
  baselineOneSecondRmsSquareSum = 0.0;
  baselineQuarterSecondBlockCount = 0U;
  baselineRmsMean = 0.0f;
  baselineRmsStd = 0.0f;
  mdfBaselineReady = false;
  mdfBaselineFailed = false;
  baselineMdfHz = 0.0f;
  baselineReady = false;
  baselineFailed = false;
  baselineSignalQualityErrorCount = 0U;
  baselineContactWarningActive = false;
  plot14ContactBad = false;
  plot14ContactBadCount = 0U;
  plot14ContactGoodCount = 0U;
  currentRms = 0.0f;
  isContracting = false;
  rmsBaselineRecalibratedPulse = false;
  resetRmsContractionGate(rmsGate);
  resetEmgWindow();
  rmsStatus = RMS_CALIBRATING;
}

void printRmsCountdownLine(int seconds) {
  char line[32];
  const int length = snprintf(line, sizeof(line),
                              "CALIB_COUNTDOWN,%d\n", seconds);
  if (length <= 0 || length >= static_cast<int>(sizeof(line))) return;
  if (serialMutex == nullptr) {
    Serial.write(reinterpret_cast<const uint8_t*>(line),
                 static_cast<size_t>(length));
    return;
  }
  if (xSemaphoreTake(serialMutex, 0) == pdTRUE) {
    if (Serial.availableForWrite() >= length) {
      Serial.write(reinterpret_cast<const uint8_t*>(line),
                   static_cast<size_t>(length));
    }
    xSemaphoreGive(serialMutex);
  }
}

void beginRmsCalibrationCountdown(unsigned long nowMs, const char* source) {
  (void)source;
  ++manualFatigueSessionGeneration;
  if (manualFatigueSessionGeneration == 0U) {
    ++manualFatigueSessionGeneration;
  }
  restartRmsCalibration();
  directInterventionState = DIRECT_INTERVENTION_IDLE;
  directInterventionStateStartMs = 0UL;
  directHeaterDemand = false;
  stage5FatigueInterventionPending = false;
  heaterPausedByUser = false;
  rmsCalibrationCountdownActive = true;
  rmsCalibrationCountdownStartMs = nowMs;
  lastRmsCountdownValue = 3;
  printRmsCountdownLine(3);
}

void updateRmsCalibrationCountdown(unsigned long nowMs) {
  if (!rmsCalibrationCountdownActive) return;
  const unsigned long elapsed = nowMs - rmsCalibrationCountdownStartMs;
  if (elapsed >= RMS_CALIBRATION_COUNTDOWN_MS) {
    rmsCalibrationCountdownActive = false;
    lastRmsCountdownValue = 0;
    // Discard every block seen during the countdown. The ten-second baseline
    // begins only after the countdown has completely finished.
    restartRmsCalibration();
    if (serialMutex != nullptr &&
        xSemaphoreTake(serialMutex, 0) == pdTRUE) {
      Serial.println("RMS_RECALIBRATING");
      Serial.println("기준값측정시작,약10초간 힘을 빼고 움직이지 마세요");
      xSemaphoreGive(serialMutex);
    }
    return;
  }
  const int seconds = 3 - static_cast<int>(elapsed / 1000UL);
  if (seconds != lastRmsCountdownValue) {
    lastRmsCountdownValue = seconds;
    printRmsCountdownLine(seconds);
  }
}

// [v1.2.6 수정] 원본 블록별 RMS 값을 모아뒀다가 다 채워지면
// checkCalibQuality()(중앙값+MAD 기반 이상치 검사, CV 검사)를 한 번
// 돌려서 평균/표준편차를 구함. 품질 검사를 통과 못 하면(접촉불량/노이즈로
// 오염된 것으로 판단) baselineFailed=true로 남기고 기준값을 채택하지
// 않는다 — 이 상태에서는 processRmsWindow()가 더 이상 이 함수를 호출하지
// 않고 'C' 재전송을 기다린다.
BaselineQualityResult evaluatePlot14BaselineQuality(const float* data, int n) {
  BaselineQualityResult result;
  if (data == nullptr || n <= 0 || n > BASELINE_BLOCK_COUNT) return result;

  double sorted[BASELINE_BLOCK_COUNT];
  for (int i = 0; i < n; ++i) {
    if (!isfinite(data[i]) || data[i] < 0.0f) return result;
    sorted[i] = static_cast<double>(data[i]);
  }
  for (int i = 1; i < n; ++i) {
    const double key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = key;
  }
  const double median = (n % 2 == 0)
      ? (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5
      : sorted[n / 2];

  double deviations[BASELINE_BLOCK_COUNT];
  for (int i = 0; i < n; ++i) {
    deviations[i] = fabs(static_cast<double>(data[i]) - median);
  }
  for (int i = 1; i < n; ++i) {
    const double key = deviations[i];
    int j = i - 1;
    while (j >= 0 && deviations[j] > key) {
      deviations[j + 1] = deviations[j];
      --j;
    }
    deviations[j + 1] = key;
  }
  double mad = ((n % 2 == 0)
      ? (deviations[n / 2 - 1] + deviations[n / 2]) * 0.5
      : deviations[n / 2]) * 1.4826;
  const double minMad = fabs(median) * 0.005;
  if (mad < minMad) mad = minMad;
  if (mad < 1e-6) mad = 1e-6;

  int outlierCount = 0;
  double sum = 0.0;
  double sumSquares = 0.0;
  for (int i = 0; i < n; ++i) {
    const double value = static_cast<double>(data[i]);
    if (fabs(value - median) > 3.0 * mad) ++outlierCount;
    sum += value;
    sumSquares += value * value;
  }
  const double mean = sum / static_cast<double>(n);
  double variance = sumSquares / static_cast<double>(n) - mean * mean;
  if (variance < 0.0 && variance > -1e-12) variance = 0.0;
  const double standardDeviation = variance > 0.0 ? sqrt(variance) : 0.0;
  const double coefficientOfVariation = mean > 0.0
      ? standardDeviation / mean : 999.0;

  result.mean = static_cast<float>(mean);
  result.std = static_cast<float>(standardDeviation);
  result.outlierCount = outlierCount;
  result.ok = isfinite(mean) && mean > 0.0 &&
              isfinite(standardDeviation) &&
              coefficientOfVariation <= 0.5 && outlierCount <= 1;
  return result;
}

bool addBaselineRms(float rms) {
  if (!isfinite(rms) || rms < 0.0f || baselineReady || baselineFailed) {
    return false;
  }

  const unsigned long nowMs = millis();
  if (baselineCalibrationStartMs == 0UL) baselineCalibrationStartMs = nowMs;
  if (baselineSampleCount < BASELINE_BLOCK_COUNT) {
    baselineSamples[baselineSampleCount++] = rms;
  }
  // Never block the 1000 Hz EMG sampling core for progress display. A long
  // sequence of Serial.print() calls here filled the UART buffer after one or
  // two windows and caused the next sample deadline to be missed, producing a
  // repeating 1/40 -> 2/40 calibration loop. Emit one short line only when the
  // entire line already fits in the UART TX buffer; otherwise skip this update.
  char progressLine[40];
  const int progressLength = snprintf(
      progressLine, sizeof(progressLine), "CALIB_TIME,%lu,%lu\n",
      nowMs - baselineCalibrationStartMs, BASELINE_CALIBRATION_MS);
  if (progressLength > 0 && progressLength < static_cast<int>(sizeof(progressLine)) &&
      xSemaphoreTake(serialMutex, 0) == pdTRUE) {
    if (Serial.availableForWrite() >= progressLength) {
      Serial.write(reinterpret_cast<const uint8_t*>(progressLine),
                   static_cast<size_t>(progressLength));
    }
    xSemaphoreGive(serialMutex);
  }
  if (nowMs - baselineCalibrationStartMs < BASELINE_CALIBRATION_MS) return true;
  if (baselineSampleCount <= 0) return false;

  const BaselineQualityResult quality =
      evaluatePlot14BaselineQuality(baselineSamples, baselineSampleCount);

  if (!quality.ok) {
    baselineFailed = true;
    baselineReady = false;
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.print("RMS_BASELINE_QUALITY_FAILED,mean=");
      Serial.print(quality.mean, 4);
      Serial.print(",std=");
      Serial.print(quality.std, 4);
      Serial.print(",outliers=");
      Serial.println(quality.outlierCount);
      Serial.println("기준값측정실패,전극 접촉과 주변 노이즈를 확인한 뒤 확인 버튼을 다시 누르세요");
      xSemaphoreGive(serialMutex);
    }
    return false;
  }

  baselineRmsMean = quality.mean;
  baselineRmsStd = quality.std;
  baselineReady = true;
  baselineFailed = false;
  baselineSignalQualityErrorCount = 0U;
  const bool recoveredFromContactWarning = baselineContactWarningActive;
  baselineContactWarningActive = false;
  baselineCalibrationStartMs = 0UL;
  ++rmsBaselineVersion;
  if (rmsBaselineVersion == 0U) ++rmsBaselineVersion;
  rmsBaselineRecalibratedPulse = true;
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (recoveredFromContactWarning) {
      Serial.println("접촉정상,유효한 기준값 수집을 완료했습니다");
    }
    Serial.print("기준값측정완료,평균RMS=");
    Serial.print(baselineRmsMean, 6);
    Serial.print(",표준편차=");
    Serial.println(baselineRmsStd, 6);
    xSemaphoreGive(serialMutex);
  }
  return true;
}

void processRmsWindow(float rms) {
  if (!isfinite(rms) || rms < 0.0f) {
    setRmsAcquisitionFault(RMS_ADC_ERROR);
    return;
  }

  currentRms = rms;

  // 기준값 측정 중 접촉 불량은 누적 횟수가 아니라 연속 불량 블록으로만
  // 판단한다. 정상 RMS 블록이 하나라도 들어오면 이전 불량 이력을 지운다.
  if (!baselineReady) {
    baselineSignalQualityErrorCount = 0U;
    baselineContactWarningActive = false;
  }

  if (rmsCalibrationCountdownActive) {
    resetRmsContractionGate(rmsGate);
    isContracting = false;
    rmsStatus = RMS_CALIBRATING;
    return;
  }

  if (!isRmsContractionConfigValid(rmsConfig)) {
    resetRmsContractionGate(rmsGate);
    isContracting = false;
    rmsStatus = RMS_CONFIG_ERROR;
    return;
  }

  if (!baselineReady) {
    resetRmsContractionGate(rmsGate);
    isContracting = false;
    // [v1.2.6 수정] baselineFailed(품질 검사 탈락)를 더 이상 addBaselineRms()
    // 무효값 실패(RMS_BASELINE_INVALID)와 뭉뚱그리지 않는다 — 이미 실패가
    // 확정된 상태에서는 addBaselineRms를 다시 부르지 않고(불러도 배열이 꽉
    // 차 있어 항상 false) 곧장 RMS_BASELINE_QUALITY_FAILED로 표시해
    // 사용자가 'C'로 재캘리브레이션해야 함을 알 수 있게 한다.
    if (baselineFailed) {
      rmsStatus = RMS_BASELINE_QUALITY_FAILED;
    } else {
      // Each block is already plot14's 1024 samples at 1000 Hz, so its RMS is
      // inserted directly into the fixed ten-second calibration collection.
      if (addBaselineRms(rms)) {
        rmsStatus = baselineReady ? RMS_OK : RMS_CALIBRATING;
      } else {
        rmsStatus = baselineFailed ? RMS_BASELINE_QUALITY_FAILED
                                   : RMS_BASELINE_INVALID;
      }
    }
    return;
  }

  isContracting = updateRmsContractionGate(
      rmsGate, currentRms, baselineRmsMean, baselineRmsStd, rmsConfig);

  // plot14 live contact check: outside 0.3x..8x baseline for two consecutive
  // complete one-second blocks enters CONTACT_BAD.
  const bool contactImplausible =
      currentRms < baselineRmsMean * 0.3f ||
      currentRms > baselineRmsMean * 8.0f;
  if (!plot14ContactBad) {
    plot14ContactBadCount = contactImplausible
        ? static_cast<uint8_t>(plot14ContactBadCount + 1U) : 0U;
    if (plot14ContactBadCount >= 2U) {
      plot14ContactBad = true;
      plot14ContactBadCount = 0U;
      plot14ContactGoodCount = 0U;
      if (xSemaphoreTake(serialMutex, 0) == pdTRUE) {
        Serial.print("EVENT,CONTACT_BAD,");
        Serial.println(millis());
        Serial.println("접촉불량,전극 접촉 또는 EMG 신호를 확인하세요");
        xSemaphoreGive(serialMutex);
      }
    }
  } else {
    plot14ContactGoodCount = !contactImplausible
        ? static_cast<uint8_t>(plot14ContactGoodCount + 1U) : 0U;
    if (plot14ContactGoodCount >= 3U) {
      plot14ContactBad = false;
      plot14ContactBadCount = 0U;
      plot14ContactGoodCount = 0U;
      if (xSemaphoreTake(serialMutex, 0) == pdTRUE) {
        Serial.print("EVENT,CONTACT_OK,");
        Serial.println(millis());
        Serial.println("접촉정상,EMG 신호가 정상 범위로 회복되었습니다");
        xSemaphoreGive(serialMutex);
      }
    }
  }
  rmsStatus = RMS_OK;
}

bool isEmgWindowSignalQualityAcceptable() {
  float minimum = emgSamples[0];
  float maximum = emgSamples[0];
  double sum = 0.0;
  for (int i = 0; i < EMG_SAMPLE_COUNT; ++i) {
    const float sample = emgSamples[i];
    if (!isfinite(sample)) return false;
    if (sample < minimum) minimum = sample;
    if (sample > maximum) maximum = sample;
    sum += static_cast<double>(sample);
  }

  if (maximum - minimum <
      static_cast<float>(MIN_EMG_PEAK_TO_PEAK_COUNTS)) {
    return false;
  }

  const double mean = sum / static_cast<double>(EMG_SAMPLE_COUNT);
  double sumSquares = 0.0;
  double maximumAbsolute = 0.0;
  for (int i = 0; i < EMG_SAMPLE_COUNT; ++i) {
    const double centered = static_cast<double>(emgSamples[i]) - mean;
    sumSquares += centered * centered;
    const double absoluteValue = centered < 0.0 ? -centered : centered;
    if (absoluteValue > maximumAbsolute) maximumAbsolute = absoluteValue;
  }

  const double rmsCounts =
      sqrt(sumSquares / static_cast<double>(EMG_SAMPLE_COUNT));
  if (!isfinite(rmsCounts) || rmsCounts <= 0.0) return false;
  const double crestFactor = maximumAbsolute / rmsCounts;
  return isfinite(crestFactor) &&
         crestFactor <= static_cast<double>(MAX_EMG_CREST_FACTOR);
}

void removeWindowDcOffset() {
  float mean = 0.0f;
  for (int i = 0; i < EMG_SAMPLE_COUNT; ++i) {
    mean += emgSamples[i];
  }
  mean /= EMG_SAMPLE_COUNT;

  for (int i = 0; i < EMG_SAMPLE_COUNT; ++i) {
    emgSamples[i] -= mean;
  }
}

void advanceNextSampleTime() {
  nextSampleUs += EMG_SAMPLE_PERIOD_US;
  samplePeriodRemainder += EMG_SAMPLE_PERIOD_REMAINDER;
  if (samplePeriodRemainder >= EMG_SAMPLE_RATE_HZ) {
    ++nextSampleUs;
    samplePeriodRemainder -= EMG_SAMPLE_RATE_HZ;
  }
}

EmgSamplingEvent updateEmgSampling() {
  const uint32_t nowUs = static_cast<uint32_t>(micros());
  if (!sampleScheduleStarted) {
    nextSampleUs = nowUs;
    sampleScheduleStarted = true;
  }

  const int32_t timeUntilSample =
      static_cast<int32_t>(nowUs - nextSampleUs);
  if (timeUntilSample < 0) return EMG_NO_EVENT;

  const uint32_t latenessUs = nowUs - nextSampleUs;
  // plot14 does not discard a complete window for ordinary loop jitter. If
  // execution fell more than 5 ms behind, resynchronize the next deadline and
  // keep this sample instead of repeatedly resetting the measurement.
  if (latenessUs > MAX_SAMPLE_LATENESS_US) nextSampleUs = nowUs;

  const int raw = analogRead(EMG_ADC_PIN);
  if (raw < 0 || raw > EMG_ADC_MAX_COUNT) {
    setRmsAcquisitionFault(RMS_ADC_ERROR);
    return EMG_ADC_FAULT;
  }

  emgSamples[emgSampleIndex] =
      applyEmgNotchFilter(static_cast<float>(raw));
  ++emgSampleIndex;
  if (raw == 0 || raw == EMG_ADC_MAX_COUNT) ++saturatedSampleCount;
  advanceNextSampleTime();

  if (emgSampleIndex < EMG_SAMPLE_COUNT) return EMG_NO_EVENT;

  removeWindowDcOffset();
  const float rms = calculateRMS(emgSamples, EMG_SAMPLE_COUNT);
  // Keep plot14's continuous 1 kHz deadline across block boundaries. Only the
  // array index is rewound; faults/manual calibration still reset the schedule.
  emgSampleIndex = 0;
  saturatedSampleCount = 0;
  processRmsWindow(rms);
  return EMG_WINDOW_READY;
}

void printRmsState() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;
  Serial.print("status=");
  Serial.print(static_cast<int>(rmsStatus));
  Serial.print(" rms=");
  Serial.print(currentRms, 6);
  Serial.print(" baselineReady=");
  Serial.print(baselineReady ? 1 : 0);
  Serial.print(" baselineMean=");
  Serial.print(baselineRmsMean, 6);
  Serial.print(" baselineStd=");
  Serial.print(baselineRmsStd, 6);
  Serial.print(" isContracting=");
  Serial.println(isContracting ? 1 : 0);
  xSemaphoreGive(serialMutex);
}

void handleSerialCommands() {
  bool recalibrationRequested = false;
  bool heaterPauseToggleRequested = false;
  int processed = 0;
  while (processed < MAX_SERIAL_BYTES_PER_LOOP &&
         Serial.available() > 0) {
    const int incoming = Serial.read();
    ++processed;
    if (!serialRecalibrationDrainActive &&
        (incoming == 'C' || incoming == 'c')) {
      recalibrationRequested = true;
    }
    if (incoming == 'P' || incoming == 'p') {
      heaterPauseToggleRequested = true;
    }
  }

  if (recalibrationRequested) {
    serialRecalibrationDrainActive = true;
    // [신규] stage5가 온열 개입 직후 자체적으로 재캘리브레이션을 진행하는
    // 도중(정지대기~수집~커밋~퍼블리시 확인까지)에는 수동 'C'를 무시한다.
    // 그러지 않으면 기본 시스템의 restartRmsCalibration()과 stage5의
    // PostInterventionRecalibrationController가 서로 독립적으로 동시에
    // 진행되다가, 같은 전역 베이스라인 변수(baselineRmsMean/Std,
    // rmsBaselineVersion)에 나중에 끝나는 쪽이 이겨서 덮어써버리는 문제가
    // 있었다 - 어느 쪽 결과가 최종 기준값이 되는지가 순전히 타이밍에
    // 좌우되어 재현 불가능했다.
    if (isManualRmsRecalibrationBlocked()) {
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("STAGE5_BUSY_IGNORED_C");
        xSemaphoreGive(serialMutex);
      }
    } else {
      beginRmsCalibrationCountdown(millis(), "SERIAL");
    }
  }
  if (heaterPauseToggleRequested) heaterPauseTogglePending = true;

  if (Serial.available() == 0) serialRecalibrationDrainActive = false;
}

// ===================================================================
// Part 3: combine the two gates (checklist item 3)
// ===================================================================

// Latest debounced IMU verdict. Written only by imuTask() (core 0, see
// below), read only by loop() (core 1) — always through imuStateMutex, since
// a bare bool/struct read/write across cores is not guaranteed atomic on
// ESP32 without one.
bool isImuStaticEligible = false;
bool isImuStaticSegmentConfirmed = false;
ImuStaticEvaluation lastImuEvaluation;
SemaphoreHandle_t imuStateMutex = nullptr;
// serialMutex is declared above the IMU helpers because their error paths use
// it. It guards every burst of Serial output from both cores.
TaskHandle_t imuTaskHandle = nullptr;

// A valid moving frame may be a short motion flicker, so the normal motion
// debounce remains in effect. A sensor/configuration error is different: the
// missing interval cannot be trusted as static. Reset the whole segment so a
// recovered sensor must establish a fresh 300 ms + 5 s static interval.
bool updateImuStaticEligibilityFailSafe(
    const ImuStaticEvaluation& evaluation, unsigned long nowMs) {
  if (evaluation.status != IMU_OK) {
    resetStaticSegmentGate(imuStaticGate);
    return false;
  }
  return updateStaticSegmentGate(
      imuStaticGate, evaluation.isStaticNow, nowMs, imuSegmentConfig);
}

void printCombinedState(unsigned long nowMs, bool contracting,
                        bool staticEligible,
                        const ImuStaticEvaluation& imuEval) {
  const bool isMdfEligible =
      isMdfEligibleForFatigueAnalysis(contracting, staticEligible);

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;
  Serial.print("COMBINED t=");
  Serial.print(nowMs);
  Serial.print(" isContracting=");
  Serial.print(contracting ? 1 : 0);
  Serial.print(" imuStatus=");
  Serial.print(static_cast<int>(imuEval.status));
  Serial.print(" imuStaticNow=");
  Serial.print(imuEval.isStaticNow ? 1 : 0);
  Serial.print(" imuStaticEligible=");
  Serial.print(staticEligible ? 1 : 0);
  Serial.print(" isMdfEligible=");
  Serial.println(isMdfEligible ? 1 : 0);
  xSemaphoreGive(serialMutex);
}

// ===================================================================
// Part 4: MDF task + SerialDataFrame output (checklist items 4 and 5)
// ===================================================================
//
// FFT uses arduino_fft_mdf_adapter.h's shared 8 KB work buffers and runs on
// core 0 in its own task, so the synchronous FFT (up to a few ms) can never
// interrupt the 1,000 Hz ADC schedule on core 1 - the same
// reasoning that already keeps the IMU's blocking I2C reads off core 1.
//
// ArduinoFftMdfConnection::processCombined() is used (not the raw
// calculateMDFForEligibleSamples() call the old dataframe file made) so the
// The connection is configured below to calculate once for every plot14-sized
// still block. Moving blocks remain RMS-only.
struct MdfTaskInput {
  float samples[EMG_SAMPLE_COUNT] = {};
  float rms = 0.0f;
  bool isContracting = false;
  bool isStaticEligible = false;
  bool isStaticSegmentConfirmed = false;
  bool samplesValid = false;
  ImuStaticEvaluation imuEval;
  unsigned long timestampMs = 0UL;
  uint32_t gateGeneration = 0U;
  bool acquisitionHealthy = false;
  bool filteringHealthy = false;
  bool indicatorsHealthy = false;
  bool baselineReady = false;
  bool baselineCollecting = false;
  float baselineRms = NAN;
  float baselineStd = NAN;
  uint32_t baselineVersion = 0U;
  bool baselineRecalibrated = false;
  uint32_t manualSessionGeneration = 0U;
};

ArduinoFftMdfConnection mdfConnection;
// core0의 mdfTask에서만 접근한다. 세그먼트 간 표본 혼합을 막기 위해
// gateGeneration이 바뀔 때마다 리셋한다.
FatigueTrendHistory fatigueTrendHistory;
constexpr size_t MDF_INPUT_QUEUE_CAPACITY = 4U;
MdfTaskInput mdfInputQueue[MDF_INPUT_QUEUE_CAPACITY];
size_t mdfInputQueueHead = 0U;
size_t mdfInputQueueTail = 0U;
size_t mdfInputQueueCount = 0U;
volatile uint32_t droppedMdfInputCount = 0U;
// [수정] 이전(v1.2.9) 라운드에서 이 카운터를 portMUX_TYPE 스핀락으로 보호
//하도록 강화했었는데, 이 stage5 통합 파일은 그 이전 버전을 베이스로 만들어져
// 다시 volatile만으로 되돌아가 있었다. core1(loop 쪽, queueMdfInput()과
// commitStage5BaselineAtomically() 양쪽)과 core0(mdfTask의
// reportDroppedMdfInputsIfNeeded())가 이 값을 공유하므로 다시 스핀락으로
// 감싼다.
portMUX_TYPE droppedMdfInputMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t mdfInputMutex = nullptr;
SemaphoreHandle_t mdfDataReady = nullptr;
TaskHandle_t mdfTaskHandle = nullptr;
bool lastQueuedMdfSegmentActive = false;
uint32_t mdfGateGeneration = 0U;

// The MDF task publishes immutable measurement snapshots here. Stage 5 reads
// them on core 1; it never reads a half-updated collection of RMS/IMU fields.
PipelineStage5MeasurementFrame latestStage5Measurement;
PipelineStage5FatigueDecision latestStage5Decision;
PipelineStage5PeripheralSnapshot latestStage5Peripherals;
bool hasStage5Measurement = false;
uint32_t nextStage5MeasurementSequence = 0U;
SemaphoreHandle_t stage5SnapshotMutex = nullptr;

Stage5SystemConfig stage5SystemConfig;
Stage5SystemController stage5SystemController;
bool stage5SystemEnabled = false;

// [신규] handleSerialCommands()와 runStage5Controller()는 둘 다 loop()
// (core1)에서만 순서대로 불리므로, 여기서 stage5SystemController를
// 뮤텍스 없이 그냥 읽어도 안전하다(같은 코어, 같은 스레드, 절대 동시에
// 실행되지 않음).
//
// STAGE5_SYSTEM_RECALIBRATING(샘플 수집 중)뿐 아니라
// WAITING_BASELINE_COMMIT/WAITING_BASELINE_PUBLISH(수집은 끝났지만 아직
// 전역 베이스라인에 커밋·반영 확인 중인 구간)까지 전부 "진행 중"으로
// 본다 - 특히 커밋 직전/직후가 수동 'C'와 부딪히면 가장 위험한 구간이라
// 반드시 포함해야 한다.
bool isManualRmsRecalibrationBlocked() {
  if (!stage5SystemEnabled) return false;

  // Starting manual baseline collection during an intervention, contact
  // recovery, automatic recalibration, commit/publish, or fault handling can
  // collect contaminated RMS samples or race the automatic baseline update.
  return stage5SystemController.state != STAGE5_SYSTEM_DISABLED &&
         stage5SystemController.state != STAGE5_SYSTEM_WAITING_DATA &&
         stage5SystemController.state != STAGE5_SYSTEM_MEASURING &&
         stage5SystemController.state !=
             STAGE5_SYSTEM_WAITING_MANUAL_BASELINE;
}

bool isStage5SystemPinConfigured(int pin) { return pin >= 0; }

struct MdfOutputState {
  uint32_t processedGateGeneration = 0U;
  bool hasCachedMdf = false;
  float cachedMdfHz = 0.0f;
  unsigned long cachedMdfTimestampMs = 0UL;
};

// Converts one immutable task input into the transport frame. Keeping this
// state transition separate from Serial I/O makes the per-block cache and
// fail-safe behavior directly testable on a PC.
SerialDataFrame buildSerialDataFrame(const MdfTaskInput& input,
                                     MdfOutputState& state,
                                     StaticMDFResult* newMdfResult = nullptr) {
  if (input.gateGeneration != state.processedGateGeneration) {
    mdfConnection.reset();
    state.hasCachedMdf = false;
    state.cachedMdfHz = 0.0f;
    state.cachedMdfTimestampMs = 0UL;
    state.processedGateGeneration = input.gateGeneration;
  }

  // Requested display behavior: every still block carries RMS + MDF; a moving
  // block carries RMS only. Fatigue history is separately restricted to
  // contracted still blocks in mdfTask().
  const bool currentWindowEligible =
      input.samplesValid && input.imuEval.status == IMU_OK &&
      input.isStaticEligible && input.isStaticSegmentConfirmed;
  const bool calculationDue = currentWindowEligible &&
      (!state.hasCachedMdf ||
       input.timestampMs - state.cachedMdfTimestampMs >=
           mdfConnection.config().analysisIntervalMs);

  const StaticMDFResult result = mdfConnection.processEligible(
      currentWindowEligible, input.timestampMs, input.samples,
      EMG_SAMPLE_COUNT, static_cast<float>(EMG_SAMPLE_RATE_HZ));

  if (newMdfResult != nullptr) *newMdfResult = result;

  if (result.valid) {
    state.hasCachedMdf = true;
    state.cachedMdfHz = result.mdfHz;
    state.cachedMdfTimestampMs = input.timestampMs;
  }

  const bool hasUsableMdf = currentWindowEligible && state.hasCachedMdf &&
                            (!calculationDue || result.valid);
  SerialDataFrame frame;
  frame.timestampMs = input.timestampMs;
  frame.rms = input.rms;
  frame.isContracting = input.isContracting;
  frame.isImuStaticNow = input.imuEval.isStaticNow;
  frame.isMdfEligible = hasUsableMdf;
  frame.mdfHz = hasUsableMdf ? state.cachedMdfHz : 0.0f;
  frame.imuStatus = input.imuEval.status;
  return frame;
}

uint32_t publishStage5Measurement(const MdfTaskInput& input,
                                  const SerialDataFrame& serialFrame) {
  if (stage5SnapshotMutex == nullptr ||
      xSemaphoreTake(stage5SnapshotMutex, portMAX_DELAY) != pdTRUE) {
    return 0U;
  }
  ++nextStage5MeasurementSequence;
  if (nextStage5MeasurementSequence == 0U) {
    ++nextStage5MeasurementSequence;
  }
  latestStage5Measurement = PipelineStage5MeasurementFrame();
  latestStage5Measurement.serial = serialFrame;
  latestStage5Measurement.sequence = nextStage5MeasurementSequence;
  latestStage5Measurement.acquisitionHealthy = input.acquisitionHealthy;
  latestStage5Measurement.filteringHealthy = input.filteringHealthy;
  latestStage5Measurement.indicatorsHealthy = input.indicatorsHealthy;
  latestStage5Measurement.rmsSampleReady = input.samplesValid;
  latestStage5Measurement.rmsSignalValid = input.samplesValid;
  latestStage5Measurement.baselineRms = input.baselineRms;
  latestStage5Measurement.baselineStd = input.baselineStd;
  latestStage5Measurement.baselineVersion = input.baselineVersion;
  latestStage5Measurement.baselineRecalibrated =
      input.baselineRecalibrated;
  hasStage5Measurement = true;
  const uint32_t publishedSequence = latestStage5Measurement.sequence;
  xSemaphoreGive(stage5SnapshotMutex);
  return publishedSequence;
}

// Call this from a validated on-board fatigue algorithm or a serial-return
// adapter. The sequence and time must be copied from the exact measurement
// frame used for the decision. isMdfEligible alone is never a fatigue result.
bool submitStage5FatigueDecision(uint32_t measurementSequence,
                                 unsigned long measurementTimeMs,
                                 bool fatigued) {
  if (stage5SnapshotMutex == nullptr || measurementSequence == 0U ||
      xSemaphoreTake(stage5SnapshotMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }
  latestStage5Decision.available = true;
  latestStage5Decision.healthy = true;
  latestStage5Decision.fatigued = fatigued;
  latestStage5Decision.measurementSequence = measurementSequence;
  latestStage5Decision.measurementTimeMs = measurementTimeMs;
  xSemaphoreGive(stage5SnapshotMutex);
  return true;
}

// Convenience entry point for a future Serial/Python return adapter. The
// existing DATA line already carries timestampMs. A late result for a frame
// that is no longer the latest snapshot is rejected rather than attached to
// newer measurements.
bool submitStage5FatigueDecisionByTimestamp(
    unsigned long measurementTimeMs, bool fatigued) {
  if (stage5SnapshotMutex == nullptr ||
      xSemaphoreTake(stage5SnapshotMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }
  if (!hasStage5Measurement ||
      latestStage5Measurement.serial.timestampMs != measurementTimeMs) {
    xSemaphoreGive(stage5SnapshotMutex);
    return false;
  }
  latestStage5Decision.available = true;
  latestStage5Decision.healthy = true;
  latestStage5Decision.fatigued = fatigued;
  latestStage5Decision.measurementSequence =
      latestStage5Measurement.sequence;
  latestStage5Decision.measurementTimeMs = measurementTimeMs;
  xSemaphoreGive(stage5SnapshotMutex);
  return true;
}

void setStage5DecisionEngineHealthy(bool healthy) {
  if (stage5SnapshotMutex == nullptr ||
      xSemaphoreTake(stage5SnapshotMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  latestStage5Decision.healthy = healthy;
  if (!healthy) latestStage5Decision.available = false;
  xSemaphoreGive(stage5SnapshotMutex);
}

void publishStage5PeripheralSnapshot(
    const PipelineStage5PeripheralSnapshot& peripherals) {
  if (stage5SnapshotMutex == nullptr ||
      xSemaphoreTake(stage5SnapshotMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  latestStage5Peripherals = peripherals;
  xSemaphoreGive(stage5SnapshotMutex);
}

// Prints one SerialDataFrame line atomically relative to other tasks.
void printDataFrame(const SerialDataFrame& frame) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;
  Serial.print("DATA,");
  Serial.print(frame.timestampMs);
  Serial.print(',');
  Serial.print(frame.rms, 6);
  Serial.print(',');
  Serial.print(frame.isContracting ? 1 : 0);
  Serial.print(',');
  Serial.print(frame.isImuStaticNow ? 1 : 0);
  Serial.print(',');
  Serial.print(frame.isMdfEligible ? 1 : 0);
  Serial.print(',');
  Serial.print(frame.mdfHz, 3);
  Serial.print(',');
  Serial.println(static_cast<int>(frame.imuStatus));
  xSemaphoreGive(serialMutex);
}

// Runs on core 1 (loop()). Only a cheap memcpy of the already-completed
// window happens here - the actual FFT never runs on this core.
// emgSamples[] still holds this window's notch-filtered, DC-removed samples at
// this point (resetEmgWindow() only rewinds the write index, it does not
// clear the array), so it is safe to copy from here.
//
// gateGeneration changes only when the confirmed MDF segment starts or ends.
// A raw moving frame blocks that frame immediately through staticEligible,
// but does not end the segment until StaticSegmentGate::isStatic becomes false.
// This distinction prevents a sub-300 ms motion flicker from resetting and
// mixing contracted fatigue-history segments.
bool queueMdfInput(unsigned long nowMs, bool contracting,
                   bool staticEligible, bool staticSegmentConfirmed,
                   const ImuStaticEvaluation& imuEval,
                   bool samplesValid) {
  const bool segmentActive = contracting && staticSegmentConfirmed &&
                             imuEval.status == IMU_OK;
  if (segmentActive != lastQueuedMdfSegmentActive) {
    ++mdfGateGeneration;
    lastQueuedMdfSegmentActive = segmentActive;
  }

  if (xSemaphoreTake(mdfInputMutex, 0) != pdTRUE) {
    portENTER_CRITICAL(&droppedMdfInputMux);
    ++droppedMdfInputCount;
    portEXIT_CRITICAL(&droppedMdfInputMux);
    return false;
  }
  if (mdfInputQueueCount >= MDF_INPUT_QUEUE_CAPACITY) {
    xSemaphoreGive(mdfInputMutex);
    portENTER_CRITICAL(&droppedMdfInputMux);
    ++droppedMdfInputCount;
    portEXIT_CRITICAL(&droppedMdfInputMux);
    return false;
  }

  MdfTaskInput& queued = mdfInputQueue[mdfInputQueueTail];
  queued.timestampMs = nowMs;
  queued.rms = currentRms;
  queued.isContracting = contracting;
  queued.isStaticEligible = staticEligible;
  queued.isStaticSegmentConfirmed = staticSegmentConfirmed;
  queued.samplesValid = samplesValid;
  queued.imuEval = imuEval;
  queued.gateGeneration = mdfGateGeneration;
  queued.acquisitionHealthy = samplesValid && rmsStatus == RMS_OK;
  queued.filteringHealthy = samplesValid;
  queued.indicatorsHealthy = samplesValid && baselineReady &&
                             rmsStatus == RMS_OK && !plot14ContactBad;
  queued.baselineReady = baselineReady;
  queued.baselineCollecting = !baselineReady && !baselineFailed &&
                              !rmsCalibrationCountdownActive;
  queued.baselineRms = baselineReady ? baselineRmsMean : NAN;
  queued.baselineStd = baselineReady ? baselineRmsStd : NAN;
  queued.baselineVersion = baselineReady ? rmsBaselineVersion : 0U;
  queued.baselineRecalibrated =
      baselineReady && rmsBaselineRecalibratedPulse;
  queued.manualSessionGeneration = manualFatigueSessionGeneration;
  const bool queuedBaselineRecalibrated = queued.baselineRecalibrated;
  if (samplesValid) {
    memcpy(queued.samples, emgSamples, sizeof(emgSamples));
  } else {
    memset(queued.samples, 0, sizeof(queued.samples));
  }
  mdfInputQueueTail = (mdfInputQueueTail + 1U) % MDF_INPUT_QUEUE_CAPACITY;
  ++mdfInputQueueCount;
  xSemaphoreGive(mdfInputMutex);
  if (queuedBaselineRecalibrated) {
    rmsBaselineRecalibratedPulse = false;
  }
  (void)xSemaphoreGive(mdfDataReady);  // One wakeup drains every queued window.
  return true;
}

bool tryDequeueMdfInput(MdfTaskInput& input) {
  if (xSemaphoreTake(mdfInputMutex, portMAX_DELAY) != pdTRUE) return false;
  if (mdfInputQueueCount == 0U) {
    xSemaphoreGive(mdfInputMutex);
    return false;
  }
  input = mdfInputQueue[mdfInputQueueHead];
  mdfInputQueueHead = (mdfInputQueueHead + 1U) % MDF_INPUT_QUEUE_CAPACITY;
  --mdfInputQueueCount;
  xSemaphoreGive(mdfInputMutex);
  return true;
}

void reportDroppedMdfInputsIfNeeded(uint32_t& lastReportedDropCount) {
  portENTER_CRITICAL(&droppedMdfInputMux);
  const uint32_t currentDropCount = droppedMdfInputCount;
  portEXIT_CRITICAL(&droppedMdfInputMux);
  if (currentDropCount == lastReportedDropCount) return;
  lastReportedDropCount = currentDropCount;
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) return;
  Serial.print("MDF_INPUT_DROPPED,total=");
  Serial.print(static_cast<unsigned long>(currentDropCount));
  Serial.println("");
  xSemaphoreGive(serialMutex);
}

void mdfTask(void* pvParameters) {
  (void)pvParameters;
  mdfConnection.begin();
  // One FFT per plot14-sized (~1.024 s) still block. The API requires a
  // non-zero interval; 1 ms therefore means every incoming eligible block.
  mdfConnection.configure(0UL, 0UL, 1UL);
  MdfOutputState outputState;
  uint32_t trendGateGeneration = 0U;
  uint32_t processedManualSessionGeneration = 0U;
  bool hasFatigueTrendSampleTime = false;
  unsigned long lastFatigueTrendSampleMs = 0UL;
  uint32_t sessionSampleCount = 0U;
  unsigned long sessionActiveDurationMs = 0UL;
  float sessionStartRms = 0.0f;
  double mdfBaselineSumHz = 0.0;
  uint16_t mdfBaselineSampleCount = 0U;
  bool hasMovingRmsSampleTime = false;
  unsigned long lastMovingRmsSampleMs = 0UL;
  uint32_t movingRmsSampleCount = 0U;
  unsigned long movingRmsActiveDurationMs = 0UL;
  float movingRmsStart = 0.0f;
  unsigned long fatigueAccumulatedMs = 0UL;
  bool fatigueConfirmed = false;
  constexpr unsigned long FATIGUE_CONFIRMATION_MS = 30000UL;
  constexpr float MOVING_RMS_FATIGUE_RATE_PERCENT_PER_MINUTE = 30.0f;
  uint32_t lastReportedDropCount = 0U;
  for (;;) {
    if (xSemaphoreTake(mdfDataReady, portMAX_DELAY) != pdTRUE) continue;

    MdfTaskInput input;
    while (tryDequeueMdfInput(input)) {
      if (directInterventionState == DIRECT_INTERVENTION_VIBRATING ||
          directInterventionState == DIRECT_INTERVENTION_HEATING) {
        continue;
      }
      if (input.gateGeneration != trendGateGeneration) {
        trendGateGeneration = input.gateGeneration;
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.println("TREND_PAUSE");
          xSemaphoreGive(serialMutex);
        }
      }

      // Only a user/boot-requested calibration starts a new fatigue session.
      // Rest, motion, contraction end and automatic post-treatment baseline
      // updates must not erase accumulated MDF/RMS fatigue.
      if (input.manualSessionGeneration != 0U &&
          input.manualSessionGeneration != processedManualSessionGeneration) {
        processedManualSessionGeneration = input.manualSessionGeneration;
        hasFatigueTrendSampleTime = false;
        lastFatigueTrendSampleMs = 0UL;
        sessionSampleCount = 0U;
        sessionActiveDurationMs = 0UL;
        sessionStartRms = 0.0f;
        mdfBaselineSumHz = 0.0;
        mdfBaselineSampleCount = 0U;
        mdfBaselineReady = false;
        mdfBaselineFailed = false;
        baselineMdfHz = 0.0f;
        hasMovingRmsSampleTime = false;
        lastMovingRmsSampleMs = 0UL;
        movingRmsSampleCount = 0U;
        movingRmsActiveDurationMs = 0UL;
        movingRmsStart = 0.0f;
        fatigueAccumulatedMs = 0UL;
        fatigueConfirmed = false;
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.println("TREND_RESET");
          xSemaphoreGive(serialMutex);
        }
      }
      // isMdfEligible represents a usable gated MDF, not merely whether a new
      // FFT happened on this exact window. If an FFT was due
      // but failed, or the current EMG/IMU window is invalid, fail closed.
      StaticMDFResult newMdfResult;
      const SerialDataFrame frame =
          buildSerialDataFrame(input, outputState, &newMdfResult);
      printDataFrame(frame);
      const uint32_t measurementSequence =
          publishStage5Measurement(input, frame);

      // RMS 기준값을 모으는 같은 10초 구간에서 계산된 정지 MDF를 함께
      // 평균내 기준 MDF로 확정한다. 최소 3개의 유효 MDF가 있어야 한다.
      if (input.baselineCollecting && newMdfResult.valid &&
          isfinite(newMdfResult.mdfHz) && newMdfResult.mdfHz > 0.0f) {
        mdfBaselineSumHz += static_cast<double>(newMdfResult.mdfHz);
        if (mdfBaselineSampleCount < 65535U) ++mdfBaselineSampleCount;
      }
      if (input.baselineReady && !mdfBaselineReady &&
          !mdfBaselineFailed) {
        if (mdfBaselineSampleCount >= 3U) {
          baselineMdfHz = static_cast<float>(
              mdfBaselineSumHz /
              static_cast<double>(mdfBaselineSampleCount));
          mdfBaselineReady = isfinite(baselineMdfHz) &&
                             baselineMdfHz > 0.0f;
          mdfBaselineFailed = !mdfBaselineReady;
        } else {
          mdfBaselineFailed = true;
        }
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
          if (mdfBaselineReady) {
            Serial.print("MDF_BASELINE_COMPLETE,");
            Serial.print(baselineMdfHz, 2);
            Serial.print(",SAMPLES=");
            Serial.println(static_cast<int>(mdfBaselineSampleCount));
          } else {
            Serial.println("MDF_BASELINE_FAILED");
          }
          xSemaphoreGive(serialMutex);
        }
      }

      if (newMdfResult.valid && input.isContracting &&
          input.isStaticEligible && input.isStaticSegmentConfirmed &&
          (!hasFatigueTrendSampleTime ||
           input.timestampMs - lastFatigueTrendSampleMs >= 5000UL)) {
        hasFatigueTrendSampleTime = true;
        lastFatigueTrendSampleMs = input.timestampMs;
        if (sessionSampleCount == 0U) {
          sessionStartRms = input.rms;
        } else {
          sessionActiveDurationMs += 5000UL;
        }
        ++sessionSampleCount;

        float sessionMdfPercent = 0.0f;
        float sessionRmsPercent = 0.0f;
        if (mdfBaselineReady && baselineMdfHz > 0.0f) {
          sessionMdfPercent =
              (newMdfResult.mdfHz - baselineMdfHz) /
              baselineMdfHz * 100.0f;
        }
        if (sessionStartRms > 0.0f) {
          sessionRmsPercent =
              (input.rms - sessionStartRms) / sessionStartRms * 100.0f;
        }
        float mdfRatePerMinute = 0.0f;
        float rmsRatePerMinute = 0.0f;
        if (sessionActiveDurationMs > 0UL) {
          const float minuteScale =
              60000.0f / static_cast<float>(sessionActiveDurationMs);
          mdfRatePerMinute = sessionMdfPercent * minuteScale;
          rmsRatePerMinute = sessionRmsPercent * minuteScale;
        }
        if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.print("TREND_PROGRESS,");
          Serial.print(static_cast<unsigned long>(sessionSampleCount));
          Serial.print(',');
          Serial.print(sessionMdfPercent, 2);
          Serial.print(',');
          Serial.print(sessionRmsPercent, 2);
          Serial.print(',');
          Serial.print(mdfRatePerMinute, 2);
          Serial.print(',');
          Serial.print(rmsRatePerMinute, 2);
          Serial.print(',');
          Serial.println(sessionActiveDurationMs / 1000UL);
          xSemaphoreGive(serialMutex);
        }
        if (mdfBaselineReady && measurementSequence != 0U) {
          const bool currentFatigue = sessionMdfPercent <= -15.0f &&
                                      rmsRatePerMinute >= -20.0f;
          // Each valid fatigue-positive trend block represents five seconds.
          // Positive time accumulates across interruptions and is reset only
          // when the user starts a new baseline/session.
          if (currentFatigue && !fatigueConfirmed) {
            if (fatigueAccumulatedMs <=
                FATIGUE_CONFIRMATION_MS - 5000UL) {
              fatigueAccumulatedMs += 5000UL;
            }
          }
          if (!fatigueConfirmed &&
              fatigueAccumulatedMs >= FATIGUE_CONFIRMATION_MS) {
            fatigueConfirmed = true;
            stage5FatigueInterventionPending = true;
          }
          float fatigueProgressPercent =
              (-sessionMdfPercent / 15.0f) * 100.0f;
          if (fatigueProgressPercent < 0.0f) fatigueProgressPercent = 0.0f;
          if (fatigueProgressPercent > 100.0f) fatigueProgressPercent = 100.0f;
          const bool rmsConditionPassed = sessionRmsPercent >= -20.0f;
          (void)submitStage5FatigueDecision(
              measurementSequence, input.timestampMs, fatigueConfirmed);
          if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print("FATIGUE_TREND,MDF_CHANGE_PERCENT=");
            Serial.print(sessionMdfPercent, 2);
            Serial.print(",RMS_CHANGE_PERCENT=");
            Serial.print(sessionRmsPercent, 2);
            Serial.print(",MDF_RATE_PER_MINUTE=");
            Serial.print(mdfRatePerMinute, 2);
            Serial.print(",RMS_RATE_PER_MINUTE=");
            Serial.print(rmsRatePerMinute, 2);
            Serial.print(",ACTIVE_SECONDS=");
            Serial.print(sessionActiveDurationMs / 1000UL);
            Serial.print(",DETECTED=");
            Serial.print(currentFatigue ? 1 : 0);
            Serial.print(",CONFIRM_SECONDS=");
            Serial.print(fatigueAccumulatedMs / 1000UL);
            Serial.print(",FATIGUED=");
            Serial.println(fatigueConfirmed ? 1 : 0);
            Serial.print("근피로진행,기준도달률=");
            Serial.print(fatigueProgressPercent, 1);
            Serial.print("%,MDF변화율=");
            Serial.print(sessionMdfPercent, 2);
            Serial.print("%,RMS변화율=");
            Serial.print(sessionRmsPercent, 2);
            Serial.print("%,RMS조건=");
            Serial.print(rmsConditionPassed ? "통과" : "실패");
            Serial.print(",최종판정=");
            Serial.println(fatigueConfirmed ? "근피로확정" :
                           (currentFatigue ? "근피로판정" : "판정대기"));
            xSemaphoreGive(serialMutex);
          }
        }
      }

      // plot14와 동일하게 IMU가 동작으로 판정한 구간은 MDF를 제외하고
      // RMS만 본다. 동작 구간의 유효 시간만 합산해 분당 RMS 증가율을
      // 계산하며, 30%/분 이상인 블록은 30초 피로 누적에 포함한다.
      if (input.samplesValid && input.acquisitionHealthy &&
          input.indicatorsHealthy && input.baselineReady &&
          input.imuEval.status == IMU_OK && !input.imuEval.isStaticNow &&
          isfinite(input.rms) && input.rms > 0.0f &&
          (!hasMovingRmsSampleTime ||
           input.timestampMs - lastMovingRmsSampleMs >= 5000UL)) {
        hasMovingRmsSampleTime = true;
        lastMovingRmsSampleMs = input.timestampMs;
        if (movingRmsSampleCount == 0U) {
          movingRmsStart = input.rms;
        } else {
          movingRmsActiveDurationMs += 5000UL;
        }
        ++movingRmsSampleCount;

        float movingRmsPercent = 0.0f;
        float movingRmsRatePerMinute = 0.0f;
        if (movingRmsStart > 0.0f) {
          movingRmsPercent =
              (input.rms - movingRmsStart) / movingRmsStart * 100.0f;
        }
        if (movingRmsActiveDurationMs > 0UL) {
          movingRmsRatePerMinute = movingRmsPercent *
              (60000.0f /
               static_cast<float>(movingRmsActiveDurationMs));
        }

        if (movingRmsSampleCount >= 6U && measurementSequence != 0U) {
          const bool movingRmsFatigue = movingRmsRatePerMinute >=
              MOVING_RMS_FATIGUE_RATE_PERCENT_PER_MINUTE;
          if (movingRmsFatigue && !fatigueConfirmed &&
              fatigueAccumulatedMs <=
                  FATIGUE_CONFIRMATION_MS - 5000UL) {
            fatigueAccumulatedMs += 5000UL;
          }
          if (!fatigueConfirmed &&
              fatigueAccumulatedMs >= FATIGUE_CONFIRMATION_MS) {
            fatigueConfirmed = true;
            stage5FatigueInterventionPending = true;
          }
          (void)submitStage5FatigueDecision(
              measurementSequence, input.timestampMs, fatigueConfirmed);
          if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print("MOVING_RMS_TREND,RMS_CHANGE_PERCENT=");
            Serial.print(movingRmsPercent, 2);
            Serial.print(",RMS_RATE_PER_MINUTE=");
            Serial.print(movingRmsRatePerMinute, 2);
            Serial.print(",THRESHOLD=30.00,DETECTED=");
            Serial.print(movingRmsFatigue ? 1 : 0);
            Serial.print(",CONFIRM_SECONDS=");
            Serial.print(fatigueAccumulatedMs / 1000UL);
            Serial.print(",FATIGUED=");
            Serial.println(fatigueConfirmed ? 1 : 0);
            xSemaphoreGive(serialMutex);
          }
        }
      }
    }
    reportDroppedMdfInputsIfNeeded(lastReportedDropCount);
  }
}

// Runs entirely on core 0. Owns Wire/MPU6050 init, reads, and the
// StaticSegmentGate. The blocking I2C calls in readImuAndEvaluate() (up to
// roughly 1 ms per read) never touch core 1, so they cannot delay the
// EMG ADC schedule's 1 ms timing budget over on the RMS side.
void imuTask(void* pvParameters) {
  (void)pvParameters;

  imuReady = initializeWireAndMpu6050();
  consecutiveReadFailures = 0U;
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(imuReady ? "MPU6050_INIT_OK" : "MPU6050_INIT_ERROR");
    xSemaphoreGive(serialMutex);
  }
  // [v1.2.6 신규] 초기화 직후 착용자가 아직 안 움직였을 때 자이로 바이어스를
  // 잡는다. 여기서 안 하면 매 프레임 원시 자이로 값을 그대로 써서 정지
  // 상태에서도 잔여 바이어스만큼 계속 "움직임"으로 오판할 수 있다.
  if (imuReady && !calibrateGyroBias()) {
    imuReady = false;
    prepareWireForRestart();
  }
  resetStaticSegmentGate(imuStaticGate);

  unsigned long lastReinitMs = millis();
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    const unsigned long now = millis();

    if (!imuReady && now - lastReinitMs >= IMU_REINITIALIZE_INTERVAL_MS) {
      lastReinitMs = now;
      prepareWireForRestart();
      imuReady = initializeWireAndMpu6050();
      consecutiveReadFailures = 0U;
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println(imuReady ? "MPU6050_REINIT_OK" : "MPU6050_REINIT_ERROR");
        xSemaphoreGive(serialMutex);
      }
      // [v1.2.6 신규] 재초기화 후에도 바이어스를 다시 잡는다 — 통신이
      // 끊겼다 복구된 경우 이전 바이어스가 더는 안 맞을 수 있음.
      if (imuReady && !calibrateGyroBias()) {
        imuReady = false;
        prepareWireForRestart();
      }
    }

    const ImuStaticEvaluation evaluation = readImuAndEvaluate();
    const bool eligible = updateImuStaticEligibilityFailSafe(
        evaluation, now);

    if (xSemaphoreTake(imuStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      isImuStaticEligible = eligible;
      isImuStaticSegmentConfirmed =
          evaluation.status == IMU_OK && imuStaticGate.isStatic;
      lastImuEvaluation = evaluation;
      xSemaphoreGive(imuStateMutex);
    }
    // If the mutex is momentarily held by loop() (copying it out below),
    // this cycle's update is simply skipped; the next cycle 100 ms later
    // publishes fresh data, so nothing is lost beyond one tick.

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(IMU_READ_INTERVAL_MS));
  }
}

// ===================================================================
// Part 5: verified pipeline -> stage-5 bridge
// ===================================================================

bool getLatestStage5MeasurementIdentity(uint32_t& sequence,
                                        unsigned long& timestampMs,
                                        bool& isMdfEligible) {
  if (stage5SnapshotMutex == nullptr ||
      xSemaphoreTake(stage5SnapshotMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }
  const bool available = hasStage5Measurement;
  if (available) {
    sequence = latestStage5Measurement.sequence;
    timestampMs = latestStage5Measurement.serial.timestampMs;
    isMdfEligible = latestStage5Measurement.serial.isMdfEligible;
  }
  xSemaphoreGive(stage5SnapshotMutex);
  return available;
}

bool readCoherentStage5SystemSnapshot(
    PipelineStage5MeasurementFrame& measurement,
    PipelineStage5FatigueDecision& decision,
    PipelineStage5PeripheralSnapshot& peripherals) {
  // This function is called only by the core-1 controller. Never wait for the
  // core-0 publisher: a wait could miss a 1,000 Hz ADC deadline. If the mutex
  // is momentarily busy, reuse the last coherent copy. Its timestamp is still
  // checked by the stage-5 controller, so it cannot outlive the configured
  // maximumSnapshotAgeMs limit.
  static PipelineStage5MeasurementFrame cachedMeasurement;
  static PipelineStage5FatigueDecision cachedDecision;
  static PipelineStage5PeripheralSnapshot cachedPeripherals;
  static bool cacheAvailable = false;

  if (stage5SnapshotMutex == nullptr) return false;
  if (xSemaphoreTake(stage5SnapshotMutex, 0) == pdTRUE) {
    cacheAvailable = hasStage5Measurement;
    if (cacheAvailable) {
      cachedMeasurement = latestStage5Measurement;
      cachedDecision = latestStage5Decision;
      cachedPeripherals = latestStage5Peripherals;
    }
    xSemaphoreGive(stage5SnapshotMutex);
  }

  if (!cacheAvailable) return false;
  measurement = cachedMeasurement;
  decision = cachedDecision;
  peripherals = cachedPeripherals;

  // A frame calculated using an older RMS baseline must not be consumed
  // after stage 5 atomically publishes a new baseline.
  return measurement.baselineVersion == rmsBaselineVersion;
}

bool commitStage5BaselineAtomically(float mean, float standardDeviation,
                                    uint32_t& publishedVersion) {
  publishedVersion = 0U;
  if (!isfinite(mean) || mean < 0.0f || !isfinite(standardDeviation) ||
      standardDeviation < 0.0f || mdfInputMutex == nullptr ||
      xSemaphoreTake(mdfInputMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }

  uint32_t nextVersion = rmsBaselineVersion + 1U;
  if (nextVersion == 0U) ++nextVersion;
  baselineRmsMean = mean;
  baselineRmsStd = standardDeviation;
  baselineReady = true;
  baselineFailed = false;
  baselineSampleCount = 0;
  isContracting = false;
  resetRmsContractionGate(rmsGate);
  rmsStatus = RMS_OK;
  rmsBaselineVersion = nextVersion;
  rmsBaselineRecalibratedPulse = true;

  // Discard queued windows from the old baseline. A window already being
  // processed may still publish, but readCoherent... rejects its old version.
  portENTER_CRITICAL(&droppedMdfInputMux);
  droppedMdfInputCount += static_cast<uint32_t>(mdfInputQueueCount);
  portEXIT_CRITICAL(&droppedMdfInputMux);
  mdfInputQueueHead = 0U;
  mdfInputQueueTail = 0U;
  mdfInputQueueCount = 0U;
  ++mdfGateGeneration;
  lastQueuedMdfSegmentActive = false;
  xSemaphoreGive(mdfInputMutex);

  publishedVersion = nextVersion;
  return true;
}

void configureStage5System() {
  // Fatigue intervention: one continuous three-second vibration, then the
  // existing stage-5 state machine advances directly to heating.
  stage5SystemConfig.vibration.pattern.onMs = 3000UL;
  stage5SystemConfig.vibration.pattern.offMs = 0UL;
  stage5SystemConfig.vibration.pattern.pulseCount = 1U;
  stage5SystemConfig.vibration.pattern.maxContinuousOnMs = 3000UL;
  stage5SystemConfig.vibration.pattern.maxTotalMs = 3000UL;
  stage5SystemConfig.vibration.dutyPercent = 100U;
  stage5SystemConfig.vibration.retriggerCooldownMs = 2000UL;

  stage5SystemConfig.contact.quality.maxBaselineJumpRatio = 0.50f;
  stage5SystemConfig.contact.quality.badSignalHoldMs = 3000UL;
  stage5SystemConfig.contact.quality.recoveryStableMs = 2000UL;
  stage5SystemConfig.contact.maximumSampleAgeMs = 500UL;
  stage5SystemConfig.contact.minimumSampleIntervalMs = 100UL;
  stage5SystemConfig.contact.maximumComparableSilenceMs = 2000UL;

  stage5SystemConfig.recalibration.collector.requiredSamples = 20;
  stage5SystemConfig.recalibration.collector.timeoutMs = 15000UL;
  stage5SystemConfig.recalibration.collector.maxAttempts = 3;
  stage5SystemConfig.recalibration.relaxedStableMs = 1000UL;
  stage5SystemConfig.recalibration.maximumRelaxedWaitMs = 10000UL;
  stage5SystemConfig.recalibration.minimumSampleIntervalMs = 200UL;
  stage5SystemConfig.recalibration.maximumSampleAgeMs = 500UL;

  stage5SystemConfig.display.brightness = 3U;
  stage5SystemConfig.display.blinkIntervalMs = 500UL;
  stage5SystemConfig.maximumSnapshotAgeMs = 500UL;

  stage5SystemConfig.heating.maximumSessionMs = 900000UL;
  stage5SystemConfig.heating.maximumContinuousOnMs = 120000UL;

  // [주의] 이 값들을 true로 바꾸면 실제 발열체가 물리적으로 작동한다.
  // 반드시 사람이 지켜보는 상태에서, 온도센서 값이 시리얼 모니터에 정상
  // 출력되는 것부터 확인한 뒤 테스트할 것. 온도퓨즈(하드웨어) 배선을
  // 실제로 연결했는지도 업로드 전에 확인할 것.
  stage5SystemConfig.vibrationHardwareEnabled = true;
  stage5SystemConfig.heatingHardwareEnabled = true;
  stage5SystemConfig.displayHardwareEnabled = true;
  stage5SystemEnabled = true;
}

void applyStage5VibrationFailSafe(bool requestedOn, uint8_t dutyPercent) {
  if (!isStage5SystemPinConfigured(STAGE5_SYSTEM_VIBRATION_PIN)) return;
  const bool directVibration =
      directInterventionState == DIRECT_INTERVENTION_VIBRATING;
  const bool allowed = (stage5SystemEnabled || directVibration) &&
      stage5SystemConfig.vibrationHardwareEnabled && dutyPercent == 100U;
  digitalWrite(STAGE5_SYSTEM_VIBRATION_PIN,
               requestedOn && allowed ? HIGH : LOW);
}

void applyStage5HeaterFailSafe(bool requestedOn, uint8_t dutyPercent) {
  if (!isStage5SystemPinConfigured(STAGE5_SYSTEM_HEATER_PIN)) return;
  const bool directHeating =
      directInterventionState == DIRECT_INTERVENTION_HEATING;
  const bool allowed = (stage5SystemEnabled || directHeating) &&
      stage5SystemConfig.heatingHardwareEnabled && dutyPercent == 100U &&
      !heaterPausedByUser;
  digitalWrite(STAGE5_SYSTEM_HEATER_PIN,
               requestedOn && allowed ? HIGH : LOW);
}

portMUX_TYPE stage5DisplayMux = portMUX_INITIALIZER_UNLOCKED;
Tm1637DisplayOutput pendingStage5Display;
bool stage5DisplayPending = false;
TaskHandle_t stage5PeripheralIoTaskHandle = nullptr;

void applyStage5Display(const Tm1637DisplayOutput& display) {
  if (!display.changed) return;
  portENTER_CRITICAL(&stage5DisplayMux);
  pendingStage5Display = display;
  stage5DisplayPending = true;
  portEXIT_CRITICAL(&stage5DisplayMux);
}

void flushPendingStage5Display() {
#if STAGE5_SYSTEM_TM1637_CLK_PIN >= 0 && \
    STAGE5_SYSTEM_TM1637_DIO_PIN >= 0
  Tm1637DisplayOutput display;
  bool haveDisplay = false;
  portENTER_CRITICAL(&stage5DisplayMux);
  if (stage5DisplayPending) {
    display = pendingStage5Display;
    stage5DisplayPending = false;
    haveDisplay = true;
  }
  portEXIT_CRITICAL(&stage5DisplayMux);
  if (!haveDisplay) return;
  const bool allowed = stage5SystemEnabled &&
      stage5SystemConfig.displayHardwareEnabled && display.enabled;
  if (!allowed) {
    stage5SystemTm1637.clear();
    return;
  }
  stage5SystemTm1637.setBrightness(display.brightness, true);
  stage5SystemTm1637.setSegments(display.segments);
#else
  stage5DisplayPending = false;
#endif
}

constexpr unsigned long DS18B20_CONVERSION_MS = 750UL;
constexpr unsigned long TEMP_SENSOR_REQUEST_INTERVAL_MS = 1000UL;

void updateTemperatureSensor(unsigned long nowMs) {
  static unsigned long lastConversionRequestMs = 0UL;
  static unsigned long conversionRequestedMs = 0UL;
  static bool conversionPending = false;

  if (!conversionPending) {
    if (nowMs - lastConversionRequestMs <
        TEMP_SENSOR_REQUEST_INTERVAL_MS) {
      return;
    }
    temperatureSensor.requestTemperatures();
    conversionPending = true;
    conversionRequestedMs = nowMs;
    lastConversionRequestMs = nowMs;
    return;
  }

  if (nowMs - conversionRequestedMs < DS18B20_CONVERSION_MS) return;

  PipelineStage5PeripheralSnapshot peripherals{};
  peripherals.temperatureSampleAvailable = true;
  peripherals.temperatureC = temperatureSensor.getTempCByIndex(0);
  peripherals.temperatureSampleTimeMs = nowMs;
  peripherals.moistureSensorHealthy = true;
  peripherals.thermalFuseHealthy = true;
  peripherals.heaterDriverHealthy = true;
  peripherals.vibrationDriverHealthy = true;
  publishStage5PeripheralSnapshot(peripherals);
  conversionPending = false;
}

ButtonState btnUpState;
ButtonState btnDownState;
ButtonState btnOkState;
constexpr unsigned long TEMP_BUTTON_DEBOUNCE_MS = 50UL;
constexpr unsigned long TEMP_SETTING_DISPLAY_MS = 2000UL;
constexpr unsigned long TEMP_SETTING_EDIT_TIMEOUT_MS = 5000UL;
float pendingSetTemperatureC = 40.0f;
bool temperatureSettingDirty = false;
unsigned long temperatureSettingDisplayUntilMs = 0UL;
unsigned long temperatureSettingLastButtonMs = 0UL;
bool hasLastDirectHeatingSegments = false;
uint8_t lastDirectHeatingSegments[4] = {0U, 0U, 0U, 0U};

void updateTemperatureButtons(unsigned long nowMs) {
  const bool upPressed = debounceButton(
      btnUpState, digitalRead(BTN_UP_PIN) == LOW, nowMs,
      TEMP_BUTTON_DEBOUNCE_MS);
  const bool downPressed = debounceButton(
      btnDownState, digitalRead(BTN_DOWN_PIN) == LOW, nowMs,
      TEMP_BUTTON_DEBOUNCE_MS);
  const bool okPressed = debounceButton(
      btnOkState, digitalRead(BTN_OK_PIN) == LOW, nowMs,
      TEMP_BUTTON_DEBOUNCE_MS);

  const bool heatingActive =
      stage5SystemController.state == STAGE5_SYSTEM_HEATING ||
      directInterventionState == DIRECT_INTERVENTION_HEATING;

  // 증가/감소 뒤 5초 동안 추가 조작이나 OK 확인이 없으면 잘못 눌린
  // 것으로 보고 임시값을 취소한 뒤 현재 센서 온도 표시로 돌아간다.
  if (heatingActive && temperatureSettingDirty &&
      nowMs - temperatureSettingLastButtonMs >=
          TEMP_SETTING_EDIT_TIMEOUT_MS) {
    pendingSetTemperatureC = stage5SystemConfig.heating.setTemperatureC;
    temperatureSettingDirty = false;
    temperatureSettingDisplayUntilMs = 0UL;
  }

  // During heating, OK confirms the staged temperature. Outside heating it
  // keeps its original role: start a new RMS baseline calibration.
  if (okPressed) {
    if (heatingActive) {
      if (temperatureSettingDirty) {
        stage5SystemConfig.heating.setTemperatureC = pendingSetTemperatureC;
        temperatureSettingDirty = false;
      }
      temperatureSettingDisplayUntilMs = nowMs + TEMP_SETTING_DISPLAY_MS;
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.print("TEMP_SET_CONFIRMED,");
        Serial.println(stage5SystemConfig.heating.setTemperatureC, 1);
        xSemaphoreGive(serialMutex);
      }
    } else if (isManualRmsRecalibrationBlocked()) {
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("STAGE5_BUSY_IGNORED_OK");
        xSemaphoreGive(serialMutex);
      }
    } else {
      beginRmsCalibrationCountdown(nowMs, "BUTTON");
    }
  }

  if (!heatingActive) {
    pendingSetTemperatureC = stage5SystemConfig.heating.setTemperatureC;
    temperatureSettingDirty = false;
    temperatureSettingDisplayUntilMs = 0UL;
    temperatureSettingLastButtonMs = 0UL;
    return;
  }
  if (!upPressed && !downPressed) return;

  float requestedSetTemperatureC = temperatureSettingDirty
      ? pendingSetTemperatureC
      : stage5SystemConfig.heating.setTemperatureC;
  if (upPressed) requestedSetTemperatureC += 1.0f;
  if (downPressed) requestedSetTemperatureC -= 1.0f;

  if (requestedSetTemperatureC <
      stage5SystemConfig.heating.minimumSetTemperatureC) {
    requestedSetTemperatureC =
        stage5SystemConfig.heating.minimumSetTemperatureC;
  } else if (requestedSetTemperatureC >
             stage5SystemConfig.heating.maximumSetTemperatureC) {
    requestedSetTemperatureC =
        stage5SystemConfig.heating.maximumSetTemperatureC;
  }
  pendingSetTemperatureC = requestedSetTemperatureC;
  temperatureSettingDirty = true;
  temperatureSettingDisplayUntilMs = 0UL;
  temperatureSettingLastButtonMs = nowMs;
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print("TEMP_SET_PENDING,");
    Serial.println(pendingSetTemperatureC, 1);
    xSemaphoreGive(serialMutex);
  }
}

void runStage5Controller(unsigned long nowMs) {
  static unsigned long lastUpdateMs = 0UL;
  static unsigned long lastStatusHeartbeatMs = 0UL;
  static Stage5SystemState lastState = STAGE5_SYSTEM_DISABLED;
  if ((nowMs - lastUpdateMs) < 50UL) return;
  lastUpdateMs = nowMs;

  if (heaterPauseTogglePending) {
    heaterPauseTogglePending = false;
    if (stage5SystemController.state == STAGE5_SYSTEM_HEATING ||
        directInterventionState == DIRECT_INTERVENTION_HEATING) {
      heaterPausedByUser = !heaterPausedByUser;
      if (xSemaphoreTake(serialMutex, 0) == pdTRUE) {
        Serial.print("HEATER_PAUSED,");
        Serial.println(heaterPausedByUser ? 1 : 0);
        xSemaphoreGive(serialMutex);
      }
    }
  }

  PipelineStage5MeasurementFrame measurement;
  PipelineStage5FatigueDecision decision;
  PipelineStage5PeripheralSnapshot peripherals;
  const bool available = readCoherentStage5SystemSnapshot(
      measurement, decision, peripherals);
  if (stage5FatigueInterventionPending && measurement.sequence != 0U) {
    decision.available = true;
    decision.healthy = true;
    decision.fatigued = true;
    decision.measurementSequence = measurement.sequence;
    decision.measurementTimeMs = measurement.serial.timestampMs;
  }
  Stage5SystemInput input = buildStage5SystemInputFromPipeline(
      nowMs, stage5SystemEnabled, available, measurement, decision,
      peripherals);
  if (stage5FatigueInterventionPending) {
    input.forceIntervention = true;
    input.fatigueDetected = true;
    input.measurementValid = true;
    input.isMdfEligible = true;
    input.isImuStaticNow = true;
  }
  Stage5SystemOutput output = updateStage5System(
      stage5SystemController, input, stage5SystemConfig);

  // The intervention sequencer is deliberately local to this .ino. Once the
  // fatigue task has latched confirmation, no later measurement/contact frame
  // can turn the request back into a waiting state.
  if (stage5FatigueInterventionPending &&
      directInterventionState == DIRECT_INTERVENTION_IDLE) {
    stage5FatigueInterventionPending = false;
    directInterventionState = DIRECT_INTERVENTION_VIBRATING;
    directInterventionStateStartMs = nowMs;
    directHeaterDemand = false;
  }

  if (directInterventionState == DIRECT_INTERVENTION_VIBRATING &&
      nowMs - directInterventionStateStartMs >= 3000UL) {
    directInterventionState = DIRECT_INTERVENTION_HEATING;
    directInterventionStateStartMs = nowMs;
    directHeaterDemand = true;
  }

  if (directInterventionState == DIRECT_INTERVENTION_HEATING &&
      nowMs - directInterventionStateStartMs >= 900000UL) {
    directInterventionState = DIRECT_INTERVENTION_COMPLETE;
    directInterventionStateStartMs = nowMs;
    directHeaterDemand = false;
    heaterPausedByUser = false;
    stage5SystemController.awaitingManualBaseline = true;
  }

  if (output.fault != STAGE5_SYSTEM_FAULT_NONE &&
      directInterventionState != DIRECT_INTERVENTION_IDLE) {
    directInterventionState = DIRECT_INTERVENTION_COMPLETE;
    directHeaterDemand = false;
  }

  if (directInterventionState != DIRECT_INTERVENTION_HEATING) {
    hasLastDirectHeatingSegments = false;
  }
  if (directInterventionState == DIRECT_INTERVENTION_VIBRATING) {
    output.state = STAGE5_SYSTEM_VIBRATING;
    output.vibrationOn = true;
    output.vibrationDutyPercent = 100U;
    output.heaterOn = false;
    output.heaterDutyPercent = 0U;
  } else if (directInterventionState == DIRECT_INTERVENTION_HEATING) {
    const bool temperatureValid = input.temperatureSampleAvailable &&
        (nowMs - input.temperatureSampleTimeMs) <= 2000UL &&
        isfinite(input.temperatureC) && input.temperatureC > -55.0f &&
        input.temperatureC < 45.0f &&
        fabsf(input.temperatureC - 85.0f) > 1.0f &&
        fabsf(input.temperatureC + 127.0f) > 1.0f &&
        input.thermalFuseHealthy && input.heaterDriverHealthy;
    if (!temperatureValid || heaterPausedByUser) {
      directHeaterDemand = false;
    } else if (input.temperatureC <=
               stage5SystemConfig.heating.setTemperatureC - 0.5f) {
      directHeaterDemand = true;
    } else if (input.temperatureC >=
               stage5SystemConfig.heating.setTemperatureC) {
      directHeaterDemand = false;
    }
    output.state = STAGE5_SYSTEM_HEATING;
    output.vibrationOn = false;
    output.vibrationDutyPercent = 0U;
    output.heaterOn = directHeaterDemand;
    output.heaterDutyPercent = directHeaterDemand ? 100U : 0U;

    // 발열 중에는 평소 실제 측정 온도를 표시한다. 증가/감소로 값을
    // 고르는 동안에는 임시 설정값을, OK를 누른 뒤에는 확정된 설정값을
    // 잠시 표시한다. 실제 발열 제어값은 OK를 눌렀을 때만 변경된다.
    const bool showConfirmedSetTemperature =
        temperatureSettingDisplayUntilMs != 0UL &&
        static_cast<long>(temperatureSettingDisplayUntilMs - nowMs) > 0L;
    const float displayedTemperatureC = temperatureSettingDirty
        ? pendingSetTemperatureC
        : (showConfirmedSetTemperature
               ? stage5SystemConfig.heating.setTemperatureC
               : input.temperatureC);
    const Tm1637LogicalFrame heatingFrame = buildStage5SystemDisplayFrame(
        STAGE5_SYSTEM_HEATING, STAGE5_SYSTEM_FAULT_NONE,
        displayedTemperatureC);
    uint8_t heatingSegments[4] = {0U, 0U, 0U, 0U};
    encodeTm1637Frame(heatingFrame, heatingSegments);
    output.display.enabled = true;
    output.display.configValid = true;
    output.display.changed = !hasLastDirectHeatingSegments ||
        !areTm1637SegmentsEqual(heatingSegments,
                                lastDirectHeatingSegments);
    for (uint8_t i = 0U; i < 4U; ++i) {
      output.display.segments[i] = heatingSegments[i];
      lastDirectHeatingSegments[i] = heatingSegments[i];
    }
    hasLastDirectHeatingSegments = true;
  } else if (directInterventionState == DIRECT_INTERVENTION_COMPLETE) {
    hasLastDirectHeatingSegments = false;
    output.state = STAGE5_SYSTEM_WAITING_MANUAL_BASELINE;
    output.vibrationOn = false;
    output.vibrationDutyPercent = 0U;
    output.heaterOn = false;
    output.heaterDutyPercent = 0U;
  } else {
    hasLastDirectHeatingSegments = false;
  }

  stage5SystemController.state = output.state;
  if (directInterventionState != DIRECT_INTERVENTION_HEATING &&
      directInterventionState != DIRECT_INTERVENTION_COMPLETE) {
    heaterPausedByUser = false;
  }
  applyStage5VibrationFailSafe(output.vibrationOn,
                               output.vibrationDutyPercent);
  applyStage5HeaterFailSafe(output.heaterOn, output.heaterDutyPercent);

  // PC 없이도 기준값/전극 접촉 상태를 확인할 수 있도록 TM1637에 표시한다.
  // 0000: 기준값 측정 중, 0: 측정 성공,
  // 1: 기준값 측정 실패 또는 접촉 불량,
  // 1111: 15분 발열 종료 후 새 기준값 측정 대기.
  // 진동/발열 중에는 기존 단계 및 온도 표시를 우선한다.
  static int8_t lastStandaloneDisplayStatus = -1;
  const bool interventionDisplayActive =
      directInterventionState == DIRECT_INTERVENTION_VIBRATING ||
      directInterventionState == DIRECT_INTERVENTION_HEATING;
  if (interventionDisplayActive) {
    lastStandaloneDisplayStatus = -1;
  } else {
    int8_t standaloneDisplayStatus = -1;
    if (directInterventionState == DIRECT_INTERVENTION_COMPLETE) {
      standaloneDisplayStatus = 3;  // 새 기준값 측정 대기: 1111
    } else if (baselineFailed || mdfBaselineFailed ||
               baselineContactWarningActive || plot14ContactBad) {
      standaloneDisplayStatus = 1;
    } else if (baselineReady && mdfBaselineReady) {
      standaloneDisplayStatus = 0;
    } else {
      standaloneDisplayStatus = 2;  // 기준값 측정 중: 0000
    }

    if (standaloneDisplayStatus >= 0) {
      const bool controllerRequestedRefresh = output.display.changed;
      output.display.enabled = true;
      output.display.configValid = true;
      const Tm1637Glyph leadingGlyph = standaloneDisplayStatus == 2
          ? TM_GLYPH_0
          : (standaloneDisplayStatus == 3
                 ? TM_GLYPH_1 : TM_GLYPH_BLANK);
      output.display.segments[0] = encodeTm1637Glyph(leadingGlyph);
      output.display.segments[1] = encodeTm1637Glyph(leadingGlyph);
      output.display.segments[2] = encodeTm1637Glyph(leadingGlyph);
      output.display.segments[3] = encodeTm1637Glyph(
          standaloneDisplayStatus == 1 || standaloneDisplayStatus == 3
              ? TM_GLYPH_1 : TM_GLYPH_0);
      output.display.changed = controllerRequestedRefresh ||
          standaloneDisplayStatus != lastStandaloneDisplayStatus;
      lastStandaloneDisplayStatus = standaloneDisplayStatus;
    } else {
      lastStandaloneDisplayStatus = -1;
    }
  }
  applyStage5Display(output.display);

  // Short, non-blocking heartbeat for the PC status dashboard. It is emitted
  // only when the complete line fits in the UART buffer so the 1000 Hz EMG
  // sampler on this core is never delayed by display output.
  if ((nowMs - lastStatusHeartbeatMs) >= 1000UL) {
    lastStatusHeartbeatMs = nowMs;
    char statusLine[176];
    const int statusLength = snprintf(
        statusLine, sizeof(statusLine),
        "LIVE,%d,%.6f,%.6f,%.6f,%d,%.2f,%d,%d,%d,%.1f,%.1f,%d\n",
        static_cast<int>(output.state), static_cast<double>(currentRms),
        static_cast<double>(baselineRmsMean),
        static_cast<double>(baselineRmsStd), isContracting ? 1 : 0,
        static_cast<double>(input.temperatureC),
        output.vibrationOn ? 1 : 0,
        output.heaterOn && !heaterPausedByUser ? 1 : 0,
        heaterPausedByUser ? 1 : 0,
        static_cast<double>(stage5SystemConfig.heating.setTemperatureC),
        static_cast<double>(pendingSetTemperatureC),
        temperatureSettingDirty ? 1 : 0);
    if (statusLength > 0 &&
        statusLength < static_cast<int>(sizeof(statusLine)) &&
        xSemaphoreTake(serialMutex, 0) == pdTRUE) {
      if (Serial.availableForWrite() >= statusLength) {
        Serial.write(reinterpret_cast<const uint8_t*>(statusLine),
                     static_cast<size_t>(statusLength));
      }
      xSemaphoreGive(serialMutex);
    }
  }

  if (output.baselineCommitRequested) {
    uint32_t publishedVersion = 0U;
    const bool committed = commitStage5BaselineAtomically(
        output.baselineMean, output.baselineStd, publishedVersion);
    (void)acknowledgeStage5SystemBaselineCommit(
        stage5SystemController, committed, publishedVersion);
    if (!committed) {
      applyStage5VibrationFailSafe(false, 0U);
      applyStage5HeaterFailSafe(false, 0U);
    }
  }

  if (output.state != lastState &&
      xSemaphoreTake(serialMutex, 0) == pdTRUE) {
    Serial.print("STAGE5_SYSTEM_STATE=");
    Serial.print(static_cast<int>(output.state));
    Serial.print(",FAULT=");
    Serial.print(static_cast<int>(output.fault));
    Serial.print(",PIPELINE=");
    Serial.println(static_cast<int>(output.pipelineStatus));
    Serial.print("시스템상태,");
    switch (output.state) {
      case STAGE5_SYSTEM_DISABLED: Serial.println("사용안함"); break;
      case STAGE5_SYSTEM_WAITING_DATA: Serial.println("데이터대기"); break;
      case STAGE5_SYSTEM_MEASURING: Serial.println("근피로측정중"); break;
      case STAGE5_SYSTEM_VIBRATING: Serial.println("진동중"); break;
      case STAGE5_SYSTEM_HEATING: Serial.println("발열중"); break;
      case STAGE5_SYSTEM_RECALIBRATING: Serial.println("자동재측정중"); break;
      case STAGE5_SYSTEM_CONTACT_BLOCKED: Serial.println("접촉확인필요"); break;
      case STAGE5_SYSTEM_WAITING_BASELINE_COMMIT: Serial.println("기준값저장대기"); break;
      case STAGE5_SYSTEM_WAITING_BASELINE_PUBLISH: Serial.println("기준값반영대기"); break;
      case STAGE5_SYSTEM_WAITING_MANUAL_BASELINE: Serial.println("치료완료_수동기준값대기"); break;
      case STAGE5_SYSTEM_FAULT: Serial.println("오류"); break;
      default: Serial.println("알수없음"); break;
    }
    xSemaphoreGive(serialMutex);
    lastState = output.state;
  }
}

void stage5PeripheralIoTask(void* pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTick = xTaskGetTickCount();
  for (;;) {
    updateTemperatureSensor(millis());
    flushPendingStage5Display();
    vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(10));
  }
}

// ===================================================================
// setup() / loop()
// ===================================================================

void setup() {
  Serial.begin(115200);
  configureStage5System();

  // RMS side (runs on core 1, in loop() below).
  pinMode(EMG_ADC_PIN, INPUT);
  analogReadResolution(EMG_ADC_BITS);
  initEmgNotchFilter();
  rmsConfig.startStdMult = 3.0f;
  rmsConfig.endStdMult = 1.5f;
  // Two consecutive approximately one-second blocks: about two seconds.
  rmsConfig.debounceBlocks = 2;
  beginRmsCalibrationCountdown(millis(), "BOOT");

  // [v1.2.6 신규] IMU 정지판정 임계값 — 기존엔 이 값들을 아예 안 채워서
  // 헤더 기본값(accel 0.05g / gyro 5dps)이 그대로 쓰였음. 이전 세션에
  // 실측된 값(가만히 있어도 자이로 노이즈가 바이어스 보정 후에도 ~48dps
  // 까지 나왔던 것)에 여유를 두고 채운다 — 이 보드/센서 개체에서도 다시
  // 실측해 조정이 필요할 수 있는 잠정값이다.
  imuConfig.accelStaticDeviationG = 0.15f;
  imuConfig.gyroStaticThresholdDps = 60.0f;
  imuSegmentConfig.motionDebounceMs = 300UL;
  imuSegmentConfig.minStaticDurationMs = 5000UL;

  // IMU side (runs entirely on core 0, in imuTask() above) so its blocking
  // I2C reads can never delay the EMG ADC schedule on core 1.
  imuStateMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  mdfInputMutex = xSemaphoreCreateMutex();
  mdfDataReady = xSemaphoreCreateBinary();
  stage5SnapshotMutex = xSemaphoreCreateMutex();
  if (imuStateMutex == nullptr || serialMutex == nullptr ||
      mdfInputMutex == nullptr || mdfDataReady == nullptr ||
      stage5SnapshotMutex == nullptr) {
    // Extremely unlikely (would need near-total heap exhaustion), but
    // xSemaphoreTake(nullptr, ...) below would be undefined behavior, so
    // fail loudly instead of continuing with a broken mutex/semaphore.
    Serial.println("MUTEX_CREATE_FAILED");
    while (true) delay(1000);
  }

  if (isStage5SystemPinConfigured(STAGE5_SYSTEM_VIBRATION_PIN)) {
    digitalWrite(STAGE5_SYSTEM_VIBRATION_PIN, LOW);
    pinMode(STAGE5_SYSTEM_VIBRATION_PIN, OUTPUT);
    digitalWrite(STAGE5_SYSTEM_VIBRATION_PIN, LOW);
  }
  if (isStage5SystemPinConfigured(STAGE5_SYSTEM_HEATER_PIN)) {
    digitalWrite(STAGE5_SYSTEM_HEATER_PIN, LOW);
    pinMode(STAGE5_SYSTEM_HEATER_PIN, OUTPUT);
    digitalWrite(STAGE5_SYSTEM_HEATER_PIN, LOW);
  }

  temperatureSensor.begin();
  temperatureSensor.setWaitForConversion(false);

  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);

#if STAGE5_SYSTEM_TM1637_CLK_PIN >= 0 && \
    STAGE5_SYSTEM_TM1637_DIO_PIN >= 0
  stage5SystemTm1637.setBrightness(stage5SystemConfig.display.brightness,
                                   true);
  stage5SystemTm1637.clear();
#endif

  // Publish a safe state before the task starts. If task allocation fails,
  // RMS acquisition may continue but MDF eligibility remains false and the
  // combined status reports IMU_COMM_ERROR instead of stale/default data.
  isImuStaticEligible = false;
  isImuStaticSegmentConfirmed = false;
  lastImuEvaluation = communicationErrorEvaluation();
  // [v1.2.9 수정] imuTask 우선순위를 mdfTask(우선순위 1)보다 높여 core0
  // 안에서 우선순위 역전을 방지한다. FFT 연산이 도는
  // 중에 IMU의 100ms 주기 읽기와 겹치면, 동일 우선순위였다면 라운드로빈
  // 때문에 한 틱 정도 밀릴 수 있었다 - IMU가 항상 FFT를 선점하게 한다.
  const BaseType_t taskCreateResult = xTaskCreatePinnedToCore(
      imuTask, "ImuTask", 4096, nullptr, 2, &imuTaskHandle, 0);
  if (taskCreateResult != pdPASS) {
    imuTaskHandle = nullptr;
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("IMU_TASK_CREATE_FAILED");
      xSemaphoreGive(serialMutex);
    }
  }

  // MdfTaskInput holds a full EMG_SAMPLE_COUNT-float array, so mdfTask gets
  // a larger stack than imuTask's.
  const BaseType_t mdfTaskCreateResult = xTaskCreatePinnedToCore(
      mdfTask, "MdfTask", 8192, nullptr, 1, &mdfTaskHandle, 0);
  if (mdfTaskCreateResult != pdPASS) {
    mdfTaskHandle = nullptr;
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("MDF_TASK_CREATE_FAILED");
      xSemaphoreGive(serialMutex);
    }
  }

  // OneWire and TM1637 transfers can exceed one 1,000 Hz EMG sample period.
  // Keep those physical bus transactions on core 0; controller decisions and
  // baseline commits remain on the EMG-owning core 1.
  const BaseType_t peripheralIoTaskCreateResult = xTaskCreatePinnedToCore(
      stage5PeripheralIoTask, "Stage5PeripheralIo", 3072, nullptr, 1,
      &stage5PeripheralIoTaskHandle, 0);
  if (peripheralIoTaskCreateResult != pdPASS) {
    stage5PeripheralIoTaskHandle = nullptr;
    stage5SystemEnabled = false;
    applyStage5VibrationFailSafe(false, 0U);
    applyStage5HeaterFailSafe(false, 0U);
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("STAGE5_PERIPHERAL_IO_TASK_CREATE_FAILED");
      xSemaphoreGive(serialMutex);
    }
  }
}

void loop() {
  // Core 1: RMS/EMG only. No blocking I2C calls and no FFT computation
  // happen on this core, so the 1,000 Hz ADC schedule's 1 ms timing
  // budget is never at risk from the IMU or MDF side.
  //
  // [수정] printRmsState()/printCombinedState()를 여기서 다시 부르지 않는다.
  // 둘 다 serialMutex를 portMAX_DELAY(무한 대기)로 잡는데, core0의 mdfTask
  // (printDataFrame)도 DATA 라인을 찍을 때 같은 serialMutex를 잡는다. 만약
  // core1이 이 두 함수를 부르는 바로 그 순간 mdfTask가 마침 프린트 중이면,
  // core1은 mdfTask가 끝날 때까지(Serial.print 자체가 수 ms 걸릴 수 있음)
  // 블로킹돼서 ~488us 타이밍 예산을 크게 초과할 수 있다. 이건 이전 버전
  // (v1.2.9)에서 이미 한 번 발견해서 이 두 호출을 loop()에서 제거하는 걸로
  // 고쳤던 문제인데, 이 stage5 통합 파일이 그 이전(고치기 전) 버전을
  // 베이스로 만들어져서 다시 들어와 있었다. DATA 라인(core0, mdfTask)만으로
  // 진단 정보는 충분하므로 여기서는 다시 제거한다.
  // Sampling must always be the first operation in loop(). OneWire, display,
  // button and controller housekeeping may occasionally take longer than an
  // ordinary loop pass; doing them first can make the scheduled 1,000 Hz ADC
  // read late even though none of those functions deliberately calls delay().
  const bool interventionActive =
      directInterventionState == DIRECT_INTERVENTION_VIBRATING ||
      directInterventionState == DIRECT_INTERVENTION_HEATING;
  EmgSamplingEvent emgEvent = EMG_NO_EVENT;
  if (!interventionActive) {
    emgEvent = updateEmgSampling();
  } else {
    // Treatment output would contaminate EMG/MDF. Stop acquisition and throw
    // away any partial window; sampling restarts cleanly after treatment.
    if (sampleScheduleStarted || emgSampleIndex != 0) resetEmgWindow();
    currentRms = 0.0f;
    isContracting = false;
  }
  if (!interventionActive &&
      (emgEvent == EMG_WINDOW_READY ||
       emgEvent == EMG_TIMING_FAULT ||
       emgEvent == EMG_ADC_FAULT ||
       emgEvent == EMG_SIGNAL_QUALITY_FAULT)) {
    // Only copy out the combined gate state right after an RMS window
    // completes (about every 1.024 s) - frequent enough to track
    // eligibility changes, and it keeps the mutex hold time on this core
    // rare rather than happening on every single loop() iteration.
    bool staticEligible = false;
    bool staticSegmentConfirmed = false;
    ImuStaticEvaluation evalCopy;
    bool haveImuState = false;
    if (xSemaphoreTake(imuStateMutex, 0) == pdTRUE) {
      staticEligible = isImuStaticEligible;
      staticSegmentConfirmed = isImuStaticSegmentConfirmed;
      evalCopy = lastImuEvaluation;
      haveImuState = true;
      xSemaphoreGive(imuStateMutex);
    }
    const unsigned long nowMs = millis();
    if (!haveImuState) {
      evalCopy = communicationErrorEvaluation();
      staticEligible = false;
      staticSegmentConfirmed = false;
    }

    if (mdfTaskHandle != nullptr) {
      (void)queueMdfInput(nowMs, isContracting, staticEligible,
                         staticSegmentConfirmed, evalCopy,
                         emgEvent == EMG_WINDOW_READY);
    }
  }

  const unsigned long nowMs = millis();
  updateRmsCalibrationCountdown(nowMs);
  updateTemperatureButtons(nowMs);
  runStage5Controller(nowMs);
  // Consume the newest stage-5 frame before accepting a manual calibration.
  // This closes the boundary where an intervention started later in the same
  // loop after 'C' had already reset the RMS baseline.
  handleSerialCommands();
}
