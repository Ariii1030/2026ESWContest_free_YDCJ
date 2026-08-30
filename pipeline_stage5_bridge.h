#ifndef PIPELINE_STAGE5_BRIDGE_H
#define PIPELINE_STAGE5_BRIDGE_H

#include <math.h>
#include <stdint.h>

#include "fatigue_logic.h"
#include "stage5_system_integration_logic.h"

// Pure bridge between checklist items 1-6 and the stage-5 controller.
// GPIO, Serial, mutexes, sensor libraries and millis() remain in the .ino.

struct PipelineStage5MeasurementFrame {
  SerialDataFrame serial;
  uint32_t sequence = 0U;
  bool acquisitionHealthy = false;
  bool filteringHealthy = false;
  bool indicatorsHealthy = false;
  bool rmsSampleReady = false;
  bool rmsSignalValid = false;
  float baselineRms = NAN;
  float baselineStd = NAN;
  uint32_t baselineVersion = 0U;
  bool baselineRecalibrated = false;
};

// A decision must name the exact measurement frame it analyzed.  This keeps
// a delayed Python/on-board result from being attached to newer RMS/MDF data.
struct PipelineStage5FatigueDecision {
  bool available = false;
  // True means the decision subsystem/connection itself is operational.
  // Non-MDF frames do not need a result, but still need a healthy engine.
  bool healthy = false;
  bool fatigued = false;
  uint32_t measurementSequence = 0U;
  unsigned long measurementTimeMs = 0UL;
};

struct PipelineStage5PeripheralSnapshot {
  bool moistureSensorHealthy = false;
  bool moistureDetected = false;
  bool temperatureSampleAvailable = false;
  unsigned long temperatureSampleTimeMs = 0UL;
  float temperatureC = NAN;
  bool thermalFuseHealthy = false;
  bool heaterDriverHealthy = false;
  bool vibrationDriverHealthy = false;
  bool emergencyStop = false;
  bool userResetRequested = false;
};

inline bool isPipelineStage5MeasurementFrameValid(
    const PipelineStage5MeasurementFrame& frame) {
  return frame.sequence != 0U &&
         frame.serial.imuStatus == IMU_OK &&
         frame.acquisitionHealthy && frame.filteringHealthy &&
         frame.indicatorsHealthy && frame.rmsSampleReady &&
         frame.rmsSignalValid && isfinite(frame.serial.rms) &&
         frame.serial.rms >= 0.0f && isfinite(frame.baselineRms) &&
         frame.baselineRms >= 0.0f && isfinite(frame.baselineStd) &&
         frame.baselineStd > 0.0f && frame.baselineVersion != 0U;
}

inline bool doesPipelineStage5DecisionMatch(
    const PipelineStage5MeasurementFrame& frame,
    const PipelineStage5FatigueDecision& decision) {
  return decision.available && decision.healthy &&
         decision.measurementSequence == frame.sequence &&
         decision.measurementTimeMs == frame.serial.timestampMs;
}

inline Stage5SystemInput buildStage5SystemInputFromPipeline(
    unsigned long nowMs,
    bool stage5Enabled,
    bool snapshotAvailable,
    const PipelineStage5MeasurementFrame& frame,
    const PipelineStage5FatigueDecision& decision,
    const PipelineStage5PeripheralSnapshot& peripherals) {
  Stage5SystemInput input;
  input.nowMs = nowMs;
  input.enabled = stage5Enabled;
  input.snapshotAvailable = snapshotAvailable;
  if (!snapshotAvailable) {
    input.emergencyStop = peripherals.emergencyStop;
    input.userResetRequested = peripherals.userResetRequested;
    return input;
  }

  const bool measurementFrameValid =
      isPipelineStage5MeasurementFrameValid(frame);
  const bool decisionMatches =
      doesPipelineStage5DecisionMatch(frame, decision);

  input.snapshotTimeMs = frame.serial.timestampMs;
  input.snapshotSequence = frame.sequence;
  input.acquisitionHealthy = frame.acquisitionHealthy;
  input.filteringHealthy = frame.filteringHealthy;
  input.indicatorsHealthy = frame.indicatorsHealthy;
  input.motionHealthy = frame.serial.imuStatus == IMU_OK;
  input.decisionHealthy = decision.healthy &&
      (!frame.serial.isMdfEligible || decisionMatches);

  // MDF eligibility means "the gated MDF is usable"; it is not itself a
  // fatigue-positive decision.  Both the eligible frame and its matching
  // decision are required before stage 5 may accept an intervention.
  input.measurementValid = measurementFrameValid &&
                           frame.serial.isMdfEligible && decisionMatches;
  input.fatigueDetected = input.measurementValid && decision.fatigued;
  input.isMdfEligible = frame.serial.isMdfEligible;
  input.isImuStaticNow = frame.serial.isImuStaticNow;
  input.isContracting = frame.serial.isContracting;

  input.relaxedRmsExpected = !frame.serial.isContracting;
  input.rmsSampleReady = frame.rmsSampleReady;
  input.rmsSampleTimeMs = frame.serial.timestampMs;
  input.rmsSignalValid = frame.rmsSignalValid;
  input.currentRms = frame.serial.rms;
  input.baselineRms = frame.baselineRms;
  input.baselineVersion = frame.baselineVersion;
  input.baselineRecalibrated = frame.baselineRecalibrated;

  input.moistureSensorHealthy = peripherals.moistureSensorHealthy;
  input.moistureDetected = peripherals.moistureDetected;
  input.temperatureSampleAvailable =
      peripherals.temperatureSampleAvailable;
  input.temperatureSampleTimeMs =
      peripherals.temperatureSampleTimeMs;
  input.temperatureC = peripherals.temperatureC;
  input.thermalFuseHealthy = peripherals.thermalFuseHealthy;
  input.heaterDriverHealthy = peripherals.heaterDriverHealthy;
  input.vibrationDriverHealthy = peripherals.vibrationDriverHealthy;
  input.emergencyStop = peripherals.emergencyStop;
  input.userResetRequested = peripherals.userResetRequested;
  return input;
}

#endif  // PIPELINE_STAGE5_BRIDGE_H
