#ifndef RECALIBRATION_LOGIC_H
#define RECALIBRATION_LOGIC_H

#include <math.h>
#include <stdint.h>

#include "fatigue_logic.h"

// Low-level collector kept API-compatible with the original stage-5 header.

struct RecalibrationConfig {
  int requiredSamples = 20;
  unsigned long timeoutMs = 15000UL;
  int maxAttempts = 3;
  BaselineQualityConfig quality;
};

inline bool isRecalibrationConfigValid(const RecalibrationConfig& config) {
  return config.requiredSamples >= 2 &&
         config.requiredSamples <= CALIB_MAX_SAMPLES &&
         config.timeoutMs > 0UL &&
         config.maxAttempts > 0 && config.maxAttempts <= 10 &&
         isfinite(config.quality.maxCoefficientOfVariation) &&
         config.quality.maxCoefficientOfVariation >= 0.0f &&
         config.quality.maxOutlierCount >= 0 &&
         isfinite(config.quality.minMadRatio) &&
         config.quality.minMadRatio >= 0.0f;
}

enum RecalibrationStatus {
  RECALIBRATION_IDLE,
  RECALIBRATION_COLLECTING,
  RECALIBRATION_COMPLETE,
  RECALIBRATION_QUALITY_FAILED,
  RECALIBRATION_TIMEOUT,
  RECALIBRATION_CONFIG_ERROR
};

struct RecalibrationCollector {
  float samples[CALIB_MAX_SAMPLES] = {};
  int count = 0;
  bool active = false;
  unsigned long startMs = 0UL;
};

struct RecalibrationResult {
  RecalibrationStatus status = RECALIBRATION_IDLE;
  int collectedSamples = 0;
  bool baselineUpdated = false;
  float baselineMean = 0.0f;
  float baselineStd = 0.0f;
};

inline bool startRecalibration(RecalibrationCollector& collector,
                               unsigned long nowMs,
                               const RecalibrationConfig& config) {
  collector = RecalibrationCollector();
  if (!isRecalibrationConfigValid(config)) return false;
  collector.active = true;
  collector.startMs = nowMs;
  return true;
}

inline RecalibrationResult updateRecalibration(
    RecalibrationCollector& collector,
    bool sampleReady,
    bool sampleAllowed,
    float rms,
    unsigned long nowMs,
    const RecalibrationConfig& config) {
  RecalibrationResult result;
  result.collectedSamples = collector.count;
  if (!isRecalibrationConfigValid(config)) {
    collector.active = false;
    result.status = RECALIBRATION_CONFIG_ERROR;
    return result;
  }
  if (!collector.active) return result;
  if ((nowMs - collector.startMs) >= config.timeoutMs) {
    collector.active = false;
    result.status = RECALIBRATION_TIMEOUT;
    return result;
  }
  if (!sampleReady || !sampleAllowed) {
    result.status = RECALIBRATION_COLLECTING;
    return result;
  }
  if (!isfinite(rms) || rms < 0.0f) {
    result.status = RECALIBRATION_COLLECTING;
    return result;
  }

  collector.samples[collector.count++] = rms;
  result.collectedSamples = collector.count;
  if (collector.count < config.requiredSamples) {
    result.status = RECALIBRATION_COLLECTING;
    return result;
  }

  collector.active = false;
  const BaselineQualityResult quality = checkCalibQuality(
      collector.samples, collector.count, config.quality);
  if (!quality.ok) {
    result.status = RECALIBRATION_QUALITY_FAILED;
    return result;
  }
  result.status = RECALIBRATION_COMPLETE;
  result.baselineUpdated = true;
  result.baselineMean = quality.mean;
  result.baselineStd = quality.std;
  return result;
}

// High-level post-intervention wrapper. It adds relaxed/static/contact gates,
// sample freshness and spacing, bounded retry, baseline-shift protection and
// an explicit atomic-commit output boundary.

struct PostInterventionRecalibrationConfig {
  RecalibrationConfig collector;
  unsigned long relaxedStableMs = 1000UL;
  unsigned long maximumRelaxedWaitMs = 10000UL;
  unsigned long minimumSampleIntervalMs = 200UL;
  unsigned long maximumSampleAgeMs = 500UL;
  unsigned long retryDelayMs = 1000UL;
  float maximumBaselineShiftRatio = 0.50f;
  float minimumExistingBaselineRms = 1e-6f;
  float minimumAcceptedRms = 1e-6f;
  float maximumAcceptedRms = 1000000.0f;
};

inline bool isPostInterventionRecalibrationConfigValid(
    const PostInterventionRecalibrationConfig& config) {
  if (!isRecalibrationConfigValid(config.collector) ||
      config.relaxedStableMs == 0UL ||
      config.maximumRelaxedWaitMs <= config.relaxedStableMs ||
      config.minimumSampleIntervalMs == 0UL ||
      config.maximumSampleAgeMs == 0UL ||
      !isfinite(config.maximumBaselineShiftRatio) ||
      config.maximumBaselineShiftRatio < 0.0f ||
      !isfinite(config.minimumExistingBaselineRms) ||
      config.minimumExistingBaselineRms <= 0.0f ||
      !isfinite(config.minimumAcceptedRms) ||
      !isfinite(config.maximumAcceptedRms) ||
      config.minimumAcceptedRms <= 0.0f ||
      config.maximumAcceptedRms <= config.minimumAcceptedRms) {
    return false;
  }
  const uint64_t minimumCollectionSpan =
      static_cast<uint64_t>(config.collector.requiredSamples - 1) *
      config.minimumSampleIntervalMs;
  return minimumCollectionSpan <
         static_cast<uint64_t>(config.collector.timeoutMs);
}

enum PostInterventionRecalibrationState {
  POST_RECAL_DISABLED,
  POST_RECAL_IDLE,
  POST_RECAL_WAITING_RELAXED,
  POST_RECAL_COLLECTING,
  POST_RECAL_RETRY_DELAY,
  POST_RECAL_COMPLETE,
  POST_RECAL_CANCELLED,
  POST_RECAL_FAILED,
  POST_RECAL_CONFIG_ERROR
};

enum PostInterventionRecalibrationRejectReason {
  POST_RECAL_REJECT_NONE,
  POST_RECAL_REJECT_BASELINE_INVALID,
  POST_RECAL_REJECT_NOT_STATIC,
  POST_RECAL_REJECT_CONTRACTING,
  POST_RECAL_REJECT_CONTACT,
  POST_RECAL_REJECT_MOISTURE,
  POST_RECAL_REJECT_SIGNAL_INVALID,
  POST_RECAL_REJECT_NO_SAMPLE,
  POST_RECAL_REJECT_STALE_SAMPLE,
  POST_RECAL_REJECT_RMS_INVALID,
  POST_RECAL_REJECT_SAMPLE_INTERVAL,
  POST_RECAL_REJECT_QUALITY,
  POST_RECAL_REJECT_BASELINE_SHIFT,
  POST_RECAL_REJECT_TIMEOUT,
  POST_RECAL_REJECT_CONFIG
};

struct PostInterventionRecalibrationInput {
  unsigned long nowMs = 0UL;
  bool enabled = false;
  bool startRequested = false;
  bool cancelRequested = false;
  float existingBaselineRms = NAN;
  bool isImuStaticNow = false;
  bool isContracting = false;
  bool contactHealthy = false;
  bool moistureDetected = false;
  bool signalValid = false;
  bool sampleReady = false;
  unsigned long sampleTimeMs = 0UL;
  float rms = NAN;
};

struct PostInterventionRecalibrationController {
  RecalibrationCollector collector;
  bool active = false;
  bool triggerArmed = true;
  bool relaxedTracking = false;
  bool retryPending = false;
  bool hasAcceptedSample = false;
  int attempt = 0;
  unsigned long relaxedStartMs = 0UL;
  unsigned long attemptStartMs = 0UL;
  unsigned long retryStartMs = 0UL;
  unsigned long lastAcceptedSampleMs = 0UL;
  float originalBaselineRms = 0.0f;
  PostInterventionRecalibrationState terminalState = POST_RECAL_IDLE;
};

struct PostInterventionRecalibrationOutput {
  PostInterventionRecalibrationState state = POST_RECAL_DISABLED;
  PostInterventionRecalibrationRejectReason rejectReason =
      POST_RECAL_REJECT_NONE;
  int attempt = 0;
  int collectedSamples = 0;
  bool sampleAccepted = false;
  bool baselineUpdated = false;
  float baselineMean = 0.0f;
  float baselineStd = 0.0f;
};

inline void resetPostInterventionRecalibration(
    PostInterventionRecalibrationController& controller,
    bool requestCurrentlyActive = false) {
  controller = PostInterventionRecalibrationController();
  controller.triggerArmed = !requestCurrentlyActive;
}

inline PostInterventionRecalibrationRejectReason getPostRecalibrationGate(
    const PostInterventionRecalibrationInput& input) {
  if (input.moistureDetected) return POST_RECAL_REJECT_MOISTURE;
  if (!input.contactHealthy) return POST_RECAL_REJECT_CONTACT;
  if (!input.signalValid) return POST_RECAL_REJECT_SIGNAL_INVALID;
  if (!input.isImuStaticNow) return POST_RECAL_REJECT_NOT_STATIC;
  if (input.isContracting) return POST_RECAL_REJECT_CONTRACTING;
  return POST_RECAL_REJECT_NONE;
}

inline void beginPostRecalibrationAttempt(
    PostInterventionRecalibrationController& controller,
    unsigned long nowMs) {
  controller.collector = RecalibrationCollector();
  controller.relaxedTracking = false;
  controller.retryPending = false;
  controller.hasAcceptedSample = false;
  controller.attemptStartMs = nowMs;
}

inline void schedulePostRecalibrationRetry(
    PostInterventionRecalibrationController& controller,
    unsigned long nowMs,
    const PostInterventionRecalibrationConfig& config) {
  controller.collector.active = false;
  controller.relaxedTracking = false;
  controller.hasAcceptedSample = false;
  if (controller.attempt < config.collector.maxAttempts) {
    controller.retryPending = true;
    controller.retryStartMs = nowMs;
  } else {
    controller.active = false;
    controller.retryPending = false;
    controller.terminalState = POST_RECAL_FAILED;
  }
}

inline PostInterventionRecalibrationOutput updatePostInterventionRecalibration(
    PostInterventionRecalibrationController& controller,
    const PostInterventionRecalibrationInput& input,
    const PostInterventionRecalibrationConfig& config) {
  PostInterventionRecalibrationOutput output;
  const bool configValid =
      isPostInterventionRecalibrationConfigValid(config);

  if (!input.enabled) {
    resetPostInterventionRecalibration(controller, input.startRequested);
    output.state = POST_RECAL_DISABLED;
    return output;
  }
  if (!configValid) {
    controller.active = false;
    controller.collector.active = false;
    controller.terminalState = POST_RECAL_CONFIG_ERROR;
    output.state = POST_RECAL_CONFIG_ERROR;
    output.rejectReason = POST_RECAL_REJECT_CONFIG;
    return output;
  }

  if (!input.startRequested) {
    controller.triggerArmed = true;
    if (!controller.active) controller.terminalState = POST_RECAL_IDLE;
  }

  if (input.cancelRequested && controller.active) {
    controller.active = false;
    controller.collector.active = false;
    controller.retryPending = false;
    controller.terminalState = POST_RECAL_CANCELLED;
    output.state = POST_RECAL_CANCELLED;
    return output;
  }

  if (!controller.active && input.startRequested &&
      controller.triggerArmed) {
    controller.triggerArmed = false;
    if (!isfinite(input.existingBaselineRms) ||
        input.existingBaselineRms < config.minimumExistingBaselineRms) {
      controller.terminalState = POST_RECAL_FAILED;
      output.state = POST_RECAL_FAILED;
      output.rejectReason = POST_RECAL_REJECT_BASELINE_INVALID;
      return output;
    }
    controller.active = true;
    controller.attempt = 1;
    controller.originalBaselineRms = input.existingBaselineRms;
    controller.terminalState = POST_RECAL_IDLE;
    beginPostRecalibrationAttempt(controller, input.nowMs);
  }

  if (!controller.active) {
    output.state = controller.terminalState;
    output.attempt = controller.attempt;
    return output;
  }

  output.attempt = controller.attempt;
  if (controller.retryPending) {
    if ((input.nowMs - controller.retryStartMs) < config.retryDelayMs) {
      output.state = POST_RECAL_RETRY_DELAY;
      return output;
    }
    ++controller.attempt;
    output.attempt = controller.attempt;
    beginPostRecalibrationAttempt(controller, input.nowMs);
  }

  if (!controller.collector.active &&
      (input.nowMs - controller.attemptStartMs) >=
          config.maximumRelaxedWaitMs) {
    output.rejectReason = POST_RECAL_REJECT_TIMEOUT;
    schedulePostRecalibrationRetry(controller, input.nowMs, config);
    output.state = controller.retryPending ? POST_RECAL_RETRY_DELAY
                                           : POST_RECAL_FAILED;
    return output;
  }

  const PostInterventionRecalibrationRejectReason gateReason =
      getPostRecalibrationGate(input);
  if (gateReason != POST_RECAL_REJECT_NONE) {
    output.rejectReason = gateReason;
    if (controller.collector.active) {
      schedulePostRecalibrationRetry(controller, input.nowMs, config);
      output.state = controller.retryPending ? POST_RECAL_RETRY_DELAY
                                             : POST_RECAL_FAILED;
      return output;
    }
    controller.relaxedTracking = false;
    output.state = POST_RECAL_WAITING_RELAXED;
    return output;
  }

  if (!controller.relaxedTracking) {
    controller.relaxedTracking = true;
    controller.relaxedStartMs = input.nowMs;
    output.state = POST_RECAL_WAITING_RELAXED;
    return output;
  }
  if (!controller.collector.active &&
      (input.nowMs - controller.relaxedStartMs) < config.relaxedStableMs) {
    output.state = POST_RECAL_WAITING_RELAXED;
    return output;
  }
  if (!controller.collector.active) {
    if (!startRecalibration(controller.collector, input.nowMs,
                            config.collector)) {
      controller.active = false;
      controller.terminalState = POST_RECAL_CONFIG_ERROR;
      output.state = POST_RECAL_CONFIG_ERROR;
      output.rejectReason = POST_RECAL_REJECT_CONFIG;
      return output;
    }
  }

  output.state = POST_RECAL_COLLECTING;
  output.collectedSamples = controller.collector.count;
  if ((input.nowMs - controller.collector.startMs) >=
      config.collector.timeoutMs) {
    output.rejectReason = POST_RECAL_REJECT_TIMEOUT;
    schedulePostRecalibrationRetry(controller, input.nowMs, config);
    output.state = controller.retryPending ? POST_RECAL_RETRY_DELAY
                                           : POST_RECAL_FAILED;
    return output;
  }
  if (!input.sampleReady) {
    output.rejectReason = POST_RECAL_REJECT_NO_SAMPLE;
    return output;
  }
  if ((input.nowMs - input.sampleTimeMs) > config.maximumSampleAgeMs) {
    output.rejectReason = POST_RECAL_REJECT_STALE_SAMPLE;
    return output;
  }
  if (!isfinite(input.rms) || input.rms < config.minimumAcceptedRms ||
      input.rms > config.maximumAcceptedRms) {
    output.rejectReason = POST_RECAL_REJECT_RMS_INVALID;
    return output;
  }
  if (controller.hasAcceptedSample &&
      (input.nowMs - controller.lastAcceptedSampleMs) <
          config.minimumSampleIntervalMs) {
    output.rejectReason = POST_RECAL_REJECT_SAMPLE_INTERVAL;
    return output;
  }

  const RecalibrationResult result = updateRecalibration(
      controller.collector, true, true, input.rms, input.nowMs,
      config.collector);
  controller.hasAcceptedSample = true;
  controller.lastAcceptedSampleMs = input.nowMs;
  output.sampleAccepted = true;
  output.collectedSamples = result.collectedSamples;

  if (result.status == RECALIBRATION_QUALITY_FAILED) {
    output.rejectReason = POST_RECAL_REJECT_QUALITY;
    schedulePostRecalibrationRetry(controller, input.nowMs, config);
    output.state = controller.retryPending ? POST_RECAL_RETRY_DELAY
                                           : POST_RECAL_FAILED;
    return output;
  }
  if (result.status == RECALIBRATION_TIMEOUT) {
    output.rejectReason = POST_RECAL_REJECT_TIMEOUT;
    schedulePostRecalibrationRetry(controller, input.nowMs, config);
    output.state = controller.retryPending ? POST_RECAL_RETRY_DELAY
                                           : POST_RECAL_FAILED;
    return output;
  }
  if (result.status == RECALIBRATION_COMPLETE) {
    const float shiftRatio =
        fabsf(result.baselineMean - controller.originalBaselineRms) /
        controller.originalBaselineRms;
    if (!isfinite(shiftRatio) ||
        shiftRatio > config.maximumBaselineShiftRatio) {
      output.baselineUpdated = false;
      output.rejectReason = POST_RECAL_REJECT_BASELINE_SHIFT;
      schedulePostRecalibrationRetry(controller, input.nowMs, config);
      output.state = controller.retryPending ? POST_RECAL_RETRY_DELAY
                                             : POST_RECAL_FAILED;
      return output;
    }
    controller.active = false;
    controller.terminalState = POST_RECAL_COMPLETE;
    output.state = POST_RECAL_COMPLETE;
    output.baselineUpdated = true;
    output.baselineMean = result.baselineMean;
    output.baselineStd = result.baselineStd;
  }
  return output;
}

#endif  // RECALIBRATION_LOGIC_H
