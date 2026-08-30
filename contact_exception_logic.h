#ifndef CONTACT_EXCEPTION_LOGIC_H
#define CONTACT_EXCEPTION_LOGIC_H

#include <math.h>
#include <stdint.h>

// Compatible low-level moisture/contact gate extracted from stage 5.

struct ContactQualityConfig {
  float maxBaselineJumpRatio = 0.50f;
  float minimumBaselineRms = 1e-6f;
  unsigned long badSignalHoldMs = 3000UL;
  unsigned long recoveryStableMs = 2000UL;
};

inline bool isContactQualityConfigValid(const ContactQualityConfig& config) {
  return isfinite(config.maxBaselineJumpRatio) &&
         isfinite(config.minimumBaselineRms) &&
         config.maxBaselineJumpRatio >= 0.0f &&
         config.minimumBaselineRms > 0.0f &&
         config.badSignalHoldMs > 0UL &&
         config.recoveryStableMs > 0UL;
}

enum ContactBlockReason {
  CONTACT_OK,
  CONTACT_MOISTURE,
  CONTACT_SIGNAL_INVALID,
  CONTACT_BASELINE_JUMP,
  CONTACT_RECOVERY_HOLD,
  CONTACT_CONFIG_ERROR
};

struct ContactQualityGate {
  bool blocked = false;
  bool hasValidSample = false;
  bool recoveryTracking = false;
  unsigned long lastBadMs = 0UL;
  unsigned long recoveryStartMs = 0UL;
  ContactBlockReason lastBadReason = CONTACT_OK;
};

struct ContactQualityResult {
  bool measurementAllowed = false;
  ContactBlockReason reason = CONTACT_SIGNAL_INVALID;
};

inline void resetContactQualityGate(ContactQualityGate& gate) {
  gate = ContactQualityGate();
}

inline ContactQualityResult updateContactQualityGate(
    ContactQualityGate& gate,
    bool moistureDetected,
    bool sampleReady,
    bool signalValid,
    float currentRms,
    float baselineRms,
    unsigned long nowMs,
    const ContactQualityConfig& config) {
  ContactQualityResult result;
  if (!isContactQualityConfigValid(config)) {
    gate.blocked = true;
    gate.recoveryTracking = false;
    gate.lastBadMs = nowMs;
    gate.lastBadReason = CONTACT_CONFIG_ERROR;
    result.reason = CONTACT_CONFIG_ERROR;
    return result;
  }

  ContactBlockReason badReason = CONTACT_OK;
  if (moistureDetected) {
    badReason = CONTACT_MOISTURE;
  } else if (sampleReady &&
             (!signalValid || !isfinite(currentRms) ||
              !isfinite(baselineRms) || currentRms < 0.0f ||
              baselineRms < config.minimumBaselineRms)) {
    badReason = CONTACT_SIGNAL_INVALID;
  } else if (sampleReady) {
    const float jumpRatio = fabsf(currentRms - baselineRms) / baselineRms;
    if (!isfinite(jumpRatio) || jumpRatio > config.maxBaselineJumpRatio) {
      badReason = CONTACT_BASELINE_JUMP;
    }
  }

  if (badReason != CONTACT_OK) {
    gate.blocked = true;
    gate.recoveryTracking = false;
    gate.lastBadMs = nowMs;
    gate.lastBadReason = badReason;
    result.reason = badReason;
    return result;
  }

  if (!sampleReady && !gate.hasValidSample) {
    result.reason = CONTACT_SIGNAL_INVALID;
    return result;
  }
  if (sampleReady) gate.hasValidSample = true;

  if (!gate.blocked) {
    result.measurementAllowed = true;
    result.reason = CONTACT_OK;
    return result;
  }
  if (!sampleReady) {
    result.reason = CONTACT_RECOVERY_HOLD;
    return result;
  }
  if ((nowMs - gate.lastBadMs) < config.badSignalHoldMs) {
    result.reason = CONTACT_RECOVERY_HOLD;
    return result;
  }
  if (!gate.recoveryTracking) {
    gate.recoveryTracking = true;
    gate.recoveryStartMs = nowMs;
    result.reason = CONTACT_RECOVERY_HOLD;
    return result;
  }
  if ((nowMs - gate.recoveryStartMs) < config.recoveryStableMs) {
    result.reason = CONTACT_RECOVERY_HOLD;
    return result;
  }

  gate.blocked = false;
  gate.recoveryTracking = false;
  gate.lastBadReason = CONTACT_OK;
  result.measurementAllowed = true;
  result.reason = CONTACT_OK;
  return result;
}

// Enhanced stage-5 contact exception wrapper.

struct ContactExceptionConfig {
  ContactQualityConfig quality;
  unsigned long maximumSampleAgeMs = 500UL;
  unsigned long minimumSampleIntervalMs = 100UL;
  unsigned long maximumComparableSilenceMs = 2000UL;
  float maximumComparableRms = 1000000.0f;
};

inline bool isContactExceptionConfigValid(
    const ContactExceptionConfig& config) {
  return isContactQualityConfigValid(config.quality) &&
         config.maximumSampleAgeMs > 0UL &&
         config.minimumSampleIntervalMs > 0UL &&
         config.maximumComparableSilenceMs >=
             config.maximumSampleAgeMs &&
         isfinite(config.maximumComparableRms) &&
         config.maximumComparableRms > config.quality.minimumBaselineRms;
}

enum ContactExceptionState {
  CONTACT_EXCEPTION_DISABLED,
  CONTACT_EXCEPTION_WAITING_REFERENCE,
  CONTACT_EXCEPTION_HEALTHY,
  CONTACT_EXCEPTION_BLOCKED_MOISTURE,
  CONTACT_EXCEPTION_BLOCKED_SIGNAL,
  CONTACT_EXCEPTION_BLOCKED_JUMP,
  CONTACT_EXCEPTION_RECOVERING,
  CONTACT_EXCEPTION_WAITING_RECALIBRATION,
  CONTACT_EXCEPTION_CONFIG_ERROR
};

enum ContactExceptionReason {
  CONTACT_EXCEPTION_REASON_NONE,
  CONTACT_EXCEPTION_REASON_SENSOR_UNHEALTHY,
  CONTACT_EXCEPTION_REASON_MOISTURE,
  CONTACT_EXCEPTION_REASON_SIGNAL_INVALID,
  CONTACT_EXCEPTION_REASON_BASELINE_JUMP,
  CONTACT_EXCEPTION_REASON_NO_REFERENCE,
  CONTACT_EXCEPTION_REASON_STALE_SAMPLE,
  CONTACT_EXCEPTION_REASON_DUPLICATE_SAMPLE,
  CONTACT_EXCEPTION_REASON_CONTRACTION_EXCLUDED,
  CONTACT_EXCEPTION_REASON_RECOVERY_HOLD,
  CONTACT_EXCEPTION_REASON_RECALIBRATION_REQUIRED,
  CONTACT_EXCEPTION_REASON_CONFIG
};

struct ContactExceptionInput {
  unsigned long nowMs = 0UL;
  bool enabled = false;
  bool moistureSensorHealthy = false;
  bool moistureDetected = false;
  bool comparableSampleExpected = false;
  bool sampleReady = false;
  unsigned long sampleTimeMs = 0UL;
  bool signalValid = false;
  bool isContracting = false;
  float currentRms = NAN;
  float baselineRms = NAN;
  uint32_t baselineVersion = 0U;
  bool baselineRecalibrated = false;
};

struct ContactExceptionController {
  ContactQualityGate gate;
  bool enabledTracking = false;
  bool hasFreshComparableSample = false;
  bool hasBaselineVersion = false;
  bool blockSeen = false;
  bool recalibrationRequired = false;
  unsigned long enabledStartMs = 0UL;
  unsigned long lastFreshComparableMs = 0UL;
  unsigned long lastAcceptedSampleMs = 0UL;
  bool hasAcceptedSampleTime = false;
  uint32_t baselineVersion = 0U;
};

struct ContactExceptionOutput {
  ContactExceptionState state = CONTACT_EXCEPTION_DISABLED;
  ContactExceptionReason reason = CONTACT_EXCEPTION_REASON_NONE;
  bool measurementAllowed = false;
  bool excludeCurrentRmsFromContactComparison = true;
  bool contactSampleAccepted = false;
  bool recalibrationRequired = false;
  float baselineJumpRatio = NAN;
};

inline void resetContactExceptionController(
    ContactExceptionController& controller) {
  controller = ContactExceptionController();
}

inline void blockContactExceptionGate(ContactExceptionController& controller,
                                      ContactBlockReason reason,
                                      unsigned long nowMs) {
  controller.gate.blocked = true;
  controller.gate.recoveryTracking = false;
  controller.gate.lastBadMs = nowMs;
  controller.gate.lastBadReason = reason;
  controller.blockSeen = true;
}

inline ContactExceptionState contactBlockedState(ContactBlockReason reason) {
  if (reason == CONTACT_MOISTURE) {
    return CONTACT_EXCEPTION_BLOCKED_MOISTURE;
  }
  if (reason == CONTACT_BASELINE_JUMP) {
    return CONTACT_EXCEPTION_BLOCKED_JUMP;
  }
  return CONTACT_EXCEPTION_BLOCKED_SIGNAL;
}

inline ContactExceptionReason contactBlockedReason(ContactBlockReason reason) {
  if (reason == CONTACT_MOISTURE) return CONTACT_EXCEPTION_REASON_MOISTURE;
  if (reason == CONTACT_BASELINE_JUMP) {
    return CONTACT_EXCEPTION_REASON_BASELINE_JUMP;
  }
  return CONTACT_EXCEPTION_REASON_SIGNAL_INVALID;
}

inline ContactExceptionOutput updateContactException(
    ContactExceptionController& controller,
    const ContactExceptionInput& input,
    const ContactExceptionConfig& config) {
  ContactExceptionOutput output;
  if (!input.enabled) {
    resetContactExceptionController(controller);
    output.state = CONTACT_EXCEPTION_DISABLED;
    return output;
  }
  if (!isContactExceptionConfigValid(config)) {
    blockContactExceptionGate(controller, CONTACT_CONFIG_ERROR, input.nowMs);
    output.state = CONTACT_EXCEPTION_CONFIG_ERROR;
    output.reason = CONTACT_EXCEPTION_REASON_CONFIG;
    return output;
  }
  if (!controller.enabledTracking) {
    controller.enabledTracking = true;
    controller.enabledStartMs = input.nowMs;
  }

  const bool baselineChanged = !controller.hasBaselineVersion ||
      controller.baselineVersion != input.baselineVersion;
  if (baselineChanged || input.baselineRecalibrated) {
    resetContactQualityGate(controller.gate);
    controller.hasFreshComparableSample = false;
    controller.hasAcceptedSampleTime = false;
    controller.blockSeen = false;
    controller.recalibrationRequired = false;
    controller.enabledStartMs = input.nowMs;
    controller.baselineVersion = input.baselineVersion;
    controller.hasBaselineVersion = true;
  }

  if (!input.moistureSensorHealthy) {
    blockContactExceptionGate(controller, CONTACT_SIGNAL_INVALID, input.nowMs);
    output.state = CONTACT_EXCEPTION_BLOCKED_SIGNAL;
    output.reason = CONTACT_EXCEPTION_REASON_SENSOR_UNHEALTHY;
    output.recalibrationRequired = controller.recalibrationRequired;
    return output;
  }

  bool comparableReady = input.sampleReady &&
                         input.comparableSampleExpected &&
                         !input.isContracting;
  bool signalValid = input.signalValid;
  ContactExceptionReason localReason = CONTACT_EXCEPTION_REASON_NONE;

  if (input.sampleReady &&
      (!input.comparableSampleExpected || input.isContracting)) {
    comparableReady = false;
    localReason = CONTACT_EXCEPTION_REASON_CONTRACTION_EXCLUDED;
  } else if (comparableReady &&
             (input.nowMs - input.sampleTimeMs) >
                 config.maximumSampleAgeMs) {
    comparableReady = false;
    localReason = CONTACT_EXCEPTION_REASON_STALE_SAMPLE;
  } else if (comparableReady && controller.hasAcceptedSampleTime &&
             (input.nowMs - controller.lastAcceptedSampleMs) <
                 config.minimumSampleIntervalMs) {
    comparableReady = false;
    localReason = CONTACT_EXCEPTION_REASON_DUPLICATE_SAMPLE;
  }

  if (comparableReady &&
      (!isfinite(input.currentRms) || !isfinite(input.baselineRms) ||
       input.currentRms < 0.0f ||
       input.currentRms > config.maximumComparableRms ||
       input.baselineRms < config.quality.minimumBaselineRms ||
       input.baselineRms > config.maximumComparableRms)) {
    signalValid = false;
  }

  const bool comparisonAccepted = comparableReady && signalValid;
  if (comparisonAccepted) {
    controller.hasFreshComparableSample = true;
    controller.lastFreshComparableMs = input.nowMs;
    controller.hasAcceptedSampleTime = true;
    controller.lastAcceptedSampleMs = input.nowMs;
    output.contactSampleAccepted = true;
    output.excludeCurrentRmsFromContactComparison = false;
    output.baselineJumpRatio =
        fabsf(input.currentRms - input.baselineRms) / input.baselineRms;
  }

  const bool silenceExpired = input.comparableSampleExpected &&
      !input.isContracting && !comparisonAccepted &&
      ((controller.hasFreshComparableSample &&
        (input.nowMs - controller.lastFreshComparableMs) >=
            config.maximumComparableSilenceMs) ||
       (!controller.hasFreshComparableSample &&
        (input.nowMs - controller.enabledStartMs) >=
            config.maximumComparableSilenceMs));

  const bool forceInvalidSample =
      (comparableReady && !signalValid) || silenceExpired;
  const bool wasBlocked = controller.gate.blocked;
  const ContactQualityResult gateResult = updateContactQualityGate(
      controller.gate, input.moistureDetected,
      comparisonAccepted || forceInvalidSample,
      comparisonAccepted, input.currentRms, input.baselineRms,
      input.nowMs, config.quality);
  if (!wasBlocked && controller.gate.blocked) controller.blockSeen = true;

  if (controller.gate.blocked) {
    if (gateResult.reason == CONTACT_RECOVERY_HOLD) {
      output.state = CONTACT_EXCEPTION_RECOVERING;
      output.reason = CONTACT_EXCEPTION_REASON_RECOVERY_HOLD;
    } else {
      output.state = contactBlockedState(controller.gate.lastBadReason);
      output.reason = contactBlockedReason(controller.gate.lastBadReason);
    }
    if (!input.moistureSensorHealthy) {
      output.reason = CONTACT_EXCEPTION_REASON_SENSOR_UNHEALTHY;
    } else if (silenceExpired) {
      output.reason = CONTACT_EXCEPTION_REASON_SIGNAL_INVALID;
    }
    output.recalibrationRequired = controller.recalibrationRequired;
    return output;
  }

  if (gateResult.measurementAllowed && controller.blockSeen) {
    controller.recalibrationRequired = true;
  }
  if (controller.recalibrationRequired) {
    output.state = CONTACT_EXCEPTION_WAITING_RECALIBRATION;
    output.reason = CONTACT_EXCEPTION_REASON_RECALIBRATION_REQUIRED;
    output.recalibrationRequired = true;
    return output;
  }
  if (!controller.gate.hasValidSample) {
    output.state = CONTACT_EXCEPTION_WAITING_REFERENCE;
    output.reason = localReason != CONTACT_EXCEPTION_REASON_NONE
                        ? localReason
                        : CONTACT_EXCEPTION_REASON_NO_REFERENCE;
    return output;
  }

  output.state = CONTACT_EXCEPTION_HEALTHY;
  output.reason = localReason;
  output.measurementAllowed = gateResult.measurementAllowed;
  output.recalibrationRequired = false;
  return output;
}

#endif  // CONTACT_EXCEPTION_LOGIC_H
