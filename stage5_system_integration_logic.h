#ifndef STAGE5_SYSTEM_INTEGRATION_LOGIC_H
#define STAGE5_SYSTEM_INTEGRATION_LOGIC_H

#include <math.h>
#include <stdint.h>

#include "contact_exception_logic.h"
#include "heating_pad_logic.h"
#include "recalibration_logic.h"
#include "tm1637_display_logic.h"
#include "vibration_motor_logic.h"

// 5-6 top-level pure logic. Hardware acquisition and GPIO remain in the .ino.

struct Stage5SystemConfig {
  ContactExceptionConfig contact;
  VibrationMotorConfig vibration;
  HeatingPadConfig heating;
  PostInterventionRecalibrationConfig recalibration;
  Tm1637DisplayConfig display;
  unsigned long maximumSnapshotAgeMs = 500UL;
  bool vibrationHardwareEnabled = false;
  bool heatingHardwareEnabled = false;
  bool displayHardwareEnabled = false;
  bool requireStaticForFatigueIntervention = true;
};

inline bool isStage5SystemConfigValid(const Stage5SystemConfig& config) {
  return isContactExceptionConfigValid(config.contact) &&
         isVibrationMotorConfigValid(config.vibration) &&
         isPostInterventionRecalibrationConfigValid(config.recalibration) &&
         isTm1637DisplayConfigValid(config.display) &&
         config.maximumSnapshotAgeMs > 0UL &&
         (!config.heatingHardwareEnabled ||
          isHeatingPadConfigValid(config.heating));
}

enum Stage5SystemState {
  STAGE5_SYSTEM_DISABLED,
  STAGE5_SYSTEM_WAITING_DATA,
  STAGE5_SYSTEM_MEASURING,
  STAGE5_SYSTEM_VIBRATING,
  STAGE5_SYSTEM_HEATING,
  STAGE5_SYSTEM_RECALIBRATING,
  STAGE5_SYSTEM_CONTACT_BLOCKED,
  STAGE5_SYSTEM_WAITING_BASELINE_COMMIT,
  STAGE5_SYSTEM_WAITING_BASELINE_PUBLISH,
  STAGE5_SYSTEM_FAULT,
  STAGE5_SYSTEM_WAITING_MANUAL_BASELINE
};

enum Stage5SystemFault {
  STAGE5_SYSTEM_FAULT_NONE,
  STAGE5_SYSTEM_FAULT_CONFIG,
  STAGE5_SYSTEM_FAULT_FRAME_ORDER,
  STAGE5_SYSTEM_FAULT_EMERGENCY_STOP,
  STAGE5_SYSTEM_FAULT_VIBRATION,
  STAGE5_SYSTEM_FAULT_HEATING,
  STAGE5_SYSTEM_FAULT_RECALIBRATION,
  STAGE5_SYSTEM_FAULT_BASELINE_COMMIT
};

enum Stage5PipelineStatus {
  STAGE5_PIPELINE_DISABLED,
  STAGE5_PIPELINE_OK,
  STAGE5_PIPELINE_NO_SNAPSHOT,
  STAGE5_PIPELINE_STALE_SNAPSHOT,
  STAGE5_PIPELINE_DUPLICATE_SNAPSHOT,
  STAGE5_PIPELINE_OUT_OF_ORDER,
  STAGE5_PIPELINE_ACQUISITION_ERROR,
  STAGE5_PIPELINE_FILTER_ERROR,
  STAGE5_PIPELINE_INDICATOR_ERROR,
  STAGE5_PIPELINE_DECISION_ERROR,
  STAGE5_PIPELINE_MOTION_ERROR
};

struct Stage5SystemInput {
  unsigned long nowMs = 0UL;
  bool enabled = false;
  bool snapshotAvailable = false;
  unsigned long snapshotTimeMs = 0UL;
  uint32_t snapshotSequence = 0U;

  bool acquisitionHealthy = false;
  bool filteringHealthy = false;
  bool indicatorsHealthy = false;
  bool decisionHealthy = false;
  bool motionHealthy = false;

  bool measurementValid = false;
  bool fatigueDetected = false;
  bool forceIntervention = false;
  bool isMdfEligible = false;
  bool isImuStaticNow = false;
  bool isContracting = false;

  bool relaxedRmsExpected = false;
  bool rmsSampleReady = false;
  unsigned long rmsSampleTimeMs = 0UL;
  bool rmsSignalValid = false;
  float currentRms = NAN;
  float baselineRms = NAN;
  uint32_t baselineVersion = 0U;
  bool baselineRecalibrated = false;

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

struct Stage5SystemController {
  Stage5SystemState state = STAGE5_SYSTEM_DISABLED;
  Stage5SystemFault fault = STAGE5_SYSTEM_FAULT_NONE;
  ContactExceptionController contact;
  VibrationMotorController vibration;
  HeatingPadController heating;
  PostInterventionRecalibrationController recalibration;
  Tm1637DisplayController display;
  bool fatigueArmed = true;
  bool hasSnapshotSequence = false;
  uint32_t lastSnapshotSequence = 0U;
  bool baselineCommitPending = false;
  bool baselinePublishPending = false;
  float pendingBaselineMean = 0.0f;
  float pendingBaselineStd = 0.0f;
  uint32_t pendingSourceBaselineVersion = 0U;
  uint32_t expectedPublishedBaselineVersion = 0U;
  bool awaitingManualBaseline = false;
};

struct Stage5SystemOutput {
  Stage5SystemState state = STAGE5_SYSTEM_DISABLED;
  Stage5SystemFault fault = STAGE5_SYSTEM_FAULT_NONE;
  Stage5PipelineStatus pipelineStatus = STAGE5_PIPELINE_DISABLED;
  ContactExceptionState contactState = CONTACT_EXCEPTION_DISABLED;
  PostInterventionRecalibrationState recalibrationState =
      POST_RECAL_DISABLED;
  bool measurementAllowed = false;
  bool interventionAccepted = false;
  bool vibrationOn = false;
  uint8_t vibrationDutyPercent = 0U;
  bool heaterOn = false;
  uint8_t heaterDutyPercent = 0U;
  bool baselineCommitRequested = false;
  float baselineMean = 0.0f;
  float baselineStd = 0.0f;
  Tm1637DisplayOutput display;
};

inline void resetStage5SystemController(Stage5SystemController& controller,
                                        bool fatigueCurrentlyActive = false) {
  controller = Stage5SystemController();
  controller.fatigueArmed = !fatigueCurrentlyActive;
}

inline void latchStage5SystemFault(Stage5SystemController& controller,
                                   Stage5SystemFault fault) {
  controller.fault = fault;
  controller.state = STAGE5_SYSTEM_FAULT;
  resetVibrationMotorController(controller.vibration, true);
  resetHeatingPadController(controller.heating, true);
  resetPostInterventionRecalibration(controller.recalibration, false);
}

inline bool isStage5SystemPipelineHealthy(Stage5PipelineStatus status) {
  return status == STAGE5_PIPELINE_OK ||
         status == STAGE5_PIPELINE_DUPLICATE_SNAPSHOT;
}

inline Stage5PipelineStatus evaluateStage5Pipeline(
    Stage5SystemController& controller,
    const Stage5SystemInput& input,
    const Stage5SystemConfig& config,
    bool& isNewSnapshot) {
  isNewSnapshot = false;
  if (!input.enabled) return STAGE5_PIPELINE_DISABLED;
  if (!input.snapshotAvailable) return STAGE5_PIPELINE_NO_SNAPSHOT;
  if ((input.nowMs - input.snapshotTimeMs) > config.maximumSnapshotAgeMs) {
    return STAGE5_PIPELINE_STALE_SNAPSHOT;
  }
  if (!input.acquisitionHealthy) return STAGE5_PIPELINE_ACQUISITION_ERROR;
  if (!input.filteringHealthy) return STAGE5_PIPELINE_FILTER_ERROR;
  if (!input.indicatorsHealthy) return STAGE5_PIPELINE_INDICATOR_ERROR;
  if (!input.decisionHealthy) return STAGE5_PIPELINE_DECISION_ERROR;
  if (!input.motionHealthy) return STAGE5_PIPELINE_MOTION_ERROR;
  if (controller.hasSnapshotSequence) {
    const uint32_t delta =
        input.snapshotSequence - controller.lastSnapshotSequence;
    if (delta == 0U) return STAGE5_PIPELINE_DUPLICATE_SNAPSHOT;
    if (delta >= 0x80000000U) return STAGE5_PIPELINE_OUT_OF_ORDER;
  }
  controller.hasSnapshotSequence = true;
  controller.lastSnapshotSequence = input.snapshotSequence;
  isNewSnapshot = true;
  return STAGE5_PIPELINE_OK;
}

inline Tm1637Glyph stage5SystemDigit(int value) {
  return value >= 0 && value <= 9
             ? static_cast<Tm1637Glyph>(value)
             : TM_GLYPH_DASH;
}

inline Tm1637LogicalFrame buildStage5SystemDisplayFrame(
    Stage5SystemState state, Stage5SystemFault fault, float temperatureC) {
  Tm1637LogicalFrame frame;
  if (state == STAGE5_SYSTEM_FAULT) {
    frame.glyphs[0] = TM_GLYPH_E;
    frame.glyphs[1] = TM_GLYPH_0;
    frame.glyphs[2] = stage5SystemDigit(
        (static_cast<int>(fault) / 10) % 10);
    frame.glyphs[3] = stage5SystemDigit(static_cast<int>(fault) % 10);
    return frame;
  }
  if (state == STAGE5_SYSTEM_VIBRATING) {
    frame.glyphs[0] = TM_GLYPH_A;
    frame.glyphs[1] = TM_GLYPH_L;
    frame.glyphs[2] = TM_GLYPH_R;
    frame.glyphs[3] = TM_GLYPH_T;
    return frame;
  }
  if (state == STAGE5_SYSTEM_RECALIBRATING ||
      state == STAGE5_SYSTEM_WAITING_MANUAL_BASELINE ||
      state == STAGE5_SYSTEM_WAITING_BASELINE_COMMIT ||
      state == STAGE5_SYSTEM_WAITING_BASELINE_PUBLISH) {
    frame.glyphs[0] = TM_GLYPH_R;
    frame.glyphs[1] = TM_GLYPH_C;
    frame.glyphs[2] = TM_GLYPH_A;
    frame.glyphs[3] = TM_GLYPH_L;
    return frame;
  }
  if (state == STAGE5_SYSTEM_CONTACT_BLOCKED ||
      state == STAGE5_SYSTEM_WAITING_DATA) {
    frame.glyphs[0] = TM_GLYPH_C;
    frame.glyphs[1] = TM_GLYPH_O;
    frame.glyphs[2] = TM_GLYPH_N;
    frame.glyphs[3] = TM_GLYPH_T;
    return frame;
  }
  if (isfinite(temperatureC) && temperatureC >= 0.0f &&
      temperatureC < 100.0f) {
    const int tenths = static_cast<int>(floorf(temperatureC * 10.0f + 0.5f));
    if (tenths < 1000) {
      frame.glyphs[0] = stage5SystemDigit((tenths / 100) % 10);
      frame.glyphs[1] = stage5SystemDigit((tenths / 10) % 10);
      frame.glyphs[2] = stage5SystemDigit(tenths % 10);
      frame.glyphs[3] = TM_GLYPH_C;
      frame.decimalPointMask = 0x02U;
    } else {
      frame.glyphs[0] = TM_GLYPH_DASH;
      frame.glyphs[1] = TM_GLYPH_DASH;
      frame.glyphs[2] = TM_GLYPH_DASH;
      frame.glyphs[3] = TM_GLYPH_C;
    }
  } else {
    frame.glyphs[0] = TM_GLYPH_DASH;
    frame.glyphs[1] = TM_GLYPH_DASH;
    frame.glyphs[2] = TM_GLYPH_DASH;
    frame.glyphs[3] = TM_GLYPH_C;
  }
  return frame;
}

inline void finalizeStage5SystemOutput(
    Stage5SystemController& controller,
    const Stage5SystemInput& input,
    const Stage5SystemConfig& config,
    Stage5SystemOutput& output) {
  output.state = controller.state;
  output.fault = controller.fault;
  const Tm1637LogicalFrame frame = buildStage5SystemDisplayFrame(
      controller.state, controller.fault, input.temperatureC);
  output.display = updateTm1637Display(
      controller.display, frame, config.displayHardwareEnabled,
      controller.state == STAGE5_SYSTEM_FAULT, input.nowMs,
      config.display);
}

inline bool acknowledgeStage5SystemBaselineCommit(
    Stage5SystemController& controller,
    bool commitSucceeded,
    uint32_t publishedBaselineVersion) {
  if (!controller.baselineCommitPending) return false;
  if (!commitSucceeded ||
      publishedBaselineVersion == controller.pendingSourceBaselineVersion) {
    controller.baselineCommitPending = false;
    latchStage5SystemFault(controller,
                           STAGE5_SYSTEM_FAULT_BASELINE_COMMIT);
    return true;
  }
  controller.baselineCommitPending = false;
  controller.baselinePublishPending = true;
  controller.expectedPublishedBaselineVersion = publishedBaselineVersion;
  controller.state = STAGE5_SYSTEM_WAITING_BASELINE_PUBLISH;
  return true;
}

inline Stage5SystemOutput updateStage5System(
    Stage5SystemController& controller,
    const Stage5SystemInput& input,
    const Stage5SystemConfig& config) {
  Stage5SystemOutput output;
  if (!input.enabled) {
    resetStage5SystemController(controller, input.fatigueDetected);
    output.pipelineStatus = STAGE5_PIPELINE_DISABLED;
    finalizeStage5SystemOutput(controller, input, config, output);
    return output;
  }

  if (!isStage5SystemConfigValid(config)) {
    latchStage5SystemFault(controller, STAGE5_SYSTEM_FAULT_CONFIG);
  } else if (input.emergencyStop) {
    latchStage5SystemFault(controller,
                           STAGE5_SYSTEM_FAULT_EMERGENCY_STOP);
  }

  if (controller.fault != STAGE5_SYSTEM_FAULT_NONE) {
    const bool safeReset = input.userResetRequested &&
                           isStage5SystemConfigValid(config) &&
                           !input.emergencyStop;
    if (safeReset) {
      resetStage5SystemController(controller, input.fatigueDetected);
      controller.state = STAGE5_SYSTEM_WAITING_DATA;
    } else {
      output.pipelineStatus = STAGE5_PIPELINE_NO_SNAPSHOT;
      finalizeStage5SystemOutput(controller, input, config, output);
      return output;
    }
  }

  bool isNewSnapshot = false;
  output.pipelineStatus = evaluateStage5Pipeline(
      controller, input, config, isNewSnapshot);
  if (output.pipelineStatus == STAGE5_PIPELINE_OUT_OF_ORDER) {
    latchStage5SystemFault(controller,
                           STAGE5_SYSTEM_FAULT_FRAME_ORDER);
    finalizeStage5SystemOutput(controller, input, config, output);
    return output;
  }
  const bool pipelineHealthy =
      isStage5SystemPipelineHealthy(output.pipelineStatus);
  // A fatigue decision may arrive shortly after its RMS/MDF frame. Waiting
  // for that exact result must stop intervention, but must not be reported to
  // the contact controller as a moisture-sensor or RMS acquisition failure.
  // Only the acquisition-related stages determine contact-data health.
  const bool contactDataHealthy = input.snapshotAvailable &&
      (input.nowMs - input.snapshotTimeMs) <= config.maximumSnapshotAgeMs &&
      input.acquisitionHealthy && input.filteringHealthy &&
      input.indicatorsHealthy && input.motionHealthy;

  ContactExceptionInput contactInput;
  contactInput.nowMs = input.nowMs;
  contactInput.enabled = true;
  contactInput.moistureSensorHealthy =
      contactDataHealthy && input.moistureSensorHealthy;
  contactInput.moistureDetected =
      contactDataHealthy && input.moistureDetected;
  contactInput.comparableSampleExpected =
      contactDataHealthy && input.relaxedRmsExpected;
  contactInput.sampleReady = contactDataHealthy && isNewSnapshot &&
                             input.rmsSampleReady;
  contactInput.sampleTimeMs = input.rmsSampleTimeMs;
  contactInput.signalValid = contactDataHealthy && input.rmsSignalValid;
  contactInput.isContracting = input.isContracting;
  contactInput.currentRms = input.currentRms;
  contactInput.baselineRms = input.baselineRms;
  contactInput.baselineVersion =
      pipelineHealthy && isNewSnapshot
          ? input.baselineVersion
          : (controller.contact.hasBaselineVersion
                 ? controller.contact.baselineVersion
                 : input.baselineVersion);
  contactInput.baselineRecalibrated =
      pipelineHealthy && isNewSnapshot && input.baselineRecalibrated;
  const ContactExceptionOutput contactOutput = updateContactException(
      controller.contact, contactInput, config.contact);
  output.contactState = contactOutput.state;

  const bool contactHealthy =
      contactOutput.state == CONTACT_EXCEPTION_HEALTHY &&
      contactOutput.measurementAllowed;
  const bool contactReadyForRecalibration =
      contactHealthy ||
      contactOutput.state == CONTACT_EXCEPTION_WAITING_RECALIBRATION;

  if (controller.baselinePublishPending && pipelineHealthy &&
      isNewSnapshot &&
      input.baselineVersion ==
          controller.expectedPublishedBaselineVersion) {
    controller.baselinePublishPending = false;
    resetPostInterventionRecalibration(controller.recalibration, false);
    controller.state = STAGE5_SYSTEM_MEASURING;
  }

  if ((!pipelineHealthy ||
       (!contactHealthy && !contactReadyForRecalibration)) &&
      !input.forceIntervention) {
    VibrationMotorInput vibrationInput;
    vibrationInput.nowMs = input.nowMs;
    vibrationInput.enabled = false;
    (void)updateVibrationMotor(controller.vibration, vibrationInput,
                               config.vibration);
    HeatingPadInput heatingInput;
    heatingInput.nowMs = input.nowMs;
    heatingInput.enabled = false;
    (void)updateHeatingPad(controller.heating, heatingInput,
                           config.heating);
    resetPostInterventionRecalibration(controller.recalibration, false);
    controller.state = controller.awaitingManualBaseline
        ? STAGE5_SYSTEM_WAITING_MANUAL_BASELINE
        : (pipelineHealthy ? STAGE5_SYSTEM_CONTACT_BLOCKED
                           : STAGE5_SYSTEM_WAITING_DATA);
    finalizeStage5SystemOutput(controller, input, config, output);
    return output;
  }

  if (controller.baselineCommitPending) {
    controller.state = STAGE5_SYSTEM_WAITING_BASELINE_COMMIT;
    output.baselineCommitRequested = true;
    output.baselineMean = controller.pendingBaselineMean;
    output.baselineStd = controller.pendingBaselineStd;
    finalizeStage5SystemOutput(controller, input, config, output);
    return output;
  }
  if (controller.baselinePublishPending) {
    controller.state = STAGE5_SYSTEM_WAITING_BASELINE_PUBLISH;
    finalizeStage5SystemOutput(controller, input, config, output);
    return output;
  }

  if (input.forceIntervention &&
      controller.fault == STAGE5_SYSTEM_FAULT_NONE) {
    controller.awaitingManualBaseline = false;
    controller.state = STAGE5_SYSTEM_MEASURING;
  } else if (controller.awaitingManualBaseline && pipelineHealthy &&
      isNewSnapshot && input.baselineRecalibrated) {
    controller.awaitingManualBaseline = false;
    controller.fatigueArmed = true;
    controller.state = STAGE5_SYSTEM_MEASURING;
  } else if (controller.awaitingManualBaseline) {
    controller.state = STAGE5_SYSTEM_WAITING_MANUAL_BASELINE;
  } else if (contactOutput.state ==
             CONTACT_EXCEPTION_WAITING_RECALIBRATION) {
    // Do not automatically replace the user's baseline because of a transient
    // contact event. Wait for a deliberate physical-button calibration.
    controller.state = STAGE5_SYSTEM_CONTACT_BLOCKED;
  } else if (controller.state == STAGE5_SYSTEM_DISABLED ||
             controller.state == STAGE5_SYSTEM_WAITING_DATA ||
             controller.state == STAGE5_SYSTEM_CONTACT_BLOCKED) {
    controller.state = STAGE5_SYSTEM_MEASURING;
  }

  const bool fatigueUsable = input.forceIntervention ||
      (pipelineHealthy && contactHealthy && input.measurementValid &&
       input.isMdfEligible &&
       (!config.requireStaticForFatigueIntervention ||
        input.isImuStaticNow));
  if (pipelineHealthy && isNewSnapshot && !input.fatigueDetected) {
    controller.fatigueArmed = true;
  }
  if ((controller.state == STAGE5_SYSTEM_MEASURING ||
       input.forceIntervention) &&
      // The MDF decision is published just after its measurement frame. The
      // controller may therefore see the frame once before the decision and
      // see the matching positive decision on the duplicate-frame update.
      fatigueUsable && input.fatigueDetected &&
      controller.fatigueArmed) {
    controller.fatigueArmed = false;
    output.interventionAccepted = true;
    controller.state = config.vibrationHardwareEnabled
                           ? STAGE5_SYSTEM_VIBRATING
                           : (config.heatingHardwareEnabled
                                  ? STAGE5_SYSTEM_HEATING
                                  : STAGE5_SYSTEM_RECALIBRATING);
  }

  VibrationMotorInput vibrationInput;
  vibrationInput.nowMs = input.nowMs;
  vibrationInput.enabled = controller.state == STAGE5_SYSTEM_VIBRATING &&
                           config.vibrationHardwareEnabled;
  vibrationInput.trigger = vibrationInput.enabled;
  vibrationInput.driverHealthy = input.vibrationDriverHealthy;
  vibrationInput.emergencyStop = input.emergencyStop;
  vibrationInput.resetFault = input.userResetRequested;
  const VibrationMotorOutput vibrationOutput = updateVibrationMotor(
      controller.vibration, vibrationInput, config.vibration);
  output.vibrationOn = vibrationOutput.motorOn;
  output.vibrationDutyPercent = vibrationOutput.dutyPercent;
  if (vibrationOutput.fault != VIBRATION_MOTOR_FAULT_NONE) {
    latchStage5SystemFault(controller, STAGE5_SYSTEM_FAULT_VIBRATION);
  } else if (controller.state == STAGE5_SYSTEM_VIBRATING &&
             vibrationOutput.patternCompleted) {
    controller.state = config.heatingHardwareEnabled
                           ? STAGE5_SYSTEM_HEATING
                           : STAGE5_SYSTEM_RECALIBRATING;
  }

  HeatingPadInput heatingInput;
  heatingInput.nowMs = input.nowMs;
  heatingInput.enabled = controller.state == STAGE5_SYSTEM_HEATING &&
                         config.heatingHardwareEnabled;
  heatingInput.sessionRequested = heatingInput.enabled;
  heatingInput.temperatureC = input.temperatureC;
  heatingInput.temperatureSampleAvailable =
      pipelineHealthy && input.temperatureSampleAvailable;
  heatingInput.temperatureSampleTimeMs = input.temperatureSampleTimeMs;
  heatingInput.thermalFuseHealthy = input.thermalFuseHealthy;
  heatingInput.driverHealthy = input.heaterDriverHealthy;
  heatingInput.emergencyStop = input.emergencyStop;
  heatingInput.resetFault = input.userResetRequested;
  const HeatingPadOutput heatingOutput = updateHeatingPad(
      controller.heating, heatingInput, config.heating);
  output.heaterOn = heatingOutput.heaterOn;
  output.heaterDutyPercent = heatingOutput.dutyPercent;
  if (heatingOutput.fault != HEATING_PAD_FAULT_NONE) {
    latchStage5SystemFault(controller, STAGE5_SYSTEM_FAULT_HEATING);
  } else if (controller.state == STAGE5_SYSTEM_HEATING &&
             heatingOutput.sessionCompleted) {
    controller.awaitingManualBaseline = true;
    controller.state = STAGE5_SYSTEM_WAITING_MANUAL_BASELINE;
  }

  PostInterventionRecalibrationInput recalibrationInput;
  recalibrationInput.nowMs = input.nowMs;
  recalibrationInput.enabled = true;
  recalibrationInput.startRequested =
      controller.state == STAGE5_SYSTEM_RECALIBRATING;
  recalibrationInput.existingBaselineRms = input.baselineRms;
  recalibrationInput.isImuStaticNow = input.isImuStaticNow;
  recalibrationInput.isContracting = input.isContracting;
  recalibrationInput.contactHealthy = contactReadyForRecalibration;
  recalibrationInput.moistureDetected = input.moistureDetected;
  recalibrationInput.signalValid = input.rmsSignalValid;
  recalibrationInput.sampleReady = isNewSnapshot && input.rmsSampleReady;
  recalibrationInput.sampleTimeMs = input.rmsSampleTimeMs;
  recalibrationInput.rms = input.currentRms;
  const PostInterventionRecalibrationOutput recalibrationOutput =
      updatePostInterventionRecalibration(
          controller.recalibration, recalibrationInput,
          config.recalibration);
  output.recalibrationState = recalibrationOutput.state;
  if (controller.state == STAGE5_SYSTEM_RECALIBRATING &&
      recalibrationOutput.baselineUpdated) {
    controller.baselineCommitPending = true;
    controller.pendingBaselineMean = recalibrationOutput.baselineMean;
    controller.pendingBaselineStd = recalibrationOutput.baselineStd;
    controller.pendingSourceBaselineVersion = input.baselineVersion;
    controller.state = STAGE5_SYSTEM_WAITING_BASELINE_COMMIT;
    output.baselineCommitRequested = true;
    output.baselineMean = recalibrationOutput.baselineMean;
    output.baselineStd = recalibrationOutput.baselineStd;
  } else if (controller.state == STAGE5_SYSTEM_RECALIBRATING &&
             (recalibrationOutput.state == POST_RECAL_FAILED ||
              recalibrationOutput.state == POST_RECAL_CONFIG_ERROR)) {
    latchStage5SystemFault(controller,
                           STAGE5_SYSTEM_FAULT_RECALIBRATION);
  }

  if (controller.fault != STAGE5_SYSTEM_FAULT_NONE) {
    output.vibrationOn = false;
    output.vibrationDutyPercent = 0U;
    output.heaterOn = false;
    output.heaterDutyPercent = 0U;
  }
  output.measurementAllowed =
      controller.state == STAGE5_SYSTEM_MEASURING &&
      controller.fault == STAGE5_SYSTEM_FAULT_NONE &&
      fatigueUsable && !input.fatigueDetected;
  // A valid fatigue-positive frame is still a valid measurement. It is kept
  // separate from intervention output to prevent accidental re-triggering.
  if (controller.state == STAGE5_SYSTEM_MEASURING &&
      controller.fault == STAGE5_SYSTEM_FAULT_NONE &&
      pipelineHealthy && contactHealthy && input.measurementValid &&
      input.isMdfEligible) {
    output.measurementAllowed = true;
  }
  finalizeStage5SystemOutput(controller, input, config, output);
  return output;
}

#endif  // STAGE5_SYSTEM_INTEGRATION_LOGIC_H
