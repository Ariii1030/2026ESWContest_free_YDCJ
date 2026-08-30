#ifndef VIBRATION_MOTOR_LOGIC_H
#define VIBRATION_MOTOR_LOGIC_H

#include <stdint.h>

// Pure, non-blocking vibration timing. No GPIO, delay(), millis(), or motor
// driver calls are allowed in this header.

struct VibrationPatternConfig {
  unsigned long onMs = 250UL;
  unsigned long offMs = 250UL;
  uint8_t pulseCount = 3U;
  unsigned long maxContinuousOnMs = 1000UL;
  unsigned long maxTotalMs = 3000UL;
};

inline bool isVibrationPatternConfigValid(
    const VibrationPatternConfig& config) {
  if (config.onMs == 0UL || config.pulseCount == 0U ||
      config.pulseCount > 20U || config.maxContinuousOnMs == 0UL ||
      config.onMs > config.maxContinuousOnMs || config.maxTotalMs == 0UL) {
    return false;
  }
  const uint64_t requiredMs =
      static_cast<uint64_t>(config.onMs) * config.pulseCount +
      static_cast<uint64_t>(config.offMs) * (config.pulseCount - 1U);
  return requiredMs <= static_cast<uint64_t>(config.maxTotalMs);
}

enum VibrationPatternStatus {
  VIBRATION_IDLE,
  VIBRATION_ON_PHASE,
  VIBRATION_OFF_PHASE,
  VIBRATION_COMPLETE,
  VIBRATION_CONFIG_ERROR,
  VIBRATION_TIMEOUT,
  VIBRATION_CONTINUOUS_LIMIT
};

struct VibrationController {
  bool active = false;
  bool outputOn = false;
  uint8_t pulseIndex = 0U;
  unsigned long patternStartMs = 0UL;
  unsigned long phaseStartMs = 0UL;
};

struct VibrationPatternResult {
  bool outputOn = false;
  bool active = false;
  VibrationPatternStatus status = VIBRATION_IDLE;
};

inline void stopVibrationPattern(VibrationController& controller) {
  controller.active = false;
  controller.outputOn = false;
  controller.pulseIndex = 0U;
}

inline bool startVibrationPattern(VibrationController& controller,
                                  unsigned long nowMs,
                                  const VibrationPatternConfig& config) {
  stopVibrationPattern(controller);
  if (!isVibrationPatternConfigValid(config)) return false;
  controller.active = true;
  controller.outputOn = true;
  controller.patternStartMs = nowMs;
  controller.phaseStartMs = nowMs;
  return true;
}

inline VibrationPatternResult updateVibrationPattern(
    VibrationController& controller,
    unsigned long nowMs,
    const VibrationPatternConfig& config) {
  VibrationPatternResult result;
  if (!isVibrationPatternConfigValid(config)) {
    stopVibrationPattern(controller);
    result.status = VIBRATION_CONFIG_ERROR;
    return result;
  }
  if (!controller.active) return result;

  if (controller.outputOn &&
      (nowMs - controller.phaseStartMs) > config.maxContinuousOnMs) {
    stopVibrationPattern(controller);
    result.status = VIBRATION_CONTINUOUS_LIMIT;
    return result;
  }
  if ((nowMs - controller.patternStartMs) > config.maxTotalMs) {
    stopVibrationPattern(controller);
    result.status = VIBRATION_TIMEOUT;
    return result;
  }

  const int maxTransitions = static_cast<int>(config.pulseCount) * 2 + 1;
  for (int transition = 0; transition < maxTransitions; ++transition) {
    if (controller.outputOn) {
      if ((nowMs - controller.phaseStartMs) < config.onMs) {
        result.outputOn = true;
        result.active = true;
        result.status = VIBRATION_ON_PHASE;
        return result;
      }
      controller.phaseStartMs += config.onMs;
      controller.outputOn = false;
      if (controller.pulseIndex + 1U >= config.pulseCount) {
        stopVibrationPattern(controller);
        result.status = VIBRATION_COMPLETE;
        return result;
      }
    } else {
      if ((nowMs - controller.phaseStartMs) < config.offMs) {
        result.active = true;
        result.status = VIBRATION_OFF_PHASE;
        return result;
      }
      controller.phaseStartMs += config.offMs;
      ++controller.pulseIndex;
      controller.outputOn = true;
      if ((nowMs - controller.phaseStartMs) > config.maxContinuousOnMs) {
        stopVibrationPattern(controller);
        result.status = VIBRATION_CONTINUOUS_LIMIT;
        return result;
      }
    }
  }

  stopVibrationPattern(controller);
  result.status = VIBRATION_TIMEOUT;
  return result;
}

// Standalone motor-alert wrapper. The stage-5 state machine uses the compatible
// low-level pattern API above; this wrapper adds driver interlocks, cooldown,
// trigger rearming and fault latching for independent board use.

struct VibrationMotorConfig {
  VibrationPatternConfig pattern;
  uint8_t dutyPercent = 100U;
  unsigned long retriggerCooldownMs = 2000UL;
  bool requireDriverHealthy = true;
};

inline bool isVibrationMotorConfigValid(const VibrationMotorConfig& config) {
  return isVibrationPatternConfigValid(config.pattern) &&
         config.dutyPercent >= 1U && config.dutyPercent <= 100U;
}

enum VibrationMotorState {
  VIBRATION_MOTOR_DISABLED,
  VIBRATION_MOTOR_IDLE,
  VIBRATION_MOTOR_RUNNING,
  VIBRATION_MOTOR_COOLDOWN,
  VIBRATION_MOTOR_FAULT
};

enum VibrationMotorFault {
  VIBRATION_MOTOR_FAULT_NONE,
  VIBRATION_MOTOR_FAULT_CONFIG,
  VIBRATION_MOTOR_FAULT_DRIVER,
  VIBRATION_MOTOR_FAULT_EMERGENCY_STOP,
  VIBRATION_MOTOR_FAULT_PATTERN_TIMEOUT,
  VIBRATION_MOTOR_FAULT_CONTINUOUS_LIMIT
};

struct VibrationMotorInput {
  unsigned long nowMs = 0UL;
  bool trigger = false;
  bool enabled = false;
  bool driverHealthy = false;
  bool emergencyStop = false;
  bool resetFault = false;
};

struct VibrationMotorController {
  VibrationController pattern;
  bool triggerArmed = true;
  bool cooldownActive = false;
  unsigned long cooldownStartMs = 0UL;
  VibrationMotorFault fault = VIBRATION_MOTOR_FAULT_NONE;
};

struct VibrationMotorOutput {
  VibrationMotorState state = VIBRATION_MOTOR_DISABLED;
  VibrationMotorFault fault = VIBRATION_MOTOR_FAULT_NONE;
  bool motorOn = false;
  uint8_t dutyPercent = 0U;
  bool patternStarted = false;
  bool patternCompleted = false;
  bool triggerAccepted = false;
};

inline void latchVibrationMotorFault(VibrationMotorController& controller,
                                     VibrationMotorFault fault) {
  stopVibrationPattern(controller.pattern);
  controller.cooldownActive = false;
  controller.fault = fault;
}

inline void resetVibrationMotorController(
    VibrationMotorController& controller,
    bool triggerCurrentlyActive = false) {
  controller = VibrationMotorController();
  controller.triggerArmed = !triggerCurrentlyActive;
}

inline VibrationMotorOutput updateVibrationMotor(
    VibrationMotorController& controller,
    const VibrationMotorInput& input,
    const VibrationMotorConfig& config) {
  VibrationMotorOutput output;
  const bool configValid = isVibrationMotorConfigValid(config);

  if (!configValid) {
    latchVibrationMotorFault(controller, VIBRATION_MOTOR_FAULT_CONFIG);
  } else if (input.emergencyStop) {
    latchVibrationMotorFault(
        controller, VIBRATION_MOTOR_FAULT_EMERGENCY_STOP);
  } else if (input.enabled && config.requireDriverHealthy &&
             !input.driverHealthy) {
    latchVibrationMotorFault(controller, VIBRATION_MOTOR_FAULT_DRIVER);
  }

  if (controller.fault != VIBRATION_MOTOR_FAULT_NONE) {
    const bool resetIsSafe = input.resetFault && configValid &&
                             !input.emergencyStop &&
                             (!input.enabled || !config.requireDriverHealthy ||
                              input.driverHealthy);
    if (resetIsSafe) {
      resetVibrationMotorController(controller, input.trigger);
    } else {
      output.state = VIBRATION_MOTOR_FAULT;
      output.fault = controller.fault;
      return output;
    }
  }

  if (!input.trigger) controller.triggerArmed = true;

  if (!input.enabled) {
    stopVibrationPattern(controller.pattern);
    controller.cooldownActive = false;
    output.state = VIBRATION_MOTOR_DISABLED;
    return output;
  }

  if (controller.cooldownActive) {
    if ((input.nowMs - controller.cooldownStartMs) <
        config.retriggerCooldownMs) {
      output.state = VIBRATION_MOTOR_COOLDOWN;
      return output;
    }
    controller.cooldownActive = false;
  }

  if (!controller.pattern.active && input.trigger &&
      controller.triggerArmed) {
    controller.triggerArmed = false;
    if (!startVibrationPattern(controller.pattern, input.nowMs,
                               config.pattern)) {
      latchVibrationMotorFault(controller, VIBRATION_MOTOR_FAULT_CONFIG);
      output.state = VIBRATION_MOTOR_FAULT;
      output.fault = controller.fault;
      return output;
    }
    output.patternStarted = true;
    output.triggerAccepted = true;
  }

  if (controller.pattern.active) {
    const VibrationPatternResult pattern = updateVibrationPattern(
        controller.pattern, input.nowMs, config.pattern);
    if (pattern.status == VIBRATION_CONFIG_ERROR) {
      latchVibrationMotorFault(controller, VIBRATION_MOTOR_FAULT_CONFIG);
    } else if (pattern.status == VIBRATION_TIMEOUT) {
      latchVibrationMotorFault(
          controller, VIBRATION_MOTOR_FAULT_PATTERN_TIMEOUT);
    } else if (pattern.status == VIBRATION_CONTINUOUS_LIMIT) {
      latchVibrationMotorFault(
          controller, VIBRATION_MOTOR_FAULT_CONTINUOUS_LIMIT);
    } else if (pattern.status == VIBRATION_COMPLETE) {
      controller.cooldownActive = config.retriggerCooldownMs > 0UL;
      controller.cooldownStartMs = input.nowMs;
      output.patternCompleted = true;
    }
  }

  output.fault = controller.fault;
  if (controller.fault != VIBRATION_MOTOR_FAULT_NONE) {
    output.state = VIBRATION_MOTOR_FAULT;
    return output;
  }
  if (controller.pattern.active) {
    output.state = VIBRATION_MOTOR_RUNNING;
    output.motorOn = controller.pattern.outputOn;
    output.dutyPercent = output.motorOn ? config.dutyPercent : 0U;
  } else if (controller.cooldownActive) {
    output.state = VIBRATION_MOTOR_COOLDOWN;
  } else {
    output.state = VIBRATION_MOTOR_IDLE;
  }
  return output;
}

#endif  // VIBRATION_MOTOR_LOGIC_H
