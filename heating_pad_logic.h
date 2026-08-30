#ifndef HEATING_PAD_LOGIC_H
#define HEATING_PAD_LOGIC_H

#include <math.h>
#include <stdint.h>

// Pure, non-blocking heating-pad control. This header never reads a sensor,
// calls millis(), or writes a GPIO. The board adapter owns all hardware I/O.

struct HeatingPadConfig {
  float setTemperatureC = 40.0f;
  float hysteresisC = 0.5f;
  float minimumSetTemperatureC = 38.0f;
  float maximumSetTemperatureC = 43.0f;
  float emergencyTemperatureC = 45.0f;
  float minimumSensorTemperatureC = -20.0f;
  float maximumSensorTemperatureC = 85.0f;
  unsigned long maximumTemperatureSampleAgeMs = 1000UL;
  unsigned long minimumOffMs = 1000UL;

  // Both remain zero until the physical heater, sensor placement, skin-side
  // temperature, power stage and independent cutoffs have been verified.
  unsigned long maximumContinuousOnMs = 0UL;
  unsigned long maximumSessionMs = 0UL;
  unsigned long postSessionCooldownMs = 10000UL;

  uint8_t dutyPercent = 100U;
  bool requireThermalFuseHealthy = true;
  bool requireDriverHealthy = true;
};

inline bool isHeatingPadConfigValid(const HeatingPadConfig& config) {
  return isfinite(config.setTemperatureC) &&
         isfinite(config.hysteresisC) && config.hysteresisC > 0.0f &&
         isfinite(config.minimumSetTemperatureC) &&
         isfinite(config.maximumSetTemperatureC) &&
         isfinite(config.emergencyTemperatureC) &&
         isfinite(config.minimumSensorTemperatureC) &&
         isfinite(config.maximumSensorTemperatureC) &&
         config.minimumSensorTemperatureC < config.minimumSetTemperatureC &&
         config.minimumSetTemperatureC <= config.setTemperatureC &&
         config.setTemperatureC <= config.maximumSetTemperatureC &&
         config.maximumSetTemperatureC < config.emergencyTemperatureC &&
         config.emergencyTemperatureC <= config.maximumSensorTemperatureC &&
         config.maximumTemperatureSampleAgeMs > 0UL &&
         config.minimumOffMs > 0UL &&
         config.maximumContinuousOnMs > 0UL &&
         config.maximumSessionMs > 0UL &&
         config.maximumContinuousOnMs <= config.maximumSessionMs &&
         config.dutyPercent >= 1U && config.dutyPercent <= 100U;
}

enum HeatingPadState {
  HEATING_PAD_DISABLED,
  HEATING_PAD_IDLE,
  HEATING_PAD_HEATING,
  HEATING_PAD_TEMPERATURE_HOLD,
  HEATING_PAD_COOLDOWN,
  HEATING_PAD_FAULT
};

enum HeatingPadFault {
  HEATING_PAD_FAULT_NONE,
  HEATING_PAD_FAULT_CONFIG,
  HEATING_PAD_FAULT_TEMPERATURE_SENSOR,
  HEATING_PAD_FAULT_THERMAL_FUSE,
  HEATING_PAD_FAULT_DRIVER,
  HEATING_PAD_FAULT_EMERGENCY_STOP,
  HEATING_PAD_FAULT_OVERHEAT,
  HEATING_PAD_FAULT_CONTINUOUS_TIMEOUT
};

struct HeatingPadInput {
  unsigned long nowMs = 0UL;
  bool sessionRequested = false;
  bool enabled = false;
  float temperatureC = NAN;
  bool temperatureSampleAvailable = false;
  unsigned long temperatureSampleTimeMs = 0UL;
  bool thermalFuseHealthy = false;
  bool driverHealthy = false;
  bool emergencyStop = false;
  bool resetFault = false;
};

struct HeatingPadController {
  bool sessionActive = false;
  bool heaterOn = false;
  bool triggerArmed = true;
  bool cooldownActive = false;
  bool offLockoutActive = false;
  unsigned long sessionStartMs = 0UL;
  unsigned long heaterOnStartMs = 0UL;
  unsigned long heaterOffStartMs = 0UL;
  unsigned long cooldownStartMs = 0UL;
  HeatingPadFault fault = HEATING_PAD_FAULT_NONE;
};

struct HeatingPadOutput {
  HeatingPadState state = HEATING_PAD_DISABLED;
  HeatingPadFault fault = HEATING_PAD_FAULT_NONE;
  bool heaterOn = false;
  uint8_t dutyPercent = 0U;
  bool sessionStarted = false;
  bool sessionCompleted = false;
  bool sessionCancelled = false;
  bool triggerAccepted = false;
};

inline void stopHeatingPadOutput(HeatingPadController& controller) {
  controller.heaterOn = false;
  controller.offLockoutActive = false;
}

inline void stopHeatingPadSession(HeatingPadController& controller) {
  stopHeatingPadOutput(controller);
  controller.sessionActive = false;
}

inline void latchHeatingPadFault(HeatingPadController& controller,
                                 HeatingPadFault fault) {
  stopHeatingPadSession(controller);
  controller.cooldownActive = false;
  controller.fault = fault;
}

inline void resetHeatingPadController(HeatingPadController& controller,
                                      bool requestCurrentlyActive = false) {
  controller = HeatingPadController();
  controller.triggerArmed = !requestCurrentlyActive;
}

inline bool isHeatingPadTemperatureSampleValid(
    const HeatingPadInput& input,
    const HeatingPadConfig& config) {
  return input.temperatureSampleAvailable && isfinite(input.temperatureC) &&
         input.temperatureC >= config.minimumSensorTemperatureC &&
         input.temperatureC <= config.maximumSensorTemperatureC &&
         (input.nowMs - input.temperatureSampleTimeMs) <=
             config.maximumTemperatureSampleAgeMs;
}

inline HeatingPadFault getHeatingPadSafetyFault(
    const HeatingPadInput& input,
    const HeatingPadConfig& config) {
  if (input.emergencyStop) return HEATING_PAD_FAULT_EMERGENCY_STOP;
  if (config.requireThermalFuseHealthy && !input.thermalFuseHealthy) {
    return HEATING_PAD_FAULT_THERMAL_FUSE;
  }
  if (config.requireDriverHealthy && !input.driverHealthy) {
    return HEATING_PAD_FAULT_DRIVER;
  }
  if (!isHeatingPadTemperatureSampleValid(input, config)) {
    return HEATING_PAD_FAULT_TEMPERATURE_SENSOR;
  }
  if (input.temperatureC >= config.emergencyTemperatureC) {
    return HEATING_PAD_FAULT_OVERHEAT;
  }
  return HEATING_PAD_FAULT_NONE;
}

inline void beginHeatingPadCooldown(HeatingPadController& controller,
                                    unsigned long nowMs,
                                    const HeatingPadConfig& config) {
  stopHeatingPadSession(controller);
  controller.cooldownActive = config.postSessionCooldownMs > 0UL;
  controller.cooldownStartMs = nowMs;
}

inline HeatingPadOutput updateHeatingPad(
    HeatingPadController& controller,
    const HeatingPadInput& input,
    const HeatingPadConfig& config) {
  HeatingPadOutput output;
  const bool configValid = isHeatingPadConfigValid(config);

  if (input.emergencyStop) {
    latchHeatingPadFault(controller, HEATING_PAD_FAULT_EMERGENCY_STOP);
  } else if (input.enabled && !configValid) {
    latchHeatingPadFault(controller, HEATING_PAD_FAULT_CONFIG);
  }

  if (controller.fault != HEATING_PAD_FAULT_NONE) {
    bool resetIsSafe = input.resetFault && !input.sessionRequested &&
                       !input.emergencyStop;
    if (resetIsSafe && input.enabled) {
      resetIsSafe = configValid &&
                    getHeatingPadSafetyFault(input, config) ==
                        HEATING_PAD_FAULT_NONE;
    }
    if (resetIsSafe) {
      resetHeatingPadController(controller, input.sessionRequested);
    } else {
      output.state = HEATING_PAD_FAULT;
      output.fault = controller.fault;
      return output;
    }
  }

  if (!input.sessionRequested) controller.triggerArmed = true;

  if (!input.enabled) {
    stopHeatingPadSession(controller);
    controller.cooldownActive = false;
    output.state = HEATING_PAD_DISABLED;
    return output;
  }

  if (controller.sessionActive ||
      (input.sessionRequested && controller.triggerArmed &&
       !controller.cooldownActive)) {
    const HeatingPadFault safetyFault =
        getHeatingPadSafetyFault(input, config);
    if (safetyFault != HEATING_PAD_FAULT_NONE) {
      latchHeatingPadFault(controller, safetyFault);
      output.state = HEATING_PAD_FAULT;
      output.fault = controller.fault;
      return output;
    }
  }

  if (controller.sessionActive && !input.sessionRequested) {
    beginHeatingPadCooldown(controller, input.nowMs, config);
    output.sessionCancelled = true;
  }

  if (controller.cooldownActive) {
    if ((input.nowMs - controller.cooldownStartMs) <
        config.postSessionCooldownMs) {
      output.state = HEATING_PAD_COOLDOWN;
      return output;
    }
    controller.cooldownActive = false;
  }

  if (!controller.sessionActive && input.sessionRequested &&
      controller.triggerArmed) {
    controller.triggerArmed = false;
    controller.sessionActive = true;
    controller.sessionStartMs = input.nowMs;
    controller.offLockoutActive = false;
    if (input.temperatureC <=
        config.setTemperatureC - config.hysteresisC) {
      controller.heaterOn = true;
      controller.heaterOnStartMs = input.nowMs;
    }
    output.sessionStarted = true;
    output.triggerAccepted = true;
  }

  if (controller.sessionActive) {
    if ((input.nowMs - controller.sessionStartMs) >=
        config.maximumSessionMs) {
      beginHeatingPadCooldown(controller, input.nowMs, config);
      output.sessionCompleted = true;
    } else if (controller.heaterOn &&
               (input.nowMs - controller.heaterOnStartMs) >=
                   config.maximumContinuousOnMs) {
      latchHeatingPadFault(controller,
                           HEATING_PAD_FAULT_CONTINUOUS_TIMEOUT);
    } else if (controller.heaterOn &&
               input.temperatureC >= config.setTemperatureC) {
      controller.heaterOn = false;
      controller.offLockoutActive = true;
      controller.heaterOffStartMs = input.nowMs;
    } else if (!controller.heaterOn) {
      if (controller.offLockoutActive &&
          (input.nowMs - controller.heaterOffStartMs) >=
              config.minimumOffMs) {
        controller.offLockoutActive = false;
      }
      if (!controller.offLockoutActive &&
          input.temperatureC <=
              config.setTemperatureC - config.hysteresisC) {
        controller.heaterOn = true;
        controller.heaterOnStartMs = input.nowMs;
      }
    }
  }

  output.fault = controller.fault;
  if (controller.fault != HEATING_PAD_FAULT_NONE) {
    output.state = HEATING_PAD_FAULT;
    return output;
  }
  if (controller.sessionActive) {
    output.state = controller.heaterOn ? HEATING_PAD_HEATING
                                      : HEATING_PAD_TEMPERATURE_HOLD;
    output.heaterOn = controller.heaterOn;
    output.dutyPercent = output.heaterOn ? config.dutyPercent : 0U;
  } else if (controller.cooldownActive) {
    output.state = HEATING_PAD_COOLDOWN;
  } else {
    output.state = HEATING_PAD_IDLE;
  }
  return output;
}

#endif  // HEATING_PAD_LOGIC_H
