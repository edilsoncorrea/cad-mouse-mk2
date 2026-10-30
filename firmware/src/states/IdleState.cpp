#include "states/IdleState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void IdleState::enter() {
  hasLastSample_ = false;
  lastActivityMs_ = millis();
  ledController.setSolid(Config::LED_IDLE_COLOR);
}

bool IdleState::handleCalibrationRequest() {
  if (inputController.takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  float raw[9] = {};
  sensorController.readRaw(raw);

  float motion[6] = {};
  motionController.compute(raw, sensorController.baseline(), dt, motion);

  if (motionController.hasMotionActivity()) {
    lastActivityMs_ = now;
  }

  const uint16_t buttonBits = inputController.buttonBits();
  const bool hidReportSent = hidController.sendReports(motion, buttonBits);
  if (telemetryController.enabled()) {
    telemetryController.publishRaw(raw, sensorController.baseline());
    telemetryController.publish(motion, buttonBits, hidReportSent);
  }
}

void IdleState::handleSleepTransition(unsigned long now) {
  const unsigned long inactiveMs = now - lastActivityMs_;
  if (inactiveMs >= Config::IDLE_SLEEP_TIMEOUT_MS) {
    stateMachine.changeState(&StateMachine::sleepState);
  }
}

void IdleState::update() {
  inputController.update();

  if (handleCalibrationRequest()) {
    return;
  }

  const unsigned long now = millis();
  if (inputController.takeActivity()) {
    lastActivityMs_ = now;
  }

  // Run the motion pipeline at a fixed rate, timed with micros(), and give
  // the filter the nominal period. A dt measured with millis() jitters by a
  // millisecond or more, and every change of dt changes the biquad
  // coefficients, which shows up as ripple on the output.
  const unsigned long nowUs = micros();
  if (hasLastSample_ && (nowUs - lastSampleUs_) < Config::SAMPLE_PERIOD_US) {
    handleSleepTransition(now);
    return;
  }
  lastSampleUs_ = nowUs;
  hasLastSample_ = true;
  const float dt = Config::SAMPLE_PERIOD_US / 1000000.0f;

  runMotionPipeline(dt, now);
  handleSleepTransition(now);
}

void IdleState::exit() {}
