#pragma once

#include "State.h"

class IdleState : public State {
 public:
  void enter() override;
  void update() override;
  void exit() override;

 private:
  bool handleCalibrationRequest();
  void runMotionPipeline(float dt, unsigned long now);
  void handleSleepTransition(unsigned long now);

  unsigned long lastSampleUs_ = 0;
  bool hasLastSample_ = false;
  unsigned long lastActivityMs_ = 0;
};
