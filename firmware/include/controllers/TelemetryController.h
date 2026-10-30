#pragma once

class TelemetryController {
 public:
  void begin();
  void publish(const float motion[6], int buttonBits, bool hidReportSent);
  // Raw sensor deltas (raw − baseline), for diagnosing a weak or noisy axis.
  void publishRaw(const float raw[9], const float* baseline);
  bool enabled() const;

 private:
  int tick_ = 0;
};
