#pragma once

class TelemetryController {
 public:
  void begin();
  void publish(const float motion[6], int buttonBits, bool hidReportSent,
               const float* delta = nullptr);
  bool enabled() const;

 private:
  int tick_ = 0;
};
