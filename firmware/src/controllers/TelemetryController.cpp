#include "controllers/TelemetryController.h"
#include <Arduino.h>
#include "Config.h"

namespace {
const int kPrintEvery = 5;
}

void TelemetryController::begin() { tick_ = 0; }

bool TelemetryController::enabled() const { return Config::ENABLE_TELEMETRY; }

void TelemetryController::publish(const float motion[6], int buttonBits,
                                  bool hidReportSent, const float* delta) {
  if (!enabled()) {
    return;
  }

  tick_++;
  if ((tick_ % kPrintEvery) != 0) {
    return;
  }

  Serial.print(">X:");
  Serial.println(motion[0]);
  Serial.print(">Y:");
  Serial.println(motion[1]);
  Serial.print(">Z:");
  Serial.println(motion[2]);
  Serial.print(">Rx:");
  Serial.println(motion[3]);
  Serial.print(">Ry:");
  Serial.println(motion[4]);
  Serial.print(">Rz:");
  Serial.println(motion[5]);
  Serial.print(">btn:");
  Serial.println(buttonBits & 0x0003);
  Serial.print(">hid:");
  Serial.println(hidReportSent ? 1 : 0);

#ifdef TELEMETRY_RAW
  if (delta != nullptr) {
    static const char* const kNames[9] = {"m1x", "m1y", "m1z", "m2x", "m2y",
                                          "m2z", "m3x", "m3y", "m3z"};
    for (int i = 0; i < 9; i++) {
      Serial.print('>');
      Serial.print(kNames[i]);
      Serial.print(':');
      Serial.println(delta[i], 3);
    }
    // Tx before sign/gain: average of the three X components.
    Serial.print(">txRaw:");
    Serial.println((delta[0] + delta[3] + delta[6]) / 3.0f, 3);
  }
#endif
}
