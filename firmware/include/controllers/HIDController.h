#pragma once

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

class HIDController {
 public:
  void begin();
  void task();
  bool sendReports(const float motion[6], uint16_t buttonBits);

 private:
  // Report ID 1: Translation (X, Y, Z)
  struct __attribute__((packed)) ReportTranslation {
    int16_t x, y, z;
  };

  // Report ID 2: Rotation (Rx, Ry, Rz)
  struct __attribute__((packed)) ReportRotation {
    int16_t rx, ry, rz;
  };

  // Report ID 3: Buttons (32 bits for up to 32 buttons)
  struct __attribute__((packed)) ReportButtons {
    uint8_t bits[4];
  };

  static void makeReports(const float motion[6],
                           ReportTranslation& trans,
                           ReportRotation& rot);
  bool reportsChanged(const ReportTranslation& trans,
                      const ReportRotation& rot) const;

  Adafruit_USBD_HID usbHid_;
  uint16_t buttonBitsSent_ = 0;
  ReportTranslation lastSentTrans_{};
  ReportRotation lastSentRot_{};
};
