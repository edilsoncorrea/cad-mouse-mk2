#include "controllers/HIDController.h"

#include <math.h>

#include "Config.h"

namespace {
// 3Dconnexion-compatible HID Report Descriptor.
// Report 1: Translation (X, Y, Z) — 3 × int16
// Report 2: Rotation (Rx, Ry, Rz) — 3 × int16
// Report 3: Buttons — 32 bits
const uint8_t kHidReportDescriptor[] PROGMEM = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x08,        // USAGE (Multi-axis Controller)
    0xA1, 0x01,        // COLLECTION (Application)

    // --- Report 1: Translation ---
    0xA1, 0x00,        //   COLLECTION (Physical)
    0x85, 0x01,        //     REPORT_ID (1)
    0x16, 0x00, 0x80,  //     LOGICAL_MINIMUM (-32768)
    0x26, 0xFF, 0x7F,  //     LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x80,  //     PHYSICAL_MINIMUM (-32768)
    0x46, 0xFF, 0x7F,  //     PHYSICAL_MAXIMUM (32767)
    0x09, 0x30,        //     USAGE (X)
    0x09, 0x31,        //     USAGE (Y)
    0x09, 0x32,        //     USAGE (Z)
    0x75, 0x10,        //     REPORT_SIZE (16)
    0x95, 0x03,        //     REPORT_COUNT (3)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    // --- Report 2: Rotation ---
    0xA1, 0x00,        //   COLLECTION (Physical)
    0x85, 0x02,        //     REPORT_ID (2)
    0x16, 0x00, 0x80,  //     LOGICAL_MINIMUM (-32768)
    0x26, 0xFF, 0x7F,  //     LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x80,  //     PHYSICAL_MINIMUM (-32768)
    0x46, 0xFF, 0x7F,  //     PHYSICAL_MAXIMUM (32767)
    0x09, 0x33,        //     USAGE (Rx)
    0x09, 0x34,        //     USAGE (Ry)
    0x09, 0x35,        //     USAGE (Rz)
    0x75, 0x10,        //     REPORT_SIZE (16)
    0x95, 0x03,        //     REPORT_COUNT (3)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    // --- Report 3: Buttons ---
    0xA1, 0x00,        //   COLLECTION (Physical)
    0x85, 0x03,        //     REPORT_ID (3)
    0x05, 0x09,        //     USAGE_PAGE (Button)
    0x19, 0x01,        //     USAGE_MINIMUM (Button 1)
    0x29, 0x20,        //     USAGE_MAXIMUM (Button 32)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x95, 0x20,        //     REPORT_COUNT (32)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    0xC0               // END_COLLECTION
};
}  // namespace

void HIDController::begin() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  usbHid_.setReportDescriptor(kHidReportDescriptor,
                               sizeof(kHidReportDescriptor));
  usbHid_.setPollInterval(2);
  usbHid_.begin();
}

void HIDController::task() { TinyUSBDevice.task(); }

void HIDController::makeReports(const float motion[6],
                                 ReportTranslation& trans,
                                 ReportRotation& rot) {
  trans.x = static_cast<int16_t>(motion[0]);
  trans.y = static_cast<int16_t>(motion[1]);
  trans.z = static_cast<int16_t>(motion[2]);
  rot.rx = static_cast<int16_t>(motion[3]);
  rot.ry = static_cast<int16_t>(motion[4]);
  rot.rz = static_cast<int16_t>(motion[5]);
}

bool HIDController::reportsChanged(const ReportTranslation& trans,
                                    const ReportRotation& rot) const {
  return trans.x != lastSentTrans_.x ||
         trans.y != lastSentTrans_.y ||
         trans.z != lastSentTrans_.z ||
         rot.rx != lastSentRot_.rx ||
         rot.ry != lastSentRot_.ry ||
         rot.rz != lastSentRot_.rz;
}

bool HIDController::sendReports(const float motion[6], uint16_t buttonBits) {
  ReportTranslation trans{};
  ReportRotation rot{};
  makeReports(motion, trans, rot);

  const bool sendAxes = reportsChanged(trans, rot);
  const bool sendButtons = (buttonBits != buttonBitsSent_);

  if (!usbHid_.ready() || (!sendAxes && !sendButtons)) {
    return false;
  }

  if (sendAxes) {
    // Send translation report (ID 1)
    usbHid_.sendReport(0x01, &trans, sizeof(trans));
    delayMicroseconds(500);
    // Send rotation report (ID 2)
    usbHid_.sendReport(0x02, &rot, sizeof(rot));
    lastSentTrans_ = trans;
    lastSentRot_ = rot;
  }

  if (sendButtons) {
    ReportButtons btn{};
    btn.bits[0] = buttonBits & 0xFF;
    btn.bits[1] = (buttonBits >> 8) & 0xFF;
    btn.bits[2] = 0;
    btn.bits[3] = 0;
    usbHid_.sendReport(0x03, &btn, sizeof(btn));
    buttonBitsSent_ = buttonBits;
  }

  return true;
}
