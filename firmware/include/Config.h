#pragma once

#include <Arduino.h>

namespace Config {

const bool ENABLE_TELEMETRY = true;

// Hardware pins (XIAO RP2040)
const int PIN_RIGHT_BTN = D0;
const int PIN_LEFT_BTN = D2;
const int PIN_LED_DATA = D3;
const int PIN_LED_LS = D1;
const int PIN_MAG1_LS = D10;
const int PIN_MAG2_LS = D9;
const int PIN_MAG3_LS = D8;

// Samples for calibration offset
const int ZERO_SAMPLES = 200;

// Sensor-to-axis transformation matrix (6 axes × 9 sensor components).
// Column order: [m1x, m1y, m1z, m2x, m2y, m2z, m3x, m3y, m3z]
// Default reproduces the original hard-coded formulas.
//   √3/3 ≈ 0.5773503,  √3/6 ≈ 0.2886751,  2√3/3 ≈ 1.1547005
const float TRANSFORM[6][9] = {
  // Tx = (m1x + m2x + m3x) / 3
  { 0.3333333f, 0, 0,  0.3333333f, 0, 0,  0.3333333f, 0, 0 },
  // Ty = (m1y + m2y + m3y) / 3
  { 0, 0.3333333f, 0,  0, 0.3333333f, 0,  0, 0.3333333f, 0 },
  // Tz = (m1z + m2z + m3z) / 3
  { 0, 0, 0.3333333f,  0, 0, 0.3333333f,  0, 0, 0.3333333f },
  // Rx = √3·(m2z + m3z − 2·m1z) / 3
  { 0, 0, -1.1547005f,  0, 0, 0.5773503f,  0, 0, 0.5773503f },
  // Ry = m3z − m2z
  { 0, 0, 0,  0, 0, -1.0f,  0, 0, 1.0f },
  // Rz = Σ(posXᵢ·magYᵢ − posYᵢ·magXᵢ)
  { 0.5773503f, 0, 0,  -0.2886751f, -0.5f, 0,  -0.2886751f, 0.5f, 0 },
};

// Gains and sign fixes
const float GAIN_T[3] = {28.0, 28.0, 24.0};
const float GAIN_R[3] = {18.0, 18.0, 20.0};
const int SIGN_AXIS[6] = {-1, +1, -1, +1, +1, +1};

// Cross-axis compensation matrix (6×6, identity = no compensation).
// Off-diagonal terms cancel residual bleed between axes.
const float COMP[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1},
};

// Per-axis dead zones
//                    Tx    Ty    Tz    Rx    Ry    Rz
const float DEADZONE[6] = {16.0, 16.0, 16.0, 20.0, 20.0, 20.0};

// Smoothing (single-pole fallback)
const float SMOOTH_TAU_S = 0.08;

// Biquad low-pass filter (comment out to use single-pole fallback).
#define FILTER_BIQUAD
const float FILTER_FREQ_HZ = 8.0f;    // cutoff frequency
const float FILTER_Q = 0.707f;        // Butterworth (maximally flat)

// Response curve: blend between linear and cubic.
// 1.0 = pure linear (current behaviour), 0.0 = pure cubic.
const float RESPONSE_LINEARITY = 1.0f;

// Final axis output range
const float AXIS_LIMIT = 350.0;

// RGB LEDs
// Define USE_ONBOARD_LED to use the single NeoPixel on the XIAO RP2040
// instead of the 8-LED ring on the sensor board.
#define USE_ONBOARD_LED

#ifdef USE_ONBOARD_LED
const int LED_COUNT = 1;
const int PIN_LED_ONBOARD_DATA = 12;    // XIAO RP2040 onboard NeoPixel
const int PIN_LED_ONBOARD_POWER = 11;   // XIAO RP2040 NeoPixel power enable
#else
const int LED_COUNT = 8;
#endif

const int LED_BRIGHTNESS = 40;
const unsigned long LED_IDLE_COLOR = 0x00FF00;
const unsigned long LED_CALIBRATING_COLOR = 0x0000FF;

// Sensor address retry
// Enable this if sensors may retain a previously assigned I2C address
// across power cycles (e.g., when PMOS doesn't fully cut power).
// When enabled, begin() will retry on the default address (A4) if the
// first attempt fails (the library's begin() resets the sensor address).
#define SENSOR_ADDR_RETRY

// FSM timing
const long IDLE_SLEEP_TIMEOUT_MS = 2 * 60 * 1000;

}  // namespace Config