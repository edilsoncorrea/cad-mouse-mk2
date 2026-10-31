#pragma once

#include <Arduino.h>

namespace Config {

const bool ENABLE_TELEMETRY = true;

// Diagnostic: also stream the 9 baseline-subtracted sensor deltas
// (m1x..m3z) alongside the processed axes. Comment out for normal use.
#define TELEMETRY_RAW

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

// Motion decomposition mode.
// By default the pipeline uses the configurable 6×9 TRANSFORM matrix below.
// Uncomment LEGACY_MOTION_FORMULAS to fall back to the original hard-coded
// trigonometric formulas (pre-refactor behaviour). Both paths are numerically
// equivalent by design; the toggle exists purely as a safety rollback so the
// original decomposition can be restored without a git revert.
#define LEGACY_MOTION_FORMULAS

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

// Global sensitivity multiplier, applied on top of GAIN_T/GAIN_R.
// DEADZONE is scaled by the same factor, so the force needed to start a
// movement (and the rest-noise rejection) stays the same; only the response
// above the dead zone gets stronger. Tune in small steps (e.g. 1.2 -> 1.4).
const float SENSITIVITY = 1.2f;

// Cross-axis compensation matrix (6×6, identity = no compensation).
// Off-diagonal terms cancel residual bleed between axes.
// Measured 2026-10-30 from per-axis push tests (TELEMETRY_RAW): inverse of
// the observed response matrix, keeping only couplings consistent in both
// directions. Main fix: X push leaked ~60% into Rx. Z row/column left as
// identity because the Z bleed was not linear (same sign both directions).
// NOTE: COMP is applied to the raw axes, BEFORE SIGN_AXIS and GAIN_T/GAIN_R,
// so coefficients are expressed in raw (pre-gain) units.
const float COMP[6][6] = {
  { 1.005f,  0.011f, 0.000f,  0.021f,  0.316f, -0.103f},
  { 0.054f,  1.163f, 0.000f,  0.318f,  0.037f, -0.009f},
  { 0.000f,  0.000f, 1.000f,  0.000f,  0.000f,  0.000f},
  {-0.842f,  0.549f, 0.000f,  1.070f,  0.026f,  0.041f},
  { 0.225f, -0.126f, 0.000f, -0.245f,  1.004f, -0.168f},
  {-0.204f, -0.002f, 0.000f, -0.004f, -0.064f,  1.021f},
};

// Second-order bleed correction, applied after sign/gain (output units):
//   y[i] -= BLEED_LIN[i][j] * y[j] + BLEED_ABS[i][j] * |y[j]|
// Cancels coupling that has the same sign in both directions (e.g. twisting
// Rz either way lifts the knob -> Z and Rx drop), which COMP cannot fix.
// Measured 2026-10-30 from per-axis push tests.
//                         X       Y  Z  Rx  Ry   Rz
// Ry column fitted from a dedicated left/right tilt test.
const float BLEED_LIN[6][6] = {
  {0,      0, 0, 0, -0.055f,  0},        // X  <- Ry
  {0,      0, 0, 0,  0.079f,  0},        // Y  <- Ry
  {0.014f, 0, 0, 0,  0.049f,  0.255f},   // Z  <- X, Ry, Rz
  {0,      0, 0, 0, -0.029f, -0.037f},   // Rx <- Ry, Rz
  {0,      0, 0, 0,  0,       0},
  {0,      0, 0, 0, -0.086f,  0},        // Rz <- Ry
};
const float BLEED_ABS[6][6] = {
  {0,       0, 0, 0, -0.074f,  0},       // X  <- |Ry|
  {0,       0, 0, 0, -0.078f,  0},       // Y  <- |Ry|
  {-0.245f, 0, 0, 0,  0.128f, -0.528f},  // Z  <- |X|, |Ry|, |Rz|
  {0,       0, 0, 0, -0.162f, -0.198f},  // Rx <- |Ry|, |Rz|
  {0,       0, 0, 0,  0,       0},
  {0,       0, 0, 0, -0.023f,  0},       // Rz <- |Ry|
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
const int LED_COUNT = 8;
const int LED_BRIGHTNESS = 40;
const unsigned long LED_IDLE_COLOR = 0x00FF00;
const unsigned long LED_CALIBRATING_COLOR = 0x0000FF;

// FSM timing
const long IDLE_SLEEP_TIMEOUT_MS = 2 * 60 * 1000;

}  // namespace Config
