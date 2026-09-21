#include "controllers/MotionController.h"

#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Config.h"

void MotionController::reset() {
  for (int i = 0; i < 6; i++) {
    filt_[i] = 0.0;
    bq_[i] = {};
  }
  motionActive_ = false;
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::hardZero(float v, float thr) {
  return (fabs(v) < thr) ? 0.0 : v;
}

float MotionController::lowpass(float prev, float x, float dt, float tau) {
  if (tau <= 0.0) return x;
  const float a = dt / (tau + dt);
  return prev + a * (x - prev);
}

float MotionController::biquadLP(BiquadState& s, float x, float dt) {
  if (dt <= 0.0f) return x;
  const float w0 = 2.0f * (float)M_PI * Config::FILTER_FREQ_HZ * dt;
  const float sinw0 = sinf(w0);
  const float cosw0 = cosf(w0);
  const float alpha = sinw0 / (2.0f * Config::FILTER_Q);

  const float a0 = 1.0f + alpha;
  const float b0 = ((1.0f - cosw0) / 2.0f) / a0;
  const float b1 = (1.0f - cosw0) / a0;
  const float b2 = b0;
  const float a1 = (-2.0f * cosw0) / a0;
  const float a2 = (1.0f - alpha) / a0;

  const float y = b0 * x + s.z1;
  s.z1 = b1 * x - a1 * y + s.z2;
  s.z2 = b2 * x - a2 * y;
  return y;
}

void MotionController::transform(const float delta[9], float axes[6]) {
  for (int r = 0; r < 6; r++) {
    float sum = 0.0f;
    for (int c = 0; c < 9; c++) {
      sum += Config::TRANSFORM[r][c] * delta[c];
    }
    axes[r] = sum;
  }
}

#ifdef LEGACY_MOTION_FORMULAS
// Original pre-refactor decomposition, kept as a compile-time rollback.
// Produces the same six raw axes [Tx, Ty, Tz, Rx, Ry, Rz] the TRANSFORM
// matrix encodes, from the baseline-subtracted sensor deltas.
void MotionController::legacyTransform(const float delta[9], float axes[6]) {
  const float mag1x = delta[0], mag1y = delta[1], mag1z = delta[2];
  const float mag2x = delta[3], mag2y = delta[4], mag2z = delta[5];
  const float mag3x = delta[6], mag3y = delta[7], mag3z = delta[8];

  // Translation: average of each component across the three sensors.
  const float tx = (mag1x + mag2x + mag3x) / 3.0f;
  const float ty = (mag1y + mag2y + mag3y) / 3.0f;
  const float tz = (mag1z + mag2z + mag3z) / 3.0f;

  // Sensor positions in the triangle (normalized), MAG1 bottom, MAG2/3 top.
  const float mag1PosX = 0.0f;
  const float mag1PosY = -0.5773503f;  // -√3/3
  const float mag2PosX = -0.5f;
  const float mag2PosY = 0.2886751f;   // √3/6
  const float mag3PosX = 0.5f;
  const float mag3PosY = 0.2886751f;

  // Rotation:
  //   Rx = √3·(mag2z + mag3z − 2·mag1z) / 3   (front/back tilt)
  //   Ry = mag3z − mag2z                       (side-to-side tilt)
  //   Rz = Σ(posXᵢ·magYᵢ − posYᵢ·magXᵢ)       (twist about vertical axis)
  const float rx = (1.7320508f * (mag2z + mag3z - 2.0f * mag1z)) / 3.0f;
  const float ry = (mag3z - mag2z);
  const float rz = (mag1PosX * mag1y - mag1PosY * mag1x) +
                   (mag2PosX * mag2y - mag2PosY * mag2x) +
                   (mag3PosX * mag3y - mag3PosY * mag3x);

  axes[0] = tx;
  axes[1] = ty;
  axes[2] = tz;
  axes[3] = rx;
  axes[4] = ry;
  axes[5] = rz;
}
#endif

void MotionController::compensate(const float in[6], float out[6]) {
  for (int r = 0; r < 6; r++) {
    float sum = 0.0f;
    for (int c = 0; c < 6; c++) {
      sum += Config::COMP[r][c] * in[c];
    }
    out[r] = sum;
  }
}

float MotionController::responseCurve(float x) {
  const float norm = clampf(x / Config::AXIS_LIMIT, -1.0f, 1.0f);
  const float lin = Config::RESPONSE_LINEARITY;
  const float shaped = lin * norm + (1.0f - lin) * norm * norm * norm;
  return shaped * Config::AXIS_LIMIT;
}

void MotionController::compute(const float raw[9], const float* baseline, float dt,
                               float out[6]) {
  // Baseline subtraction: convert to deltas around the calibrated rest pose.
  float delta[9];
  for (int i = 0; i < 9; i++) {
    delta[i] = raw[i] - baseline[i];
  }

  // Sensor deltas → 6 raw axes.
  float axes[6];
#ifdef LEGACY_MOTION_FORMULAS
  // Rollback path: original hard-coded trigonometric decomposition.
  // Sensor triangle (all sensors share the same package orientation):
  //   MAG1 = bottom, MAG2 = top left, MAG3 = top right.
  legacyTransform(delta, axes);
#else
  // Matrix transform: 9 sensor deltas → 6 raw axes.
  transform(delta, axes);
#endif

  // Cross-axis compensation.
  float comp[6];
  compensate(axes, comp);

  // Apply sign fixes and gains.
  float y[6];
  for (int i = 0; i < 3; i++) {
    y[i]     = Config::SIGN_AXIS[i]     * comp[i]     * Config::GAIN_T[i];
    y[i + 3] = Config::SIGN_AXIS[i + 3] * comp[i + 3] * Config::GAIN_R[i];
  }

  // Non-linear response curve.
  for (int i = 0; i < 6; i++) {
    y[i] = responseCurve(y[i]);
  }

  // Filter, clamp to range and dead zones.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = Config::DEADZONE[i];

    if (fabs(y[i]) < dead) {
      filt_[i] = 0.0;
      bq_[i] = {};
    } else {
#ifdef FILTER_BIQUAD
      filt_[i] = biquadLP(bq_[i], y[i], dt);
#else
      filt_[i] = lowpass(filt_[i], y[i], dt, Config::SMOOTH_TAU_S);
#endif
    }

    const float limited =
        clampf(filt_[i], -Config::AXIS_LIMIT, Config::AXIS_LIMIT);
    out[i] = hardZero(limited, dead);
    if (out[i] != 0.0) {
      motionActive_ = true;
    }
  }
}

bool MotionController::hasMotionActivity() const { return motionActive_; }
