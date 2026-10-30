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

float MotionController::deadzone(float v, float thr) {
  // Continuous dead zone: 0 inside, then ramps from 0 so there is no jump at
  // the threshold, and is rescaled so ±AXIS_LIMIT is still reachable.
  const float mag = fabsf(v);
  if (mag <= thr) return 0.0f;
  const float span = Config::AXIS_LIMIT - thr;
  if (span <= 0.0f) return 0.0f;
  const float scaled = (mag - thr) * Config::AXIS_LIMIT / span;
  return (v < 0.0f) ? -scaled : scaled;
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

  // Matrix transform: 9 sensor deltas → 6 raw axes.
  float axes[6];
  transform(delta, axes);

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

  // Filter, clamp to range, then dead zone.
  // The filter always runs, so its state is never reset mid-motion. Resetting
  // it whenever the input dipped under the dead zone made small or noisy
  // movements stutter or vanish entirely.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
#ifdef FILTER_BIQUAD
    filt_[i] = biquadLP(bq_[i], y[i], dt);
#else
    filt_[i] = lowpass(filt_[i], y[i], dt, Config::SMOOTH_TAU_S);
#endif

    const float limited =
        clampf(filt_[i], -Config::AXIS_LIMIT, Config::AXIS_LIMIT);
    out[i] = deadzone(limited, Config::DEADZONE[i]);
    if (out[i] != 0.0f) {
      motionActive_ = true;
    }
  }
}

bool MotionController::hasMotionActivity() const { return motionActive_; }
