#pragma once

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;

  struct BiquadState {
    float z1 = 0.0f;
    float z2 = 0.0f;
  };

 private:
  static float clampf(float v, float lo, float hi);
  static float hardZero(float v, float thr);
  static float lowpass(float prev, float x, float dt, float tau);
  static float biquadLP(BiquadState& s, float x, float dt);
  static void transform(const float delta[9], float axes[6]);
  static void compensate(const float in[6], float out[6]);
  static float responseCurve(float x);
  float filt_[6] = {};
  BiquadState bq_[6] = {};
  bool motionActive_ = false;
};
