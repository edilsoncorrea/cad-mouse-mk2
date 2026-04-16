# Motion Processing — Improvement Spec

**Status:** Implemented (Phases 1–5)  
**Date:** 2026-04-15  
**Scope:** `MotionController` pipeline improvements  
**Target:** Firmware on RP2040 (125 MHz ARM Cortex-M0+, 264 KB RAM)

---

## 1. Context

The current pipeline in `MotionController::compute()` is:

```
raw[9] → baseline subtract → linear decomposition → gain × sign → lowpass → deadzone → clamp → out[6]
```

Known limitations (documented by the original author):
- **Cross-axis bleed** — translation and rotation are computed independently; tilting the knob contaminates translation readings.
- **Linearity assumption** — magnetic field follows inverse-cube law, but the code treats sensor deltas as proportional to displacement.

This spec defines five improvements that preserve the existing architecture and public API.

---

## 2. Summary of Changes

| # | Feature | Where | Breaking? |
|---|---------|-------|-----------|
| 2.1 | Transformation matrix 6×9 | replaces hard-coded formulas | No |
| 2.2 | Cross-axis compensation | post-transform correction | No |
| 2.3 | Per-axis deadzone | `Config.h` | No (superset of current) |
| 2.4 | Non-linear response curve | post-gain shaping | No |
| 2.5 | Second-order (biquad) filter | replaces single-pole lowpass | No |

New pipeline:

```
raw[9] → baseline subtract → matrix transform 6×9 → cross-axis compensation → gain × sign → response curve → biquad filter → per-axis deadzone → clamp → out[6]
```

---

## 2.1 Transformation Matrix 6×9

### Problem

Translation and rotation are currently calculated with hard-coded formulas derived from an ideal equilateral triangle. This couples the decomposition to the assumed geometry and makes tuning individual sensor contributions impossible.

### Design

Replace the six formulas with a single 6×9 matrix multiply:

```cpp
// Config.h
const float TRANSFORM[6][9] = { ... };
```

Each row maps the 9 sensor deltas `[m1x, m1y, m1z, m2x, m2y, m2z, m3x, m3y, m3z]` to one output axis.

**Default matrix** (reproduces current behavior):

```
         m1x    m1y    m1z    m2x    m2y    m2z    m3x    m3y    m3z
Tx  [  1/3      0      0    1/3      0      0    1/3      0      0   ]
Ty  [    0    1/3      0      0    1/3      0      0    1/3      0   ]
Tz  [    0      0    1/3      0      0    1/3      0      0    1/3   ]
Rx  [    0      0   -2√3/3    0      0    √3/3     0      0    √3/3  ]
Ry  [    0      0      0      0      0     -1      0      0      1   ]
Rz  [    0   √3/3      0   -1/2  -√3/6     0    1/2  -√3/6     0   ]
```

> The Rz row encodes `Σ(posXᵢ·magYᵢ − posYᵢ·magXᵢ)` with the sensor triangle positions baked into the coefficients.

### Pseudocode

```cpp
void MotionController::transform(const float delta[9], float axes[6]) {
  for (int r = 0; r < 6; r++) {
    float sum = 0.0f;
    for (int c = 0; c < 9; c++) {
      sum += Config::TRANSFORM[r][c] * delta[c];
    }
    axes[r] = sum;
  }
}
```

**Cost:** 54 multiply-adds per cycle — trivial at 125 MHz.

### Calibration Workflow

Users can derive a custom matrix empirically:
1. Move one physical axis at a time (e.g., pure Tx).
2. Record the raw delta vector.
3. Build the matrix rows via least-squares fit or manual tuning.

---

## 2.2 Cross-Axis Compensation

### Problem

Even with the matrix, residual bleed exists because the magnetic field is not a pure linear function of displacement. Tilting produces a small translation signal that the matrix alone cannot fully cancel.

### Design

A 6×6 compensation matrix applied **after** the transform:

```cpp
// Config.h
const float COMP[6][6] = {
  // Identity by default — no compensation
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1},
};
```

Off-diagonal terms subtract the measured bleed. Example: if Tz leaks `α` per unit of Rx:

```
COMP[2][3] = -α
```

### Pseudocode

```cpp
void MotionController::compensate(const float in[6], float out[6]) {
  for (int r = 0; r < 6; r++) {
    float sum = 0.0f;
    for (int c = 0; c < 6; c++) {
      sum += Config::COMP[r][c] * in[c];
    }
    out[r] = sum;
  }
}
```

**Cost:** 36 multiply-adds per cycle.

### Tuning

1. Enable telemetry.
2. Move one axis at a time and note unwanted output on other axes.
3. Set off-diagonal coefficients to cancel the leak.
4. Iterate until bleed is within deadzone.

---

## 2.3 Per-Axis Deadzone

### Problem

Current deadzones are shared across translation (`DEAD_T`) and rotation (`DEAD_R`). Each axis has different noise characteristics and mechanical play.

### Design

```cpp
// Config.h  —  replaces DEAD_T and DEAD_R
const float DEADZONE[6] = {16.0, 16.0, 16.0, 20.0, 20.0, 20.0};
//                          Tx    Ty    Tz    Rx    Ry    Rz
```

### Changes to MotionController

Remove `axisBaseDead()`. Replace inline:

```cpp
const float dead = Config::DEADZONE[i];
```

The rest of the filtering logic stays the same.

---

## 2.4 Non-Linear Response Curve

### Problem

Linear gain makes the device equally sensitive at center and extremes. CAD workflows benefit from fine control near center and progressive acceleration toward edges. The 3Dconnexion SpaceMouse uses a similar curve internally.

### Design

Apply a tunable cubic response curve **after** gain, **before** filtering:

```cpp
// Config.h
const float RESPONSE_LINEARITY = 0.6f;  // 0.0 = pure cubic, 1.0 = pure linear
```

### Shaping Function

```cpp
float MotionController::responseCurve(float x, float linearity) {
  // Normalize to [-1, 1]
  const float norm = clampf(x / Config::AXIS_LIMIT, -1.0f, 1.0f);
  // Blend linear + cubic
  const float shaped = linearity * norm + (1.0f - linearity) * norm * norm * norm;
  return shaped * Config::AXIS_LIMIT;
}
```

Behavior:
- `linearity = 1.0` → current behavior (linear)
- `linearity = 0.6` → gentle curve, good default for CAD
- `linearity = 0.0` → aggressive cubic, very fine center control

### Per-Axis Linearity (optional extension)

```cpp
const float RESPONSE_LINEARITY[6] = {0.6, 0.6, 0.6, 0.6, 0.6, 0.6};
```

This allows translation and rotation to have different curves if needed.

---

## 2.5 Second-Order (Biquad) Low-Pass Filter

### Problem

The current single-pole exponential filter has a gentle -6 dB/octave rolloff. Sensor noise at higher frequencies leaks through, requiring a larger deadzone to compensate. A second-order filter provides -12 dB/octave rolloff — better noise rejection with less phase lag at the cutoff frequency.

### Design

Replace the per-axis single-pole filter with a biquad (Direct Form II Transposed):

```cpp
// Config.h
const float FILTER_FREQ_HZ = 8.0f;   // cutoff frequency
const float FILTER_Q = 0.707f;        // Butterworth (maximally flat)
```

### State

```cpp
// MotionController.h
struct BiquadState {
  float z1 = 0.0f;
  float z2 = 0.0f;
};

BiquadState bq_[6];
```

### Pseudocode

```cpp
float MotionController::biquadLP(BiquadState& s, float x, float dt) {
  const float w0 = 2.0f * M_PI * Config::FILTER_FREQ_HZ * dt;
  const float alpha = sinf(w0) / (2.0f * Config::FILTER_Q);
  const float cosw0 = cosf(w0);

  // Normalize coefficients
  const float a0 = 1.0f + alpha;
  const float b0 = ((1.0f - cosw0) / 2.0f) / a0;
  const float b1 = (1.0f - cosw0) / a0;
  const float b2 = b0;
  const float a1 = (-2.0f * cosw0) / a0;
  const float a2 = (1.0f - alpha) / a0;

  // Direct Form II Transposed
  const float y = b0 * x + s.z1;
  s.z1 = b1 * x - a1 * y + s.z2;
  s.z2 = b2 * x - a2 * y;
  return y;
}
```

> **Note:** Coefficients depend on `dt`, which varies slightly between samples. On the RP2040 at ~100 Hz loop rate, dt jitter is small enough that recomputing per-sample is acceptable. If profiling shows this is too expensive, precompute coefficients at a fixed rate and use `dt` only for the single-pole fallback.

### Fallback

Keep `SMOOTH_TAU_S` as a config option. A compile-time flag selects the filter:

```cpp
// Config.h
#define FILTER_BIQUAD  // comment out to use single-pole
```

---

## 3. Updated `MotionController` API

No public API changes. Internal additions:

```cpp
class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;

 private:
  // Existing
  static float clampf(float v, float lo, float hi);
  static float hardZero(float v, float thr);

  // New
  static void transform(const float delta[9], float axes[6]);
  static void compensate(const float in[6], float out[6]);
  static float responseCurve(float x, float linearity);

#ifdef FILTER_BIQUAD
  struct BiquadState { float z1 = 0.0f; float z2 = 0.0f; };
  BiquadState bq_[6];
  static float biquadLP(BiquadState& s, float x, float dt);
#else
  float filt_[6] = {};
  static float lowpass(float prev, float x, float dt, float tau);
#endif

  bool motionActive_ = false;
};
```

---

## 4. Updated `Config.h` Additions

```cpp
// --- Transformation matrix (6×9) ---
const float TRANSFORM[6][9] = { /* default: current formulas */ };

// --- Cross-axis compensation (6×6, identity default) ---
const float COMP[6][6] = { /* identity */ };

// --- Per-axis deadzones (replaces DEAD_T / DEAD_R) ---
const float DEADZONE[6] = {16.0, 16.0, 16.0, 20.0, 20.0, 20.0};

// --- Response curve ---
const float RESPONSE_LINEARITY = 0.6f;

// --- Filter ---
#define FILTER_BIQUAD
const float FILTER_FREQ_HZ = 8.0f;
const float FILTER_Q = 0.707f;
```

---

## 5. Constraints

- **No public API changes** — `compute()` signature unchanged.
- **Default behavior preserved** — with identity compensation, default matrix, and `linearity = 1.0`, output matches current firmware.
- **No dynamic allocation** — all state is fixed-size arrays.
- **No floating-point double** — all `float` (32-bit) for RP2040 performance.
- **Telemetry compatibility** — existing serial output format unchanged.

---

## 6. Testing Strategy

| Test | Method |
|------|--------|
| Default matrix matches current output | Feed recorded raw data through both old and new code; diff must be zero |
| Compensation identity = no change | Unit test: `compensate(in) == in` when `COMP` is identity |
| Response curve at linearity=1.0 | Must be identity within float epsilon |
| Biquad filter stability | Step response must settle without oscillation at Q=0.707 |
| Per-axis deadzone | Verify each axis uses its own threshold independently |
| Overflow safety | Inputs at ±max sensor range must not overflow intermediate floats |
| Real-device feel test | Subjective evaluation in Fusion 360 / FreeCAD with 3DxWare or spacenav |
