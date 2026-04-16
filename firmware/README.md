You can customize several variables to tune gains, smoothing, and deadzones for all six axes.
Most of these settings are defined in [`Config.h`](include/Config.h) and are the main place to adjust the overall feel of the device.

```cpp
// Gains and sign fixes
const float GAIN_T[3] = {28.0, 28.0, 24.0};
const float GAIN_R[3] = {18.0, 18.0, 20.0};
const int SIGN_AXIS[6] = {-1, +1, -1, +1, +1, +1};

// Per-axis dead zones (Tx, Ty, Tz, Rx, Ry, Rz)
const float DEADZONE[6] = {16.0, 16.0, 16.0, 20.0, 20.0, 20.0};

// Response curve (1.0 = linear, 0.0 = pure cubic)
const float RESPONSE_LINEARITY = 1.0;

// Smoothing — single-pole fallback
const float SMOOTH_TAU_S = 0.08;

// Biquad low-pass filter (comment out #define to use single-pole)
#define FILTER_BIQUAD
const float FILTER_FREQ_HZ = 8.0;    // cutoff frequency
const float FILTER_Q = 0.707;        // Butterworth
```

⚠️ Refer to the video at [6:23](https://youtu.be/62xlzGs8LXA?si=ld2shDCaTxOLIGB8&t=383) for a demo of driver support. Related settings can be found commented in[`platformio.ini`](../platformio.ini).

### Motion Processing Pipeline

The [`MotionController`](src/controllers/MotionController.cpp) processes 9 raw sensor readings (3 axes × 3 sensors) into 6 output axes:

```
raw[9] → baseline subtract → matrix transform 6×9 → cross-axis compensation 6×6
       → gain × sign → response curve → biquad filter → per-axis deadzone → clamp → out[6]
```

**Sensor layout:**
- `mag1` = bottom
- `mag2` = top left
- `mag3` = top right

**Stage details:**

| Stage | Config | Purpose |
|-------|--------|---------|
| Transform 6×9 | `TRANSFORM[6][9]` | Maps 9 sensor deltas to 6 axes via matrix multiply. Default encodes the original triangle geometry formulas. |
| Compensation 6×6 | `COMP[6][6]` | Corrects residual cross-axis bleed after transform. Identity by default — set off-diagonal terms to cancel measured bleed. |
| Gain × sign | `GAIN_T[3]`, `GAIN_R[3]`, `SIGN_AXIS[6]` | Scales and inverts each axis to match the host coordinate system. |
| Response curve | `RESPONSE_LINEARITY` | Blends linear + cubic shaping. `1.0` = linear (default), `0.6` = good for CAD, `0.0` = aggressive cubic for fine center control. |
| Filter | `FILTER_FREQ_HZ`, `FILTER_Q` | Biquad low-pass (Direct Form II Transposed). Comment out `#define FILTER_BIQUAD` to fall back to single-pole exponential (`SMOOTH_TAU_S`). |
| Deadzone | `DEADZONE[6]` | Per-axis noise gate. Signal below the threshold is zeroed and filter state is reset. |
| Clamp | `AXIS_LIMIT` | Output clamped to ±350. |

### Tuning Guide

**Cross-axis compensation:** Enable telemetry, move one axis at a time, note unwanted output on other axes, then set `COMP` off-diagonal terms to cancel the leak.

**Response curve:** Lower `RESPONSE_LINEARITY` (e.g. 0.6) for smoother centre feel in CAD apps. Test in Fusion 360 / FreeCAD.

**Filter tuning:** Lower `FILTER_FREQ_HZ` for more smoothing (more latency). Raise `FILTER_Q` above 0.707 for sharper cutoff (but introduces overshoot).

See [`SPEC.md`](SPEC.md) for full technical details.
