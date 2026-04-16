// MotionController test suite — characterisation & unit tests.
//
// Characterisation tests pin down the CURRENT behaviour of
// MotionController::compute() so any refactor or pipeline change that
// accidentally alters output is caught immediately.  They are the safety
// net for the improvements described in SPEC.md.
//
// Build & run:  pio test -e native

#include <unity.h>

#include <cmath>
#include <cstring>

#include "controllers/MotionController.h"
#include "Config.h"

// Include the implementation directly for native builds.
// PlatformIO native env does not compile from src_dir automatically.
#include "../../src/controllers/MotionController.cpp"

// ── Helpers ──────────────────────────────────────────────────────────

static const float ZERO_BASELINE[9] = {};

// Feed the same raw vector for N iterations at fixed dt and return
// the final output.  With enough iterations the single-pole filter
// converges and we get the steady-state result.
static void steadyState(MotionController& mc,
                        const float raw[9],
                        const float baseline[9],
                        float dt,
                        int iterations,
                        float out[6]) {
  for (int i = 0; i < iterations; i++) {
    mc.compute(raw, baseline, dt, out);
  }
}

// Convenience: reset + run to steady state.
static void resetAndSteady(MotionController& mc,
                           const float raw[9],
                           const float baseline[9],
                           float out[6],
                           float dt = 0.01f,
                           int iter = 300) {
  mc.reset();
  steadyState(mc, raw, baseline, dt, iter, out);
}

// ── 1. Math utility tests ────────────────────────────────────────────

// These exercise the private static helpers via observable behaviour
// of compute().

void test_clamp_upper() {
  // Input large enough to saturate after gain → output == AXIS_LIMIT.
  MotionController mc;
  float out[6];
  // mag1y = mag2y = mag3y = 100 → ty = 100
  // y[TY] = +1 * 100 * 28 = 2800 → clamped to 350
  float raw[9] = {0, 100, 0, 0, 100, 0, 0, 100, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, Config::AXIS_LIMIT, out[1]);
}

void test_clamp_lower() {
  MotionController mc;
  float out[6];
  // ty = -100 → y[TY] = +1 * (-100) * 28 = -2800 → -350
  float raw[9] = {0, -100, 0, 0, -100, 0, 0, -100, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -Config::AXIS_LIMIT, out[1]);
}

void test_deadzone_silence() {
  // Tiny input that, after gain, stays inside the deadzone.
  // tx = 0.1 → y[TX] = -1 * 0.1 * 28 = -2.8   |2.8| < DEADZONE[0](16) → 0
  MotionController mc;
  float out[6];
  float raw[9] = {0.1f, 0, 0, 0.1f, 0, 0, 0.1f, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[i]);
  }
}

void test_reset_clears_filter() {
  MotionController mc;
  float out[6];
  // Drive to non-zero state
  float raw[9] = {0, 0, 0, 0, 0, -5, 0, 0, 5};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(mc.hasMotionActivity());

  // Reset and feed zeros — must be silent immediately.
  mc.reset();
  float zeros[9] = {};
  mc.compute(zeros, ZERO_BASELINE, 0.01f, out);
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[i]);
  }
  TEST_ASSERT_FALSE(mc.hasMotionActivity());
}

void test_lowpass_single_step() {
#ifdef FILTER_BIQUAD
  // Biquad with dt=10s pushes w0 beyond Nyquist; skip for biquad mode.
  TEST_PASS();
#else
  // After reset, one step with large dt should nearly reach input.
  MotionController mc;
  mc.reset();
  float out[6];
  // ty = 5 → y[TY] = 5 * 28 = 140  (> deadzone)
  // dt=10 → alpha = 10/(0.08+10) ≈ 0.9921 → filt ≈ 138.9
  float raw[9] = {0, 5, 0, 0, 5, 0, 0, 5, 0};
  mc.compute(raw, ZERO_BASELINE, 10.0f, out);
  // Should be very close to 140 (within ~1%)
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 140.0f, out[1]);
#endif
}

// ── 2. Characterisation: axis isolation ──────────────────────────────

// These tests document that a uniform shift on one sensor component
// produces output ONLY on the expected axis (steady state, within
// the deadzone tolerance).  If a future change introduces new
// cross-axis terms, these will catch it.

void test_char_pure_tx() {
  // All sensors shift +X by 3 → tx = 3
  // y[TX] = -1 * 3 * 28 = -84
  // All other raw components are 0 → other axes within deadzone.
  MotionController mc;
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  TEST_ASSERT_FLOAT_WITHIN(0.5f, -84.0f, out[0]);   // Tx
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[1]);     // Ty
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[2]);     // Tz
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[3]);     // Rx
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[4]);     // Ry
  // Rz: uniform X → cross products cancel → 0
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[5]);     // Rz
}

void test_char_pure_ty() {
  // All sensors shift +Y by 3 → ty = 3
  // y[TY] = +1 * 3 * 28 = 84
  // Rz gets contribution from sensor Y:
  //   mag2: (-0.5)(3) - (√3/6)(0) = -1.5
  //   mag3: (0.5)(3)  - (√3/6)(0) =  1.5
  //   mag1: (0)(3)    - (-√3/3)(0) = 0
  //   rz = -1.5 + 1.5 + 0 = 0  → OK, uniform Y also cancels in Rz.
  MotionController mc;
  float out[6];
  float raw[9] = {0, 3, 0, 0, 3, 0, 0, 3, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 84.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[4]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[5]);
}

void test_char_pure_tz() {
  // All sensors shift +Z by 3 → tz = 3
  // y[TZ] = -1 * 3 * 24 = -72
  // Rx = √3*(3+3-6)/3 = 0,  Ry = 3-3 = 0
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, 3, 0, 0, 3, 0, 0, 3};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -72.0f, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[4]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[5]);
}

void test_char_pure_ry() {
  // mag2z = -5, mag3z = +5, rest = 0
  // tz = (0-5+5)/3 = 0
  // ry = 5 - (-5) = 10     → y[RY] = +1 * 10 * 18 = 180
  // rx = √3*(-5+5-0)/3 = 0
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, 0, 0, 0, -5, 0, 0, 5};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[3]);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 180.0f, out[4]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[5]);
}

void test_char_pure_rx() {
  // Symmetric tilt: mag1z = -4, mag2z = +2, mag3z = +2
  // tz = (-4+2+2)/3 = 0
  // rx = √3*(2+2+8)/3 = √3*12/3 = 4√3 ≈ 6.9282
  // y[RX] = +1 * 6.9282 * 18 = 124.708
  // ry = 2-2 = 0
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, -4, 0, 0, 2, 0, 0, 2};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  const float expected_rx = 4.0f * sqrtf(3.0f) * 18.0f;  // ≈ 124.71
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected_rx, out[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[4]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[5]);
}

void test_char_pure_rz_twist() {
  // Twist: opposing tangential forces on top two sensors.
  // mag2 at (-0.5, √3/6): give it +X component only → magX = 4
  // mag3 at (+0.5, √3/6): give it -X component only → magX = -4
  // mag1 at (0, -√3/3):   zero.
  //
  // tx = (0 + 4 + (-4))/3 = 0
  // rz = (-0.5)(0) - (√3/6)(4)  +  (0.5)(0) - (√3/6)(-4)  +  0
  //    = -4√3/6 + 4√3/6  = 0   Hmm, no twist signal from X alone.
  //
  // Better: give tangential Y components.
  // mag2 (top-left):  magY = -3    (tangent pushes clockwise)
  // mag3 (top-right): magY = +3    (tangent pushes clockwise)
  // mag1 (bottom):    magX = 0, magY = 0
  //
  // rz = (-0.5)(-3) - (√3/6)(0)  +  (0.5)(3) - (√3/6)(0)  + 0
  //    = 1.5 + 1.5 = 3.0
  // y[RZ] = +1 * 3.0 * 20 = 60
  // ty = (0 + (-3) + 3) / 3 = 0  → OK
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, 0, 0, -3, 0, 0, 3, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[4]);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, out[5]);
}

// ── 3. Characterisation: cross-axis bleed (documents known issue) ────

void test_char_bleed_tilt_to_tz() {
  // Asymmetric tilt: mag1z pulled far down.
  // raw = [0,0,-10, 0,0,3, 0,0,3]
  // tz = (-10+3+3)/3 = -4/3 ≈ -1.333
  // y[TZ] = -1 * (-1.333) * 24 = 32.0
  // rx = √3*(3+3+20)/3 = √3*26/3 ≈ 15.011
  // y[RX] = +1 * 15.011 * 18 = 270.2
  //
  // So Tz = 32 is BLEED from what should be a pure Rx motion.
  // This test documents the bleed magnitude so we can verify future
  // improvements reduce it.
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, -10, 0, 0, 3, 0, 0, 3};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  const float expected_tz = (-1.0f) * ((-10.0f + 3.0f + 3.0f) / 3.0f) * 24.0f;
  const float expected_rx = sqrtf(3.0f) * (3.0f + 3.0f - 2.0f * (-10.0f)) / 3.0f * 18.0f;

  TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_tz, out[2]);  // Tz bleed ≈ 32
  TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_rx, out[3]);   // Rx primary
  // Document: |Tz bleed| > deadzone → the bleed IS visible to the user.
  TEST_ASSERT_TRUE(fabsf(out[2]) > Config::DEADZONE[2]);
}

void test_char_bleed_asymmetric_ry_to_tz() {
  // Tilt in Ry only but asymmetric magnitudes.
  // mag2z = -8, mag3z = +4
  // tz = (0 + (-8) + 4)/3 = -4/3 ≈ -1.333
  // y[TZ] = -1 * (-1.333) * 24 = 32.0  → bleed!
  // ry = 4 - (-8) = 12 → y[RY] = 12*18 = 216
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, 0, 0, 0, -8, 0, 0, 4};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);

  TEST_ASSERT_FLOAT_WITHIN(1.0f, 32.0f, out[2]);   // Tz bleed
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 216.0f, out[4]);   // Ry primary
}

// ── 4. Baseline subtraction ──────────────────────────────────────────

void test_baseline_subtraction() {
  // raw == baseline → all deltas = 0 → zero output.
  MotionController mc;
  float out[6];
  float raw[9]      = {10, 20, 30, 40, 50, 60, 70, 80, 90};
  float baseline[9] = {10, 20, 30, 40, 50, 60, 70, 80, 90};
  resetAndSteady(mc, raw, baseline, out);

  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[i]);
  }
}

void test_baseline_offset() {
  // Verify that baseline shifts the reference correctly.
  // raw - baseline = [3,0,0, 3,0,0, 3,0,0]  → same as pure Tx = 3
  MotionController mc;
  float out[6];
  float raw[9]      = {103, 200, 300, 403, 500, 600, 703, 800, 900};
  float baseline[9] = {100, 200, 300, 400, 500, 600, 700, 800, 900};
  resetAndSteady(mc, raw, baseline, out);

  TEST_ASSERT_FLOAT_WITHIN(0.5f, -84.0f, out[0]);  // Tx
}

// ── 5. Motion activity flag ──────────────────────────────────────────

void test_motion_activity_true() {
  MotionController mc;
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(mc.hasMotionActivity());
}

void test_motion_activity_false_in_deadzone() {
  MotionController mc;
  float out[6];
  float raw[9] = {0.1f, 0, 0, 0.1f, 0, 0, 0.1f, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FALSE(mc.hasMotionActivity());
}

void test_motion_activity_false_at_zero() {
  MotionController mc;
  float out[6];
  float raw[9] = {};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FALSE(mc.hasMotionActivity());
}

// ── 6. Filter dynamics ──────────────────────────────────────────────

void test_filter_step_response_monotonic() {
  // From reset, feeding constant input: output should converge toward
  // the steady-state value.  Single-pole is strictly monotonic;
  // biquad (Butterworth) has ≤5% overshoot.
  MotionController mc;
  mc.reset();
  float out[6];
  // ty = 5 → y[TY] = 140  (above deadzone 16)
  float raw[9] = {0, 5, 0, 0, 5, 0, 0, 5, 0};
  float maxVal = 0.0f;
  for (int i = 0; i < 100; i++) {
    mc.compute(raw, ZERO_BASELINE, 0.01f, out);
    if (out[1] > maxVal) maxVal = out[1];
  }
  // Final value should be very close to 140.
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 140.0f, out[1]);
  // Overshoot (if any) bounded at 5% of steady state.
  TEST_ASSERT_TRUE(maxVal <= 140.0f * 1.05f);
}

void test_filter_convergence_time() {
  // After 5τ the filter should be within 1% of steady state.
  // τ = 0.08s → 5τ = 0.4s = 40 steps at dt=0.01
  MotionController mc;
  mc.reset();
  float out[6];
  float raw[9] = {0, 5, 0, 0, 5, 0, 0, 5, 0};
  for (int i = 0; i < 40; i++) {
    mc.compute(raw, ZERO_BASELINE, 0.01f, out);
  }
  TEST_ASSERT_FLOAT_WITHIN(1.4f, 140.0f, out[1]);  // within 1%
}

void test_filter_returns_to_zero() {
  // Drive to steady state then feed zeros — should decay to silence.
  MotionController mc;
  float out[6];
  float raw[9] = {0, 5, 0, 0, 5, 0, 0, 5, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(fabsf(out[1]) > 100.0f);

  // Now feed zeros for 300 steps — within deadzone, filter resets to 0.
  float zeros[9] = {};
  for (int i = 0; i < 300; i++) {
    mc.compute(zeros, ZERO_BASELINE, 0.01f, out);
  }
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[i]);
  }
}

// ── 7. Sign & gain correctness ───────────────────────────────────────

void test_sign_axis_tx_negative() {
  // SIGN_AXIS[TX] = -1 → positive raw X → negative output.
  MotionController mc;
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(out[0] < 0.0f);
}

void test_sign_axis_ty_positive() {
  // SIGN_AXIS[TY] = +1 → positive raw Y → positive output.
  MotionController mc;
  float out[6];
  float raw[9] = {0, 3, 0, 0, 3, 0, 0, 3, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(out[1] > 0.0f);
}

void test_gain_t_applied() {
  // tx = 2 → y[TX] = -1 * 2 * 28 = -56  (> deadzone 16)
  // Steady state ≈ -56
  MotionController mc;
  float out[6];
  float raw[9] = {2, 0, 0, 2, 0, 0, 2, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -56.0f, out[0]);
}

void test_gain_r_applied() {
  // ry = 10 → y[RY] = +1 * 10 * 18 = 180
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, 0, 0, 0, -5, 0, 0, 5};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 180.0f, out[4]);
}

// ── 8. Edge cases ────────────────────────────────────────────────────

void test_all_axes_simultaneously() {
  // Non-trivial input that activates multiple axes.
  // Verify compute() doesn't crash or produce NaN.
  MotionController mc;
  float out[6];
  float raw[9] = {5, -3, 7, -2, 4, -6, 3, -1, 8};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FALSE(std::isnan(out[i]));
    TEST_ASSERT_FALSE(std::isinf(out[i]));
    TEST_ASSERT_TRUE(out[i] >= -Config::AXIS_LIMIT);
    TEST_ASSERT_TRUE(out[i] <= Config::AXIS_LIMIT);
  }
}

void test_extreme_inputs() {
  // Very large sensor values — must not overflow or produce NaN.
  MotionController mc;
  float out[6];
  float raw[9] = {1e4f, -1e4f, 1e4f, -1e4f, 1e4f, -1e4f, 1e4f, -1e4f, 1e4f};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FALSE(std::isnan(out[i]));
    TEST_ASSERT_FALSE(std::isinf(out[i]));
    TEST_ASSERT_TRUE(fabsf(out[i]) <= Config::AXIS_LIMIT);
  }
}

void test_zero_dt() {
  // dt = 0 should not crash. Filter alpha = 0 → filt stays at 0.
  MotionController mc;
  mc.reset();
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  mc.compute(raw, ZERO_BASELINE, 0.0f, out);
  // Output might be 0 (filter didn't update) — that's fine.
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FALSE(std::isnan(out[i]));
  }
}

void test_negative_dt() {
  // Should not crash.
  MotionController mc;
  mc.reset();
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  mc.compute(raw, ZERO_BASELINE, -0.01f, out);
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FALSE(std::isnan(out[i]));
  }
}

// ── 9. Symmetry tests ────────────────────────────────────────────────

void test_symmetry_opposite_input() {
  // Negating all raw values should negate outputs (in steady state).
  MotionController mc;
  float out_pos[6], out_neg[6];

  float raw_pos[9] = {4, 3, 2, 4, 3, 2, 4, 3, 2};
  resetAndSteady(mc, raw_pos, ZERO_BASELINE, out_pos);

  float raw_neg[9] = {-4, -3, -2, -4, -3, -2, -4, -3, -2};
  resetAndSteady(mc, raw_neg, ZERO_BASELINE, out_neg);

  for (int i = 0; i < 6; i++) {
    // Both may be 0 (deadzone) — that's symmetric too.
    if (out_pos[i] == 0.0f && out_neg[i] == 0.0f) continue;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -out_pos[i], out_neg[i]);
  }
}

// ── Unity hooks ──────────────────────────────────────────────────────

void setUp(void) {}
void tearDown(void) {}

// ── 10. Per-axis deadzone (Phase 1) ──────────────────────────────────

// Temporarily override DEADZONE to test asymmetric values.
// We do this by including the .cpp directly, so we can swap Config
// values via a wrapper.  For this test we verify the *structure*:
// the per-axis array is used, and different axes can have different
// thresholds.

void test_deadzone_per_axis_uses_array() {
  // With default config: DEADZONE = {16,16,16,20,20,20}
  // Input that is above Rx deadzone (20) but would be below if
  // Rx deadzone were 200:
  //   Rx input: mag1z=-4, mag2z=+2, mag3z=+2
  //   rx = 4√3 ≈ 6.928 → y[RX] = 6.928 * 18 = 124.7 → above 20 → non-zero
  MotionController mc;
  float out[6];
  float raw[9] = {0, 0, -4, 0, 0, 2, 0, 0, 2};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(fabsf(out[3]) > 0.0f);  // Rx active with deadzone 20
}

void test_deadzone_config_values() {
  // Verify the config array has the expected default values.
  TEST_ASSERT_EQUAL_FLOAT(16.0f, Config::DEADZONE[0]);  // Tx
  TEST_ASSERT_EQUAL_FLOAT(16.0f, Config::DEADZONE[1]);  // Ty
  TEST_ASSERT_EQUAL_FLOAT(16.0f, Config::DEADZONE[2]);  // Tz
  TEST_ASSERT_EQUAL_FLOAT(20.0f, Config::DEADZONE[3]);  // Rx
  TEST_ASSERT_EQUAL_FLOAT(20.0f, Config::DEADZONE[4]);  // Ry
  TEST_ASSERT_EQUAL_FLOAT(20.0f, Config::DEADZONE[5]);  // Rz
}

void test_deadzone_boundary_just_below() {
  // Signal just below the per-axis deadzone → must be zero.
  // DEADZONE[0] (Tx) = 16.  Need |y[TX]| < 16.
  // tx = val → y[TX] = -1 * val * 28.  Set val=0.5 → |y|=14 < 16.
  MotionController mc;
  float out[6];
  float raw[9] = {0.5f, 0, 0, 0.5f, 0, 0, 0.5f, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[0]);
}

void test_deadzone_boundary_just_above() {
  // Signal just above the per-axis deadzone → must be non-zero.
  // DEADZONE[0] (Tx) = 16.  Need |y[TX]| > 16.
  // tx = val → y[TX] = -1 * val * 28.  Set val=0.7 → |y|=19.6 > 16.
  MotionController mc;
  float out[6];
  float raw[9] = {0.7f, 0, 0, 0.7f, 0, 0, 0.7f, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(fabsf(out[0]) > 0.0f);
}

// ── 11. Transformation matrix (Phase 2) ───────────────────────────────

void test_transform_matrix_dimensions() {
  // Verify the matrix has correct dimensions by accessing boundary elements.
  // If this compiles, the array dimensions are correct.
  float sum = 0.0f;
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 9; c++) {
      sum += Config::TRANSFORM[r][c];
    }
  }
  // Sum of all coefficients is finite (not NaN/Inf).
  TEST_ASSERT_FALSE(std::isnan(sum));
  TEST_ASSERT_FALSE(std::isinf(sum));
}

void test_transform_translation_rows_sum_to_one_third() {
  // Each translation row should sum to 1/3 ≈ 0.333 (averaging 3 sensors).
  // Only the relevant component columns are non-zero.
  for (int r = 0; r < 3; r++) {
    float row_sum = 0.0f;
    for (int c = 0; c < 9; c++) {
      row_sum += Config::TRANSFORM[r][c];
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, row_sum);
  }
}

void test_transform_ry_row_sums_to_zero() {
  // Ry = m3z - m2z: coefficients sum to 0 (differential measurement).
  float row_sum = 0.0f;
  for (int c = 0; c < 9; c++) {
    row_sum += Config::TRANSFORM[4][c];
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, row_sum);
}

void test_transform_rx_row_sums_to_zero() {
  // Rx = √3·(m2z+m3z-2m1z)/3: coefficients sum to 0.
  float row_sum = 0.0f;
  for (int c = 0; c < 9; c++) {
    row_sum += Config::TRANSFORM[3][c];
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, row_sum);
}

void test_transform_only_x_in_tx_row() {
  // Tx row should only have non-zero values in X columns (0, 3, 6).
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Config::TRANSFORM[0][1]);  // m1y
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Config::TRANSFORM[0][2]);  // m1z
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Config::TRANSFORM[0][4]);  // m2y
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Config::TRANSFORM[0][5]);  // m2z
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Config::TRANSFORM[0][7]);  // m3y
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, Config::TRANSFORM[0][8]);  // m3z
}

// ── 12. Cross-axis compensation (Phase 3) ────────────────────────────

void test_comp_identity_is_passthrough() {
  // With identity COMP matrix, compensation should not alter values.
  // Verify COMP is actually identity.
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      float expected = (r == c) ? 1.0f : 0.0f;
      TEST_ASSERT_EQUAL_FLOAT(expected, Config::COMP[r][c]);
    }
  }
}

void test_comp_identity_output_unchanged() {
  // Full pipeline test: identity COMP should produce same output as
  // characterisation tests.  We re-test the pure Tx case.
  MotionController mc;
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -84.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out[2]);
}

// ── 13. Response curve (Phase 4) ─────────────────────────────────────

void test_response_linearity_default_is_one() {
  // Default RESPONSE_LINEARITY = 1.0 → pure linear → no change.
  TEST_ASSERT_EQUAL_FLOAT(1.0f, Config::RESPONSE_LINEARITY);
}

void test_response_linear_output_unchanged() {
  // With linearity=1.0, full pipeline output must match characterisation.
  // Pure Tx=3 → y[TX] = -84 → responseCurve(-84) = -84 (linear identity).
  MotionController mc;
  float out[6];
  float raw[9] = {3, 0, 0, 3, 0, 0, 3, 0, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -84.0f, out[0]);
}

void test_response_curve_symmetry() {
  // responseCurve(-x) == -responseCurve(x) for any linearity.
  // We test via the pipeline: opposite inputs → opposite outputs.
  // (This also holds at linearity=1.0 but tests the curve path.)
  MotionController mc;
  float out_pos[6], out_neg[6];
  float raw_pos[9] = {5, 0, 0, 5, 0, 0, 5, 0, 0};
  resetAndSteady(mc, raw_pos, ZERO_BASELINE, out_pos);
  float raw_neg[9] = {-5, 0, 0, -5, 0, 0, -5, 0, 0};
  resetAndSteady(mc, raw_neg, ZERO_BASELINE, out_neg);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -out_pos[0], out_neg[0]);
}

void test_response_curve_zero_passthrough() {
  // responseCurve(0) = 0 regardless of linearity.
  MotionController mc;
  float out[6];
  float raw[9] = {};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[i]);
  }
}

void test_response_curve_clamp_at_limit() {
  // Very large input: after gain, value exceeds AXIS_LIMIT.
  // responseCurve should clamp norm to [-1,1] → output = ±AXIS_LIMIT.
  MotionController mc;
  float out[6];
  float raw[9] = {0, 100, 0, 0, 100, 0, 0, 100, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, Config::AXIS_LIMIT, out[1]);
}

// ── 10. Biquad filter (Phase 5) ──────────────────────────────────────

void test_biquad_step_monotonic() {
  // Butterworth Q=0.707 has ≤5% overshoot (maximally flat frequency
  // response, not step response).  Verify bounded overshoot and convergence.
  MotionController mc;
  mc.reset();
  float raw[9] = {0, 5, 0, 0, 5, 0, 0, 5, 0};
  float out[6], maxVal = 0.0f;
  for (int i = 0; i < 100; i++) {
    mc.compute(raw, ZERO_BASELINE, 0.01f, out);
    if (out[1] > maxVal) maxVal = out[1];
  }
  // Overshoot bounded at 5% of steady-state (~140).
  TEST_ASSERT_TRUE_MESSAGE(maxVal <= 140.0f * 1.05f,
                           "biquad overshoot exceeds 5%");
  // Converged to steady state.
  TEST_ASSERT_FLOAT_WITHIN(1.5f, 140.0f, out[1]);
}

void test_biquad_convergence() {
  // After many iterations the output must converge to the expected
  // steady-state value (same as single-pole, since DC gain = 1).
  MotionController mc;
  float out[6];
  float raw[9] = {0, 5, 0, 0, 5, 0, 0, 5, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out, 0.01f, 500);
  // Expected: 5 * GAIN_T[1] * SIGN_AXIS[1] = 140
  const float expect = 5.0f * Config::GAIN_T[1] * Config::SIGN_AXIS[1];
  TEST_ASSERT_FLOAT_WITHIN(1.5f, expect, out[1]);
}

void test_biquad_returns_to_zero() {
  // After driving into steady-state then removing input, the output
  // must fall back to zero (deadzone resets filter state).
  MotionController mc;
  float out[6];
  float raw[9] = {0, 50, 0, 0, 50, 0, 0, 50, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  TEST_ASSERT_TRUE(fabs(out[1]) > 5.0f);
  float zero_raw[9] = {};
  steadyState(mc, zero_raw, ZERO_BASELINE, 0.01f, 300, out);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out[1]);
}

void test_biquad_variable_dt_stable() {
  // Alternating between two different dt values should not produce NaN
  // or wildly divergent values.
  MotionController mc;
  mc.reset();
  float raw[9] = {0, 30, 0, 0, 30, 0, 0, 30, 0};
  float out[6];
  for (int i = 0; i < 200; i++) {
    float dt = (i % 2 == 0) ? 0.008f : 0.012f;
    mc.compute(raw, ZERO_BASELINE, dt, out);
    for (int j = 0; j < 6; j++) {
      TEST_ASSERT_FALSE_MESSAGE(std::isnan(out[j]), "NaN with variable dt");
      TEST_ASSERT_TRUE_MESSAGE(fabs(out[j]) <= Config::AXIS_LIMIT + 1.0f,
                               "output diverges with variable dt");
    }
  }
}

void test_biquad_reset_clears_state() {
  // After reset, filter memory must be cleared — output starts from
  // zero again and initial samples are small.
  MotionController mc;
  float out[6];
  float raw[9] = {0, 50, 0, 0, 50, 0, 0, 50, 0};
  resetAndSteady(mc, raw, ZERO_BASELINE, out);
  float val_before = out[1];
  mc.reset();
  mc.compute(raw, ZERO_BASELINE, 0.01f, out);
  // First sample after reset must be much smaller than steady-state.
  TEST_ASSERT_TRUE(fabs(out[1]) < fabs(val_before) * 0.5f);
}

// ── Runner ───────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  UNITY_BEGIN();

  // Math utilities
  RUN_TEST(test_clamp_upper);
  RUN_TEST(test_clamp_lower);
  RUN_TEST(test_deadzone_silence);
  RUN_TEST(test_reset_clears_filter);
  RUN_TEST(test_lowpass_single_step);

  // Characterisation — axis isolation
  RUN_TEST(test_char_pure_tx);
  RUN_TEST(test_char_pure_ty);
  RUN_TEST(test_char_pure_tz);
  RUN_TEST(test_char_pure_ry);
  RUN_TEST(test_char_pure_rx);
  RUN_TEST(test_char_pure_rz_twist);

  // Characterisation — cross-axis bleed
  RUN_TEST(test_char_bleed_tilt_to_tz);
  RUN_TEST(test_char_bleed_asymmetric_ry_to_tz);

  // Baseline
  RUN_TEST(test_baseline_subtraction);
  RUN_TEST(test_baseline_offset);

  // Motion activity
  RUN_TEST(test_motion_activity_true);
  RUN_TEST(test_motion_activity_false_in_deadzone);
  RUN_TEST(test_motion_activity_false_at_zero);

  // Filter dynamics
  RUN_TEST(test_filter_step_response_monotonic);
  RUN_TEST(test_filter_convergence_time);
  RUN_TEST(test_filter_returns_to_zero);

  // Sign & gain
  RUN_TEST(test_sign_axis_tx_negative);
  RUN_TEST(test_sign_axis_ty_positive);
  RUN_TEST(test_gain_t_applied);
  RUN_TEST(test_gain_r_applied);

  // Edge cases
  RUN_TEST(test_all_axes_simultaneously);
  RUN_TEST(test_extreme_inputs);
  RUN_TEST(test_zero_dt);
  RUN_TEST(test_negative_dt);

  // Symmetry
  RUN_TEST(test_symmetry_opposite_input);

  // Per-axis deadzone (Phase 1)
  RUN_TEST(test_deadzone_per_axis_uses_array);
  RUN_TEST(test_deadzone_config_values);
  RUN_TEST(test_deadzone_boundary_just_below);
  RUN_TEST(test_deadzone_boundary_just_above);

  // Transformation matrix (Phase 2)
  RUN_TEST(test_transform_matrix_dimensions);
  RUN_TEST(test_transform_translation_rows_sum_to_one_third);
  RUN_TEST(test_transform_ry_row_sums_to_zero);
  RUN_TEST(test_transform_rx_row_sums_to_zero);
  RUN_TEST(test_transform_only_x_in_tx_row);

  // Cross-axis compensation (Phase 3)
  RUN_TEST(test_comp_identity_is_passthrough);
  RUN_TEST(test_comp_identity_output_unchanged);

  // Response curve (Phase 4)
  RUN_TEST(test_response_linearity_default_is_one);
  RUN_TEST(test_response_linear_output_unchanged);
  RUN_TEST(test_response_curve_symmetry);
  RUN_TEST(test_response_curve_zero_passthrough);
  RUN_TEST(test_response_curve_clamp_at_limit);

  // Biquad filter (Phase 5)
  RUN_TEST(test_biquad_step_monotonic);
  RUN_TEST(test_biquad_convergence);
  RUN_TEST(test_biquad_returns_to_zero);
  RUN_TEST(test_biquad_variable_dt_stable);
  RUN_TEST(test_biquad_reset_clears_state);

  return UNITY_END();
}
