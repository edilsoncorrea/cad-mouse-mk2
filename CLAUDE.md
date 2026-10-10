# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project

CAD Mouse MK2: a DIY 6DoF CAD controller (SpaceMouse-style). A knob on a 3D-printed PETG spring moves a magnet over a custom PCB with three Infineon TLx493D 3D magnetic sensors; a Seeed XIAO RP2040 turns the readings into 6 axes (Tx, Ty, Tz, Rx, Ry, Rz) and sends them over USB HID. Original design by Salim Benbouziyane ([build video](https://youtu.be/62xlzGs8LXA)); this fork reworks the firmware motion pipeline and adds 3Dconnexion driver compatibility. License: CC BY-NC-SA 4.0.

## Layout

| Path | Contents |
|------|----------|
| `firmware/` | PlatformIO firmware (Arduino on the earlephilhower RP2040 core) |
| `platformio.ini` | Build config at the repo root; points `src_dir`/`include_dir`/`test_dir` into `firmware/` |
| `enclosure/` | STL (print), STEP (assembly), Fusion 360 sources (incl. parametric spring) |
| `pcbs/` | Eagle sources (`src/`), Gerbers + pick-and-place (`fab/`), sensor board BOM (`bom/`) |
| `bom/` | Top-level parts list |

## Firmware architecture

- `main.cpp` creates global controllers and starts the state machine in `CalibratingState`.
- State machine (`StateMachine`, `firmware/src/states/`):
  - `Calibrating` captures `Config::ZERO_SAMPLES` baseline samples, then goes to `Idle`.
  - `Idle` runs the motion pipeline and sends HID reports. It goes back to `Calibrating` on a button request, or to `Sleep` after `IDLE_SLEEP_TIMEOUT_MS` (2 min) without activity.
  - `Sleep` turns the LEDs off and wakes to `Idle`.
- Controllers (`firmware/src/controllers/`): `Sensor` (3× TLx493D), `Motion` (pipeline), `HID` (TinyUSB reports), `Input` (AceButton side buttons), `LED` (NeoPixel), `Telemetry` (serial output).
- `firmware/include/Config.h` is the single place for pins, gains, matrices, deadzones, filter and LED settings.

### Motion pipeline (`MotionController::compute`)

```
raw[9] → baseline subtract → TRANSFORM 6×9 → COMP 6×6 → gain × sign
       → response curve → biquad LP → per-axis deadzone → clamp (±AXIS_LIMIT) → out[6]
```

Sensor order: `mag1` = bottom, `mag2` = top left, `mag3` = top right. Matrix columns are `[m1x, m1y, m1z, m2x, m2y, m2z, m3x, m3y, m3z]`. Stage-by-stage details and tuning tips are in [`firmware/README.md`](firmware/README.md); design rationale and math are in [`firmware/SPEC.md`](firmware/SPEC.md).

### HID / driver emulation

`platformio.ini` sets the USB VID/PID to 3Dconnexion's (`0x256F`/`0xC631`, "SpaceMouse Pro Wireless (cabled)") so the official 3DxWare driver recognizes the device. `HIDController` sends report ID 1 (translation, 3× int16), ID 2 (rotation, 3× int16) and ID 3 (buttons, 32 bits), and only sends motion when values change.

## Build and test

```bash
pio run                      # build firmware (default env: seeed_xiao_rp2040)
pio run -t upload            # flash the XIAO RP2040
pio test -e native           # desktop unit tests for MotionController
```

- The `native` env compiles no sources (`build_src_filter = -<*>`). Tests `#include` `MotionController.cpp` directly and use the Arduino shim in `firmware/test/shims/Arduino.h`. A new dependency in `MotionController` or `Config.h` may need a shim symbol there.
- The `native` env pins `toolchain-gccmingw32`, which suggests the tests are run on Windows (inferred). On Linux/macOS that package line may need to be removed.
- Tests are in `firmware/test/test_motion/test_motion.cpp`. The characterization tests pin the default pipeline output. With identity `COMP`, the default `TRANSFORM` and `RESPONSE_LINEARITY = 1.0`, output must match the original firmware.

## Conventions

- Firmware constraints (SPEC §5): no dynamic allocation, `float` only (no `double`), keep `compute()`'s public signature, keep the telemetry output format.
- New tuning parameters go in `Config.h` with a default that preserves current behavior. Add a test for the new stage.
- After a pipeline change, both `pio test -e native` and `pio run -e seeed_xiao_rp2040` must pass. `TASKS.md` records the Flash/RAM delta for each phase.
- Commit messages use the `feat:` / `fix:` prefix style.
- Planning and progress are tracked in [`firmware/TASKS.md`](firmware/TASKS.md), which is written in Portuguese.

## Sensor power-up and I2C addressing

All three TLx493D sensors boot on the same default I2C address. `SensorController::begin()` powers them up one at a time through per-sensor load switches (`PIN_MAG1_LS`=D10, `PIN_MAG2_LS`=D9, `PIN_MAG3_LS`=D8) and re-addresses each one before powering the next. On `main` the load switches are AP22816 with active-low `!EN`, inverted in hardware, so the firmware writes HIGH = on and LOW = off. The LED ring has its own 5 V AP22816 switch on `PIN_LED_LS` (D1). Changing switch hardware or sensor generation means revisiting this sequence and the on/off polarity.

## Work on other branches (not on `main`)

- **`feat/motion-pipeline-improvements`**: Edilson's current local branch. Adds a `LEGACY_MOTION_FORMULAS` toggle in `Config.h` (commented out by default) that restores the original hard-coded decomposition instead of `TRANSFORM`. It also adds sensor board v11/v13 CAM outputs (`pcbs/src/Placa-sensor v*/`) and `docs/referencia.txt` (link to the build video).
- **`feature/a1b6-sensor-migration`**: hardware-driven firmware changes, with specs in Portuguese:
  - The mounted sensors are **TLI493D-A1B6 (gen 1)**, not A2B6 as `main` assumes. The class becomes `TLx493D_A1B6`. The sensors boot on A4 (0x1F, 7-bit, SDA low): MAG1 → A6 (0x0F), MAG2 → A5 (0x1B), MAG3 stays on A4. `setSensitivity()` is removed because gen 1 is full range only. Specs: `.kiro/specs/a1b6-sensor-migration/` (requirements, design, tasks; all done).
  - The sensor switches U1–U3 change from AP22816 to **AO3407 PMOS** (LOW = on, HIGH = off; the gate needs a pull-up so the sensors stay off at boot). U4, the 5 V LED switch, stays on AP22816. Specs: `.kiro/specs/pmos-power-switch-support/requirements.md`, `firmware/KIRO_SPEC_AP22816_AO3407.md`, `firmware/CONVERSA_AP22816_KIRO.md`.
  - `SENSOR_ADDR_RETRY` recovers an I2C address that persists across power cycles. `USE_ONBOARD_LED` uses the XIAO's NeoPixel for testing without the ring.
  - The branch also adds a Kiro hook `.kiro/hooks/build-and-flash.kiro.hook`, `parse_sch.py` and Fusion PCB sources (`pcbs/fusion/`).
- The sensor generation is not consistent across branches: `main` and the v13 partlist say A2B6, while the A1B6 specs say the boards carry A1B6. Confirm which hardware a change targets before touching `SensorController`.

## Open items (from `firmware/TASKS.md`)

- End-to-end test on real hardware (Fusion 360 / FreeCAD).
- Tune the `COMP` and `RESPONSE_LINEARITY` coefficients on the physical device.
- Update the bleed tests once `COMP` is tuned.

## Gotchas

- `platformio.ini` sets `-DDISABLE_USB_SERIAL=1` while `Config::ENABLE_TELEMETRY = true`. Telemetry output (needed for the README's cross-axis tuning steps) probably does not reach USB serial in the current build (inferred, not verified on hardware).
- The README notes the PETG spring's longevity is uncertain. A knob redesign may be needed.

## Docs

- [`README.md`](README.md): project overview, video, license
- [`firmware/README.md`](firmware/README.md): config parameters, pipeline stages, tuning guide
- [`firmware/SPEC.md`](firmware/SPEC.md): motion processing spec (status: implemented, phases 1–5)
- [`firmware/TASKS.md`](firmware/TASKS.md): implementation checklist by phase
- On branch `feature/a1b6-sensor-migration`: `.kiro/specs/` (A1B6 migration, PMOS power switch), `firmware/KIRO_SPEC_AP22816_AO3407.md`, `firmware/CONVERSA_AP22816_KIRO.md`
