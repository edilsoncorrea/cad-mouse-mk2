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
