// Minimal Arduino.h shim for native (desktop) test builds.
// Only provides the symbols that Config.h and MotionController need.
#pragma once

#include <cmath>
#include <cstdint>

// Pin constants used by Config.h — values don't matter for motion tests.
constexpr int D0  = 0;
constexpr int D1  = 1;
constexpr int D2  = 2;
constexpr int D3  = 3;
constexpr int D8  = 8;
constexpr int D9  = 9;
constexpr int D10 = 10;
