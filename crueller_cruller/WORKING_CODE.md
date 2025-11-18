# WORKING CODE - DO NOT BREAK

This document marks code sections that are **WORKING** and should not be modified without careful testing.

## Problem 1: POV LED Calibration (WORKING ✅)

The POV LED calibration system is working. The LED can be stabilized by adjusting the radius.

### Critical Files:
- `crueller_cruller.ino`: `getMeltyParameters()` - LED timing calculation
- `crueller_cruller.ino`: `updateHeadingLED()` - POV effect rendering
- `radius_tuner.cpp`: `processStickInput()` - Real-time radius adjustment

### What Works:
- Right stick X adjusts radius in 0.1mm increments
- Radius adjustment works while spinning
- POV LED shows at fixed position (20% offset)
- LED window size varies with RPM (10-90% of rotation)
- Config mode shows purple POV for tuning

### DO NOT MODIFY:
- LED timing calculation in `getMeltyParameters()` (lines ~578-593)
- POV rendering logic in `updateHeadingLED()` (lines ~598-624)
- Radius tuner increment logic (0.1mm steps)

## Problem 2: ESC Communication

**DUAL IMPLEMENTATION** - Both DShot and PWM are available!

### How to Switch:
Edit `esc.h` line 9:
- `#define USE_DSHOT 1` → Use DShot600 (EXPERIMENTAL)
- `#define USE_DSHOT 0` → Use PWM (WORKING for basic spin)

### DShot Implementation (EXPERIMENTAL ⚠️):
- `esc.cpp`: DShot600 via DShotRMT library
- Startup: 2 seconds of 0 commands for arming
- Main loop: Continuous DShot commands (no delays)
- Status: ⚠️ NOT YET VERIFIED - Upload and test

### PWM Implementation (WORKING ✅):
- `esc.cpp`: 50Hz PWM via ESP32 LEDC
- Pulse width: 1006us (armed) to 2006us (full throttle)
- Status: ✅ WORKING - Was working for basic spin (not fast enough for translation)

### If DShot Works:
- Mark DShot section as WORKING
- Document arming sequence
- Keep PWM as backup option

### If DShot Fails:
- Switch to PWM (`USE_DSHOT 0`)
- PWM works for basic spin, but may not be fast enough for translation

## Version Control Strategy

1. **Before making changes**: Create a git branch
2. **Test thoroughly**: Verify both problems still work
3. **Mark working code**: Add `// WORKING - DO NOT MODIFY` comments
4. **Document changes**: Update this file when status changes

