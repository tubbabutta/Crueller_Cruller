# Testing Checklist

Before marking code as WORKING or making changes, verify:

## Problem 1: POV LED Calibration ✅ (WORKING)

- [ ] Robot spins up correctly
- [ ] POV LED appears at fixed position (not drifting)
- [ ] Right stick X adjusts radius (left = decrease, right = increase)
- [ ] Radius adjustment works while spinning
- [ ] LED stabilizes when radius is tuned correctly
- [ ] Config mode shows purple POV for tuning

**If all checked:** Code is WORKING - do not modify marked sections

## Problem 2: ESC Communication ⚠️ (TESTING)

### DShot600 Test:
- [ ] ESC arms on startup (2 second sequence)
- [ ] ESC stays armed when idle (continuous 0 commands)
- [ ] Motor responds to throttle input
- [ ] Motor spins smoothly
- [ ] Translation works (robot moves in commanded direction)

**If all checked:** Mark DShot code as WORKING
**If any fail:** Document failure and consider alternatives

### If DShot Fails:
- [ ] Try different DShot rate (DSHOT300, DSHOT150)
- [ ] Check RMT channel conflicts
- [ ] Verify ESC firmware supports DShot
- [ ] Consider reverting to PWM (was working for basic spin)

## Before Making Changes

1. **Create git branch**: `git checkout -b feature/your-change-name`
2. **Read WORKING_CODE.md**: Understand what's locked in
3. **Test current code**: Verify both problems still work
4. **Make changes**: Only modify non-WORKING code
5. **Test again**: Verify nothing broke
6. **Update WORKING_CODE.md**: If something new works

## Git Strategy

```bash
# Before changes
git checkout -b feature/experimental-change
# Make changes
# Test thoroughly
# If it works:
git commit -m "Feature: description"
git checkout main
git merge feature/experimental-change
# If it breaks:
git checkout main  # Discard changes
```

