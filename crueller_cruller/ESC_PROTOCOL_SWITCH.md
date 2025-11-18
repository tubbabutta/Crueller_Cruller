# ESC Protocol Switching Guide

## Quick Switch

Edit `esc.h` line 9:
```cpp
#define USE_DSHOT 1  // 1 = DShot600, 0 = PWM
```

## DShot600 (EXPERIMENTAL)
- **Status**: Testing - not yet verified
- **Pros**: Fast enough for translation, digital protocol
- **Cons**: May have arming issues
- **When to use**: When you need fast translation control

## PWM (WORKING)
- **Status**: ✅ Working for basic spin
- **Pros**: Proven to work, simple
- **Cons**: May not be fast enough for translation
- **When to use**: When DShot isn't working or for testing

## Testing Checklist

### DShot Test:
1. Set `USE_DSHOT 1` in `esc.h`
2. Upload code
3. Check if ESC arms (should hear beep after 2 seconds)
4. Test throttle response
5. Test translation (if motor spins)

### PWM Test:
1. Set `USE_DSHOT 0` in `esc.h`
2. Upload code
3. Check if ESC arms (should hear beep immediately)
4. Test throttle response
5. Test basic spin (should work)

## Both Implementations Preserved

Both code paths are in `esc.cpp`:
- Lines 3-25: DShot implementation
- Lines 26-81: PWM implementation

You can switch between them without losing either!

