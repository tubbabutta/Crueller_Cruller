# ESP32-S3 SuperMini Stadia Controller NeoPixel Controller

**⚠️ IMPORTANT COMPATIBILITY ISSUE ⚠️**

The Stadia controller uses **Bluetooth Classic (BR/EDR)**, but the **ESP32-S3 only supports BLE (Bluetooth Low Energy)**. This is a fundamental hardware incompatibility.

**This project may not work with ESP32-S3 + Stadia controller.**

**Solutions:**
1. **Use original ESP32** (not S3) which supports Bluetooth Classic, with [Bluepad32 library](https://github.com/ricardoquesada/bluepad32)
2. **Use a BLE-compatible controller** (Xbox Wireless Controller, Steam Controller) with ESP32-S3
3. **This code is experimental** - attempting to work around the limitation, but may not succeed

---

This sketch attempts to connect an ESP32-S3 SuperMini to a Google Stadia controller via Bluetooth and control two daisy-chained NeoPixels based on D-pad input.

## Features

- Bluetooth Low Energy (BLE) connection to Stadia controller
- D-pad color mapping:
  - **Up**: Blue
  - **Down**: Red
  - **Left**: Yellow
  - **Right**: Green
  - **No input**: White
- Two NeoPixels daisy-chained on pin 7
- AM32 ESC control via DShot600 on pin 6 (single direction, forward only)
- Throttle control via right trigger or right stick

## Hardware Requirements

- ESP32-S3 SuperMini
- Google Stadia controller (in Bluetooth mode)
- 2x NeoPixel LEDs (WS2812B or compatible) daisy-chained
- AM32 brushless ESC configured for single direction
- Appropriate power supply for NeoPixels (if using more than a few pixels)
- Appropriate power supply for ESC and motor (separate from ESP32 power)

## Wiring

- **NeoPixels**: 
  - Data: Connect to **Pin 7** (DIN of first NeoPixel)
  - Daisy chain: Connect DOUT of first NeoPixel to DIN of second NeoPixel
  - Power: 5V with adequate current capacity (60mA per pixel at full brightness)
  - Ground: Connect common ground between ESP32 and NeoPixels

- **ESC**:
  - DShot signal: Connect to **Pin 6** (ESC PWM/DShot input pin)
  - Power: Connect ESC power supply (typically 2S-6S LiPo, check ESC specs)
  - Ground: Connect common ground between ESP32 and ESC
  - **Important**: Keep ESC power supply separate from ESP32 power supply, but connect grounds together

## Software Requirements

1. **Arduino IDE** with ESP32 board support:
   - Add ESP32 board support: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Install "ESP32" board package
   - Select "ESP32S3 Dev Module" or "ESP32-S3-SuperMini" board

2. **Required Libraries**:
   - Adafruit NeoPixel library (install via Library Manager)
   - ESP32 BLE libraries (included with ESP32 board package)

## Installation

1. Install Arduino IDE and ESP32 board support
2. Install Adafruit NeoPixel library via Library Manager
3. Open `stadia_neopixel.ino` in Arduino IDE
4. Select your board: **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**
5. Select the correct port: **Tools → Port**
6. Adjust settings if needed:
   - **Partition Scheme**: Default (or as needed)
   - **PSRAM**: Enabled (if available on your board)
   - **Upload Speed**: Try lowering to 115200 if upload fails
   - **Erase All Flash Before Sketch Upload**: "Enabled" (if upload keeps failing)
7. Upload the sketch

### Upload Troubleshooting

If upload fails with "chip stopped responding" or similar errors:

1. **Manual reset**: Hold the **BOOT** button, press and release **RESET**, then release **BOOT**
2. **Lower upload speed**: Go to **Tools → Upload Speed** and try 115200 or 921600
3. **Try different USB cable/port**: Use a data-capable USB cable (not charge-only)
4. **Close Serial Monitor**: Make sure Serial Monitor is closed during upload
5. **Erase flash**: Use **Tools → Erase Flash → All Flash Contents**, then try uploading again
6. **Check drivers**: Ensure ESP32-S3 USB drivers are installed correctly
7. **Power**: Ensure board has stable power (try powered USB hub if needed)

## Usage

### Connecting the Stadia Controller

1. **Put the Stadia controller in Bluetooth pairing mode**:
   - **First, make sure the controller is OFF** (LED is off, not connected to anything)
   - **Unpair from other devices first** (phone, computer, etc.) and wait 1-2 minutes
   - **Turn on the controller**: Press the **Stadia button** once (brief press, don't hold)
   - **Immediately hold Stadia button**: Right after turning on, hold the **Stadia button** for **3 seconds**
   - The controller LED should start **blinking/flashing** to indicate pairing mode
   - The controller will be discoverable for a limited time (usually 1-2 minutes)
   
2. **If the controller won't enter pairing mode**:
   - **Make sure controller is OFF first**: If holding the button turns it off, that means it's already on - let it turn off, wait 5 seconds, then try again
   - **Try this sequence**: 
     1. Controller should be OFF (LED off)
     2. Unpair from all devices and wait 1-2 minutes
     3. Press Stadia button once (brief) to turn on
     4. Immediately hold Stadia button for 3 seconds
   - **Check the LED**: When in pairing mode, the LED should be **blinking/flashing rapidly**, not solid
   - If LED is solid = connected to something (unpair and wait)
   
3. **Controller pairing tips**:
   - The controller must be **completely disconnected** from other devices before pairing
   - If you see the controller in your phone's Bluetooth list, **forget/remove it** first
   - The pairing window is **time-limited** - if it times out, put it back in pairing mode
   - Keep the controller **close to the ESP32** (within 3 feet) during pairing

2. **Power on the ESP32-S3 SuperMini**:
   - Upload the sketch if you haven't already
   - Power on or reset the ESP32
   - The ESP32 will automatically start scanning for the controller

3. **Monitor the connection**:
   - Open **Serial Monitor** in Arduino IDE (115200 baud)
   - You should see: `"Scanning for Stadia controller..."`
   - The ESP32 will scan for 5 seconds initially
   - When found, you'll see: `"Stadia controller found!"` and `"Connected to Stadia controller"`

4. **Verify connection**:
   - Once connected, the NeoPixels should respond to D-pad input
   - The ESC will respond to right trigger or right stick input
   - If connection fails, the ESP32 will automatically retry scanning

### What to Expect

- **On startup**: NeoPixels will be white, ESC will be stopped
- **D-pad input**: NeoPixels change color based on direction
- **Right trigger/stick**: Controls ESC throttle (forward only)
- **Disconnection**: NeoPixels turn white, ESC stops automatically

### If Connection Fails

**On the Controller:**
1. **Unpair from other devices**: 
   - Go to your phone/computer Bluetooth settings
   - Find "Stadia" or "Stadia Controller" and **forget/remove it**
   - This is critical - the controller can only connect to one device at a time

2. **Wait after unpairing** (IMPORTANT):
   - After unpairing from phone/computer, **wait 1-2 minutes**
   - The controller needs time to fully disconnect and reset its BLE stack
   - The LED will turn off, then you can put it in pairing mode
   - Don't try to pair immediately after unpairing

3. **Reset the controller** (if needed):
   - Hold **Stadia button** for **10 seconds** (longer than pairing mode)
   - This forces a full reset
   - Release and wait 5 seconds
   - Then wait another 1-2 minutes before trying pairing mode

4. **Enter pairing mode correctly**:
   - **Controller must be OFF first** (LED off)
   - If holding the button turns it off, that means it was already on - let it turn off completely
   - Wait 5 seconds after it turns off
   - **Press Stadia button once** (brief press) to turn on
   - **Immediately hold Stadia button for 3 seconds** (while it's turning on)
   - LED should start **blinking/flashing rapidly** = pairing mode
   - If LED is solid = connected to something (unpair and wait 1-2 minutes first)

4. **Timing**:
   - Put controller in pairing mode **BEFORE** powering on ESP32
   - Or put it in pairing mode **while** ESP32 is scanning
   - The pairing window is only 1-2 minutes, so timing matters

**On the ESP32:**
- Check Serial Monitor for discovered devices - the controller name might be different
- The ESP32 will automatically retry scanning every few seconds
- You may need to adjust `STADIA_DEVICE_NAME` in the code if your controller uses a different BLE name
- If you see "ACL buf alloc failed" errors, try changing the partition scheme (see troubleshooting)

## Configuration

You can adjust these constants in the sketch:

- `NEOPIXEL_PIN`: Pin number for NeoPixel data (default: 7)
- `NUM_PIXELS`: Number of NeoPixels (default: 2)
- `NEOPIXEL_BRIGHTNESS`: Brightness level 0-255 (default: 50)
- `ESC_PIN`: Pin number for ESC DShot signal (default: 6)
- `STADIA_DEVICE_NAME`: BLE device name to search for (default: "Stadia")

**Note**: The throttle input mapping (right trigger vs right stick) may need adjustment in `parseControllerInput()` based on your Stadia controller's HID report format. Check Serial Monitor to see the actual HID data structure.

## Troubleshooting

### BLE Compilation Errors (`'BLEScan' does not name a type`)

If you get compilation errors about BLE types not being found:

1. **Update ESP32 Board Package**:
   - Go to **Tools → Board → Boards Manager**
   - Search for "esp32" and update to the latest version (2.0.0 or later)
   - ESP32-S3 BLE support requires a recent version of the board package

2. **Verify Board Selection**:
   - Select **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**
   - Make sure you're using an ESP32-S3 board, not ESP32 or ESP32-C3

3. **Check Partition Scheme**:
   - Go to **Tools → Partition Scheme**
   - Select a scheme that includes BLE support (most default schemes do)

4. **Alternative: Use NimBLE Library**:
   - If standard BLE libraries don't work, you may need to use the NimBLE library
   - Install via Library Manager: search for "NimBLE-Arduino"
   - The code would need to be modified to use NimBLE APIs instead

5. **Verify BLE is Enabled**:
   - Some ESP32-S3 board packages may have BLE disabled by default
   - Check your board's configuration in the Arduino IDE
   - You may need to create a `sdkconfig` file or modify board settings

### Controller Not Connecting

**Step-by-step troubleshooting:**

1. **Verify controller is in pairing mode**:
   - Hold the Stadia button (center logo button) for 3 seconds
   - LED should be blinking/flashing
   - Controller must be in pairing mode when ESP32 scans

2. **Check Serial Monitor output**:
   - Open Serial Monitor at 115200 baud
   - Look for `"Found device: ..."` messages
   - Note the actual device name shown

3. **Controller name mismatch**:
   - If you see devices but not "Stadia controller found!", the name might be different
   - Common names: "Stadia", "Stadia Controller", or a MAC address-like name
   - Update `STADIA_DEVICE_NAME` in the code to match what you see

4. **Timing issues**:
   - Put controller in pairing mode FIRST, then power on/reset ESP32
   - Or wait for ESP32 to start scanning, then put controller in pairing mode
   - Controller pairing window is limited (usually 1-2 minutes)

5. **Automatic retry**:
   - The ESP32 will automatically retry scanning if connection fails
   - Wait a few seconds and check Serial Monitor again
   - You can also reset the ESP32 to force a new scan

6. **BLE range**:
   - Keep controller within a few feet of ESP32 during pairing
   - After connection, range is typically 10-30 feet depending on environment

### Fundamental Compatibility Issue

**The ESP32-S3 cannot connect to Stadia controllers** because:
- Stadia controller uses **Bluetooth Classic (BR/EDR)**
- ESP32-S3 only supports **BLE (Bluetooth Low Energy)**
- These are incompatible protocols

**If you're seeing connection failures, this is likely why.**

**Solutions:**
1. **Use original ESP32** (not S3) - supports Bluetooth Classic
   - Use [Bluepad32 library](https://github.com/ricardoquesada/bluepad32)
   - This is the proven solution for Stadia controllers
   
2. **Use a BLE-compatible controller** with ESP32-S3:
   - Xbox Wireless Controller (BLE mode)
   - Steam Controller
   - Other BLE gamepads
   - Use [BLE-Gamepad-Client library](https://github.com/tbekas/BLE-Gamepad-Client)

3. **This code is experimental** - attempting workarounds but may not work

### ACL Buffer Allocation Errors

If you see messages like `"ACL buf alloc failed"` or `"Free ACL mbufs: 36"`:

**These are usually harmless warnings**, not critical errors. The connection may still work despite these messages.

**To reduce these warnings:**

1. **Change Partition Scheme** (if available):
   - Go to **Tools → Partition Scheme**
   - Select a scheme with more memory for BLE (e.g., "Huge APP" or "Minimal SPIFFS")
   - This gives the BLE stack more memory to work with

2. **The code has been optimized** to use passive scanning instead of active scanning, which reduces buffer usage

3. **Test if it's working**:
   - Try pressing the D-pad - do the NeoPixels change color?
   - Try the right trigger/stick - does the ESC respond?
   - If these work, the errors are just warnings and can be ignored

4. **If connection is unstable**:
   - These errors can sometimes cause connection drops
   - Try resetting the ESP32
   - Make sure you're using a recent ESP32 board package version (3.0.0+)

### D-pad Not Working

- The HID report parsing may need adjustment based on your specific controller firmware
- Check Serial Monitor for D-pad value output (hex format)
- Adjust the `updatePixelsFromDpad()` function based on actual values you see
- Stadia controllers may use different HID report formats depending on firmware version

### NeoPixels Not Lighting

- Verify wiring: Data to pin 7, power and ground connected
- Check power supply: NeoPixels need adequate current (60mA per pixel at full brightness)
- Verify pin number matches your wiring
- Try reducing brightness if power supply is insufficient

## Notes

- The HID report parsing is based on standard gamepad formats but may need adjustment for your specific Stadia controller firmware
- If colors don't match D-pad directions, you may need to calibrate the D-pad value parsing
- The sketch includes debug output via Serial Monitor to help diagnose connection and parsing issues

## License

This code is provided as-is for educational and personal use.

