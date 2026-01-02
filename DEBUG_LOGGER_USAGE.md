# OGX-Mini Debug Logger - Usage Guide

## Overview

The Debug Logger is a high-performance USB CDC (serial) device driver that logs every operation of interest on the Pico, specifically designed to help diagnose phantom key presses, battery issues, and other hardware anomalies.

## Features

### What Gets Logged

1. **Button Events** - Every button press and release with microsecond timestamps
2. **D-Pad Changes** - Directional pad state transitions
3. **Joystick Positions** - Left and right stick movements (decimal and hex, with 1% change threshold)
4. **Trigger Values** - L/R trigger positions (decimal and hex, with 1% change threshold)
5. **Battery Status** - Raw value, hex, and percentage
6. **Rumble Output** - Vibration motor commands
7. **Motion Sensors** - Accelerometer and gyroscope (for DualShock controllers)
8. **Flash Operations** - Every NVS read/write/erase with timing metrics
9. **Performance Metrics** - Max process time per 1000 iterations
10. **System Events** - Mode changes, initialization, etc.

### Performance Characteristics

- **Non-blocking**: 8KB ring buffer prevents blocking on USB writes
- **Change detection**: Only logs when values actually change
- **Noise filtering**: 1% thresholds prevent joystick/trigger noise logging
- **Memory safe**: Zero heap allocations in hot path
- **Microsecond timestamps**: Precise timing for debugging timing issues
- **Sub-millisecond overhead**: Minimal impact on gamepad processing

## How to Activate Debug Logger Mode

### Method 1: Button Combo (Recommended)

1. Connect your controller to the OGX-Mini
2. Hold: **START + X** for 3 seconds
3. The Pico will reboot in Debug Logger mode
4. Connect the Pico to your computer via USB
5. Open a serial terminal (see below)

### Method 2: Flash Default Mode

Modify `UserSettings.cpp` to set DEBUG_LOGGER as the default:
```cpp
DeviceDriverType UserSettings::DEFAULT_DRIVER()
{
    return DeviceDriverType::DEBUG_LOGGER;  // Changed from VALID_DRIVER_TYPES[0]
}
```

## Connecting to the Debug Logger

### Windows

1. **Device Manager**
   - Open Device Manager
   - Look for "USB Serial Device (COMx)" under "Ports (COM & LPT)"
   - Note the COM port number

2. **Terminal Software Options**
   - **PuTTY**: Connection type: Serial, Port: COMx, Speed: 115200
   - **Tera Term**: File → New Connection → Serial: COMx
   - **Arduino Serial Monitor**: Select COMx, 115200 baud
   - **PowerShell**: `mode COMx BAUD=115200 PARITY=N DATA=8 ; type COMx`

### Linux / macOS

1. **Find the Device**
   ```bash
   # Linux
   ls /dev/ttyACM*

   # macOS
   ls /dev/cu.usbmodem*
   ```

2. **Connect with Screen**
   ```bash
   # Linux
   sudo screen /dev/ttyACM0 115200

   # macOS
   screen /dev/cu.usbmodem14201 115200

   # Exit: Ctrl+A, then K, then Y
   ```

3. **Connect with Minicom**
   ```bash
   sudo minicom -D /dev/ttyACM0 -b 115200
   ```

4. **Connect with Cat (read-only)**
   ```bash
   sudo cat /dev/ttyACM0
   ```

5. **Save to File**
   ```bash
   sudo cat /dev/ttyACM0 > debug_log.txt
   ```

## Log Format

### Example Output

```
[12345678] EVT: === DEBUG LOGGER INITIALIZED ===
[12345679] EVT: Format: [timestamp_us] CHAN: data
[12345680] EVT: Channels: BTN=Buttons, JOY=Joysticks, TRIG=Triggers, BAT=Battery, RUM=Rumble, FLASH=Flash, TIME=Timing, EVT=Event
[12345681] EVT: ================================
[15234567] FLASH: op=READ key=driver_type len=1
[15234890] TIME: flash_read took 323us
[15678901] BTN+: START (0x0080)
[15890234] JOY: L=(-1234,5678 | 0xFB2E,0x162E) R=(0,0 | 0x0000,0x0000)
[16123456] BTN+: A (0x0081)
[16234567] TRIG: L=128 (0x80) R=0 (0x00)
[16345678] BTN-: START (0x0001)
[17456789] BAT: raw=204 (0xCC) pct=80.0%
[18567890] RUM: L=64 (0x40) R=32 (0x20)
[19678901] FLASH: op=WRITE key=profile_1 len=256
[19679234] TIME: flash_write took 45678us
[20000000] PERF: max_process_time=125us, iterations=1000
```

### Channel Descriptions

| Channel | Description | Example |
|---------|-------------|---------|
| **BTN+** | Button pressed | `BTN+: A B X (0x0007)` |
| **BTN-** | Button released | `BTN-: START (0x0080)` |
| **DPAD** | D-pad change | `DPAD: NONE -> UP_RIGHT (0x09)` |
| **JOY** | Joystick positions | `JOY: L=(x,y \| hex,hex) R=(x,y \| hex,hex)` |
| **TRIG** | Trigger values | `TRIG: L=255 (0xFF) R=128 (0x80)` |
| **BAT** | Battery level | `BAT: raw=204 (0xCC) pct=80.0%` |
| **RUM** | Rumble output | `RUM: L=255 (0xFF) R=128 (0x80)` |
| **MOTION** | Motion sensors | `MOTION: accel=(x,y,z) gyro_z=1234` |
| **FLASH** | Flash operation | `FLASH: op=WRITE key=profile_1 len=256` |
| **TIME** | Timing metric | `TIME: flash_write took 45678us` |
| **EVT** | System event | `EVT: Mode changed to XInput` |
| **PERF** | Performance stats | `PERF: max_process_time=125us, iterations=1000` |

### Button Names

- Face: `A`, `B`, `X`, `Y`
- Shoulders: `LB`, `RB`, `L3`, `R3`
- Menu: `START`, `BACK`, `SYS`, `MISC`

### Value Ranges

- **Joysticks**: -32768 to 32767 (int16_t)
- **Triggers**: 0 to 255 (uint8_t)
- **Battery**: 0 to 255 (0% to 100%)
- **Rumble**: 0 to 255 (off to max)

## Debugging Common Issues

### Phantom Button Presses

Look for unexpected `BTN+` events:
```
[12345678] BTN+: R3 (0x0020)  <-- Phantom press?
[12345890] BTN-: R3 (0x0000)  <-- Released 212us later
```

**Check for:**
1. Timestamp differences (too short = electrical noise)
2. Patterns (repeating at intervals = software bug)
3. Correlation with other events (triggered by joystick movement?)

### Battery Issues

Monitor battery readings:
```
[10000000] BAT: raw=255 (0xFF) pct=100.0%  <-- Initial
[20000000] BAT: raw=0 (0x00) pct=0.0%      <-- Sudden drop? Bad reading
[30000000] BAT: raw=204 (0xCC) pct=80.0%   <-- Normal
```

**Check for:**
1. Sudden jumps in battery level
2. Stuck at 0 or 255 (ADC not connected)
3. Rapid fluctuations (noisy ADC)

### Joystick Deadzone Problems

Compare raw values vs expected deadzones:
```
[12345678] JOY: L=(50,30 | 0x0032,0x001E) R=(0,0 | 0x0000,0x0000)
```

**Small values near center?**
- Should be filtered by deadzone
- Check profile settings in WebApp mode

### Flash Performance

Monitor flash operation timing:
```
[15000000] FLASH: op=WRITE key=profile_1 len=256
[15045678] TIME: flash_write took 45678us  <-- 45ms, normal for flash
```

**Slow flash operations:**
- Writes: 20-50ms (normal, erases sector)
- Reads: <1ms (fast, no erase)
- Erase all: 100-500ms (erases 4 sectors)

## Advanced Usage

### Filtering Logs (Linux/macOS)

**Only button events:**
```bash
sudo cat /dev/ttyACM0 | grep "BTN"
```

**Only flash operations:**
```bash
sudo cat /dev/ttyACM0 | grep -E "FLASH|TIME"
```

**Save timestamped log:**
```bash
sudo cat /dev/ttyACM0 | ts '[%Y-%m-%d %H:%M:%.S]' > debug.log
```

### Analyzing Timing Issues

**Find slow operations:**
```bash
cat debug.log | grep "TIME:" | awk '{print $4}' | sort -n | tail -10
```

**Count phantom presses:**
```bash
cat debug.log | grep "BTN+: R3" | wc -l
```

## Troubleshooting

### No Serial Device Appears

1. **Check USB cable**: Must support data (not charge-only)
2. **Try different USB port**: Some hubs cause issues
3. **Windows**: Install USB CDC drivers (usually automatic)
4. **Linux**: Add user to `dialout` group: `sudo usermod -a -G dialout $USER`

### Garbled Output

- **Wrong baud rate**: Must be 115200
- **Terminal settings**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Multiple terminals**: Only one can connect at a time

### Missing Logs

- **Ring buffer overflow**: 8KB buffer fills at ~30K events/sec
  - Solution: Reduce USB latency or increase RING_BUFFER_SIZE
- **Change threshold too high**: Analog changes <1% filtered
  - Solution: Lower JOYSTICK_THRESHOLD or TRIGGER_THRESHOLD in code

### Logs Stop Flowing

- **USB disconnected**: Check cable/connection
- **Pico crashed**: Look for watchdog reset
- **Terminal closed**: Reconnect serial terminal

## Performance Impact

The debug logger is optimized for minimal overhead:

| Operation | Overhead | Notes |
|-----------|----------|-------|
| No changes | <10µs | Fast path with no logging |
| Button change | ~50µs | Format + ring buffer write |
| Joystick change | ~100µs | Format 4 values + write |
| Flash operation | +100-200µs | Timing instrumentation |
| Process loop | <200µs | Including all checks |

**Total impact:** <1% of CPU time in typical usage.

## Code Customization

### Change Analog Thresholds

Edit `DebugLogger.h`:
```cpp
// In GamepadState struct
static constexpr int16_t JOYSTICK_THRESHOLD = 328;   // ~1% of 32768
static constexpr uint8_t TRIGGER_THRESHOLD = 3;      // ~1% of 255
```

### Increase Ring Buffer Size

Edit `DebugLogger.h`:
```cpp
static constexpr size_t RING_BUFFER_SIZE = 16384;  // 16KB instead of 8KB
```

### Add Custom Logging

From anywhere in the code:
```cpp
#include "USBDevice/DeviceDriver/DebugLogger/DebugLogger.h"

DebugLoggerDevice::log_event("Custom event happened");
DebugLoggerDevice::log_timing("custom_operation", duration_us);
DebugLoggerDevice::log_flash_op("CUSTOM", "key", len);
```

## Implementation Details

### Architecture

1. **Ring Buffer**: Lock-free circular buffer (8KB default)
2. **Change Detection**: Compares against previous state before logging
3. **Async Writes**: USB writes happen in background, never block
4. **Static Allocation**: No heap usage in hot path
5. **Threshold Filtering**: 1% change required for analog inputs

### Memory Usage

- **Ring buffer**: 8192 bytes (static)
- **State tracking**: ~50 bytes (static)
- **Stack per log**: ~256 bytes max (snprintf buffer)
- **Total**: ~8.5KB RAM

### File Structure

```
USBDevice/DeviceDriver/DebugLogger/
├── DebugLogger.h         # Header with class definition
└── DebugLogger.cpp       # Implementation

UserSettings/
└── NVSTool.h            # Modified: flash operation hooks

USBDevice/
├── DeviceDriverTypes.h  # Modified: DEBUG_LOGGER enum
└── DeviceManager.cpp    # Modified: driver registration

UserSettings/
└── UserSettings.cpp     # Modified: button combo + valid drivers
```

## Support

For issues or questions:
1. Check GitHub Issues: https://github.com/wiredopposite/OGX-Mini/issues
2. Include debug log excerpt
3. Specify controller model and Pico board type
4. Describe observed vs expected behavior

---

**Note:** The debug logger runs on Core 0 (same as USB device output). Core 1 continues handling USB host input polling independently, ensuring no input lag.
