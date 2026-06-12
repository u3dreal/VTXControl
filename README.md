# VTXControl

Arduino library providing video transmitter (VTX) control via SmartAudio and Tramp protocols.

Supports switching power levels, channels/frequencies, and PitMode on VTX modules
from Arduino-based systems (AVR, ESP32, etc.).

Protocol implementations derived from BetaFlight and ArduPilot. Serial transport
uses `SoftwareSerialWithHalfDuplex` (AVR, interrupt-driven) or `ESP32HalfDuplex`
(ESP32, polling bit-banged), selected automatically at compile time.

## Supported Platforms

| Platform | Transport | Mechanism |
|----------|-----------|-----------|
| **AVR** (Uno, Mega, etc.) | `SoftwareSerialWithHalfDuplex` | Interrupt-driven, pin-change based |
| **ESP32** | `ESP32HalfDuplex` | Polling-based, bit-banged GPIO |

## Transport Details

### ESP32HalfDuplex

- **Polling RX** — `available()` busy-waits up to 150ms for the first byte, then 5ms between subsequent bytes. No ISR dependency.
- **Bit-banged TX** — GPIO driven directly with `wait_us()` delays.
- **Line settling** — `listen()` inserts a 1ms delay after TX to suppress false start bits from line ringing.
- **Buffer drain** — `flush()` clears the circular buffer and waits for idle line (≥10ms).

### SmartAudio v2/v2.1 Response Parsing

Byte-index based parsing replaces the packed-struct reinterpret-cast that had wrong alignment:

```
AA 55 CMD LEN [VER] [CH] [PW] [MODE] [FREQ_H] [FREQ_L] ... [CRC]
              [4]   [5]   [6]   [7]
```

Fixes silent misconfiguration where `buffer[4]` (version byte) was previously read as the channel index.

## Usage

### SmartAudio Mode (ESP32 example)

```cpp
#include <VTXControl.h>

const uint16_t powers[] = {25, 200, 400, 800};
const uint16_t freqs[] = {
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,  // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866,   // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,   // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,   // Fatshark
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,   // Race
};

VTXControl vtx(SmartAudio, 17, powers, 4, freqs, 40);

void setup() {
  Serial.begin(115200);
  if (vtx.updateParameters())
    vtx.setPitMode(false);       // ensure VTX is transmitting
}

void loop() {
  vtx.setChannel(0);             // 5865 MHz
  vtx.setPower(1);               // 200 mW (index 1)
  delay(1000);
  vtx.setChannel(39);            // 5917 MHz
  delay(1000);
}
```

### Tramp Mode (AVR example)

```cpp
VTXControl vtx(Tramp, 53, powers, 4, freqs, 40);

void setup() {
  Serial.begin(115200);
  vtx.updateParameters();
}
```

## Constructor

```cpp
VTXControl(
  int vtxMode,                  // SmartAudio or Tramp
  int softPin,                  // GPIO pin for half-duplex communication
  const uint16_t powers[],      // power levels in mW
  int powers_len,               // number of power levels
  const uint16_t freqs[],       // frequency table in MHz
  int freqs_len,                // number of frequencies
  int responseTimeOut = 1000,   // response timeout per poll (ms)
  bool smartBaudRate = true,    // auto-scan baud rates on failure
  int numtries = 3              // retries before changing baud rate
);
```

## API

| Method | Description |
|--------|-------------|
| `updateParameters()` | Detect VTX and read current settings (channel, power, PitMode) |
| `setChannel(int idx)` | Set frequency by channel table index |
| `setFrequency(uint16_t freq)` | Set frequency by value (MHz); uses channel table if found, else direct freq command |
| `setPower(int idx)` | Set power by power table index |
| `setPowerInmW(uint16_t mW)` | Set power by value (mW) |
| `setNextChannel()` / `setPrevChannel()` | Increment/decrement channel (wraps around) |
| `setPitMode(bool enabled)` | Enable or disable PitMode (SmartAudio only) |
| `getChannelIndex()` | Current channel index (`-1` if unknown) |
| `getPowerLevel()` | Current power level index (`-1` if unknown) |
| `getPitMode()` | Current PitMode state |
| `getErrors()` | Bitmask of `VTXErrors` flags from last failed operation |
| `clearErrors()` | Reset error flags |
| `getSpeed()` | Current serial baud rate negotiated with the VTX |

## Notes

- SmartAudio uses **4800 baud** (or auto-detected, typically ~4980 baud), **8N2**
- Tramp uses **9600 baud**, **8N1**
- PitMode is supported for SmartAudio only (not Tramp)
- Temperature query (`GetSensor`) for Tramp is parsed but not exposed
- CRC verification can be disabled via `SMARTAUDIO_IGNORE_CRC` in `VTX_SmartAudio.h`
- Eachine TX5258 and similar clones may need `SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END` set to `true`
- Use the `VTX_Test` sketch to diagnose unknown VTX models

## VTX_Test

The `VTX_Test/` folder contains a diagnostic sketch that can:
- Send arbitrary SmartAudio commands
- Analyze VTX responses
- Try different baud rates
- Dump raw response bytes

Use this to characterize a new VTX before integrating.

## License

`ESP32HalfDuplex` and `SoftwareSerialWithHalfDuplex` are LGPL 2.1.
The remainder of VTXControl is GPL 3.0 — see LICENSE file.
