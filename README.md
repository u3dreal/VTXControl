# VTXControl

Arduino library providing video transmitter (VTX) control by SmartAudio/Tramp protocol.

This C/C++ code uses modified SoftwareSerialWithHalfDuplex library (part code taken from CustomSoftwareSerial to support different configuration of serial port (especially 8N2)), some code taken from BetaFlight and ArduPilot (SmartAudio and Tramp protocols support) code.

This code created to use features of Tramp/SmartAudio on VTX (like switching power modes and channels/frequencies) by code/wire on Arduino driven systems or robots.

VTXControl works in two modes/protocols - SmartAudio and Tramp, communications with VTX established by software serial port (SoftwareSerialWithHalfDuplex).

## Supported Platforms

- **AVR (Arduino Uno, Mega, etc.)** — uses `SoftwareSerialWithHalfDuplex` (interrupt-driven, pin-change based)
- **ESP32** — uses `ESP32HalfDuplex` (polling-based, bit-banged GPIO)

Platform selection is automatic at compile time via `#if defined(ESP32)` guards.

## ESP32 Support

The library now runs on ESP32 using a new `ESP32HalfDuplex` transport layer:

- **Polling RX** — `available()` busy-waits for up to 150ms for the first byte, then 5ms between subsequent bytes. No ISR, no pin-change interrupt dependency.
- **Bit-banged TX** — `sendByte()` drives the GPIO directly with `wait_us()` microsecond delays.
- **Line settling** — `listen()` includes a 1ms settling delay after TX to suppress false start bits from line ringing on half-duplex single-wire connections.
- **Buffer drain** — `flush()` clears the circular buffer and waits for the line to be idle for 10ms.

### SmartAudio v2/v2.1 Response Parsing Fix

SMARTAUDIO_RSP_GET_SETTINGS_V2 and V21 handlers now read channel and power with correct byte offsets:

```
// Response layout: AA 55 CMD LEN [VER] [CH] [PW] [MODE] [FREQ_H] [FREQ_L] ... [CRC]
ch_index = buffer[5];    // was incorrectly reading buffer[4]
pwr_Level = buffer[6];
```

This fixes silent misconfiguration on SmartAudio v2/v2.1 VTXes where the version byte at buffer[4] was being interpreted as the channel index.

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
  bool ok = vtx.updateParameters();
  Serial.printf("VTX detected: %s, ch=%d, pwr=%d\n",
    ok ? "yes" : "no", vtx.getChannelIndex(), vtx.getPowerLevel());
}

void loop() {
  vtx.setChannel(0);      // 5865 MHz
  vtx.setPower(1);        // 200 mW (index 1)
  delay(1000);
  vtx.setChannel(39);     // 5917 MHz
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
  int vtxMode,          // SmartAudio or Tramp
  int softPin,          // GPIO pin for half-duplex communication
  const uint16_t powers[],     // power levels in mW
  int powers_len,               // number of power levels
  const uint16_t freqs[],       // frequency table in MHz
  int freqs_len,                // number of frequencies
  int responseTimeOut = 1000,   // response timeout (ms)
  bool smartBaudRate = true,    // auto-detect baud rate
  int numtries = 3              // retries per command
);
```

## API

| Method | Description |
|--------|-------------|
| `updateParameters()` | Detect VTX and read current settings |
| `setChannel(int idx)` | Set frequency by channel table index |
| `setFrequency(uint16_t freq)` | Set frequency by value (MHz) |
| `setPower(int idx)` | Set power by power table index |
| `setPowerInmW(uint16_t mW)` | Set power by value (mW) |
| `setNextChannel()` / `setPrevChannel()` | Increment/decrement channel |
| `getChannelIndex()` | Current channel index |
| `getPowerLevel()` | Current power level index |

## Notes

- SmartAudio uses **4800 baud** (or auto-detected, typically 4980 baud), **8N2** (8 data bits, no parity, 2 stop bits)
- Tramp uses **9600 baud**, **8N1**
- The library does not support PitMode or GetTemperature
- CRC verification can be disabled via `SMARTAUDIO_IGNORE_CRC` in `VTX_SmartAudio.h`
- Eachine TX5258 and similar clones may need `SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END` set to true
- Use `VTX_Test` sketch to diagnose unknown VTX models

## VTX_Test

The `VTX_Test/` folder contains a diagnostic sketch that can:
- Send arbitrary SmartAudio commands
- Analyze VTX responses
- Try different baud rates
- Dump raw response bytes

Use this to characterize a new VTX before integrating.

## License

LGPL 2.1 — see LICENSE file.
