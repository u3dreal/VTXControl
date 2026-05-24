# Changelog

## Unreleased — ESP32 support

### Added
- **ESP32HalfDuplex transport layer** — new `ESP32HalfDuplex` class (`ESP32HalfDuplex.h/.cpp`) provides polling-based bit-banged half-duplex serial for ESP32. No ISR or pin-change interrupt dependency.
  - `available()` — busy-waits with dual timeout: 150ms for first byte, 5ms between subsequent bytes
  - `sendByte()` — drives GPIO directly with microsecond-level timing
  - `listen()` — 1ms settling delay after TX to suppress line-ringing false start bits on single-wire half-duplex
  - `flush()` — clears buffer + waits for 10ms idle line to drain in-flight bytes
- **ESP32 compile-time platform selection** — `VTXControl.h` and `VTXControl.cpp` use `#if defined(ESP32)` to conditionally include `ESP32HalfDuplex` or `SoftwareSerialWithHalfDuplex`

### Changed
- **`SoftwareSerialWithHalfDuplex.h`** — wrapped entire header in `#if !defined(ESP32)` / `#endif` to prevent AVR-specific includes (`avr/interrupt.h`, `avr/pgmspace.h`) on ESP32
- **`SoftwareSerialWithHalfDuplex.cpp`** — wrapped entire implementation in `#if !defined(ESP32)` / `#endif` to prevent compilation of AVR interrupt handlers on ESP32
- **`VTXControl.h`** — removed duplicate default argument values from constructor declaration (defaults only in definition)
- **`VTXControl.cpp`**:
  - Constructor: removed duplicate default argument values for `responseTimeOut`, `smartBaudRate`, `numtries`
  - `sa_readResponse()`: on ESP32, skips `waitForInMs(_responseTimeOut)` — `ESP32HalfDuplex::available()` does the blocking internally
  - `trampReadResponse()`: on ESP32, uses `delay(10)` polling loop instead of `waitForInMs(100)` for the initial response wait
- **SmartAudio v2/v2.1 response parsing fix** — `sa_parseResponseBuffer()` cases `SMARTAUDIO_RSP_GET_SETTINGS_V2` and `SMARTAUDIO_RSP_GET_SETTINGS_V21` now read channel and power with correct byte offsets (`buffer[5]` for channel, `buffer[6]` for power) instead of using the packed struct `SettingsResponseFrame`/`SettingsExtendedResponseFrame` reinterpret-cast, which had the wrong alignment

### Fixed
- False start bit on half-duplex single-wire after TX: line ringing from GPIO direction switch triggered `tryReceiveByte()` to capture a garbage byte before the real VTX response. Fixed by 1ms settling delay in `listen()`.
- SmartAudio v2/v2.1 `getSettings` response parsing: `version` byte at `buffer[4]` was being read as `channel`, causing VTX to operate on wrong frequency after `updateParameters()`.

### Removed
- Debug `SA_RESP:` hex dump from `sa_parseResponseBuffer()` (was used during ESP32 bring-up)
