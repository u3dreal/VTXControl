#ifndef ESP32HalfDuplex_h
#define ESP32HalfDuplex_h

#include <Arduino.h>
#include <inttypes.h>
#include <Stream.h>

#if !defined(ESP32)
#error "ESP32HalfDuplex is for ESP32 only"
#endif

#define ESP32HD_RX_BUF_SIZE 64

enum esp32hdErrors {
  esp32hdNoErrors = 0x00,
};

class ESP32HalfDuplex : public Stream {
private:
  int8_t _rxPin, _txPin;
  long _baud;
  uint16_t _config;
  uint8_t _bitTime;
  uint8_t _dataBits, _stopBits;
  uint8_t _parity;

  uint8_t _rxBuf[ESP32HD_RX_BUF_SIZE];
  volatile uint8_t _rxHead, _rxTail;
  volatile bool _overflow;

  bool _listening;
  bool _receiving;
  esp32hdErrors _errors;

  static void wait_us(unsigned long us) {
    unsigned long t = micros();
    while ((int32_t)(micros() - t) < (int32_t)us) {};
  }

  void sendByte(uint8_t b);
  bool tryReceiveByte();

public:
  ESP32HalfDuplex(uint8_t receivePin, uint8_t transmitPin,
    bool inverse_logic = false, bool full_duplex = true);
  ~ESP32HalfDuplex();

  void begin(long speed);
  void begin(long speed, uint16_t configuration);
  void setSpeed(long speed);
  bool listen();
  void end();
  bool isListening() { return _listening; }
  bool stopListening();
  bool overflow() { bool r = _overflow; _overflow = false; return r; }
  int peek();

  long getSpeed() { return _baud; }
  uint16_t getConfiguration() { return _config; }

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();

  void clearErrors();
  esp32hdErrors getErrors() { return _errors; }
  void writeDummyByte();
  void dumpReceiveBuffer();

  using Print::write;
};

#endif
