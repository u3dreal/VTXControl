/*
 * ESP32HalfDuplex.cpp - ESP32 bit-banged half-duplex serial for VTXControl
 *
 * Copyright (C) 2024 u3dreal <info@q3de.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <Arduino.h>
#include <driver/gpio.h>
#include "ESP32HalfDuplex.h"

static uint16_t parseConfig(uint16_t cfg, uint8_t &dataBits, uint8_t &stopBits, uint8_t &parity) {
  dataBits = cfg / 100;
  uint8_t r = cfg % 100;
  parity = r / 10;     // 0=None, 1=Odd, 2=Even
  stopBits = r % 10;   // 1 or 2
  if (dataBits < 5 || dataBits > 8) dataBits = 8;
  if (stopBits < 1 || stopBits > 2) stopBits = 2;
  if (parity > 2) parity = 0;
  return cfg;
}

ESP32HalfDuplex::ESP32HalfDuplex(uint8_t receivePin, uint8_t transmitPin,
    bool inverse_logic, bool full_duplex) {
  _rxPin = receivePin;
  _txPin = transmitPin;
  _baud = 0;
  _bitTime = 0;
  _config = 0;
  _dataBits = 8;
  _stopBits = 2;
  _parity = 0;
  _rxTail = 0;
  _rxHead = 0;
  _overflow = false;
  _listening = false;
  _receiving = false;
  _errors = esp32hdNoErrors;
}

ESP32HalfDuplex::~ESP32HalfDuplex() {
  end();
}

void ESP32HalfDuplex::begin(long speed) {
  _baud = speed;
  _bitTime = (uint8_t)(1000000L / speed);
  _config = 802; // 8N2 default
  parseConfig(_config, _dataBits, _stopBits, _parity);
  gpio_reset_pin((gpio_num_t)_txPin);
  gpio_set_pull_mode((gpio_num_t)_txPin, GPIO_PULLUP_ONLY);
  gpio_set_direction((gpio_num_t)_txPin, GPIO_MODE_INPUT);
  _listening = true;
}

void ESP32HalfDuplex::setSpeed(long speed) {
  _baud = speed;
  _bitTime = (uint8_t)(1000000L / speed);
}

void ESP32HalfDuplex::begin(long speed, uint16_t configuration) {
  _baud = speed;
  _bitTime = (uint8_t)(1000000L / speed);
  _config = configuration;
  parseConfig(_config, _dataBits, _stopBits, _parity);
  gpio_reset_pin((gpio_num_t)_txPin);
  gpio_set_pull_mode((gpio_num_t)_txPin, GPIO_PULLUP_ONLY);
  gpio_set_direction((gpio_num_t)_txPin, GPIO_MODE_INPUT);
  _listening = true;
}

bool ESP32HalfDuplex::listen() {
  _listening = true;
  gpio_set_direction((gpio_num_t)_txPin, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)_txPin, GPIO_PULLUP_ONLY);
  wait_us(1000);  // let line settle after TX
  return true;
}

void ESP32HalfDuplex::end() {
  _listening = false;
  gpio_reset_pin((gpio_num_t)_txPin);
}

bool ESP32HalfDuplex::stopListening() {
  _listening = false;
  return true;
}

// ----- TX -----

void ESP32HalfDuplex::sendByte(uint8_t b) {
  gpio_set_direction((gpio_num_t)_txPin, GPIO_MODE_OUTPUT);
  // Start bit (LOW)
  gpio_set_level((gpio_num_t)_txPin, 0);
  unsigned long deadline = micros() + _bitTime;
  while (micros() < deadline);
  // Data bits LSB first
  for (uint8_t i = 0; i < _dataBits; i++) {
    gpio_set_level((gpio_num_t)_txPin, (b >> i) & 1);
    deadline += _bitTime;
    while (micros() < deadline);
  }
  // Parity (if enabled) — not used by SmartAudio/Tramp but handled
  if (_parity) {
    uint8_t p = 0, t = b;
    for (uint8_t i = 0; i < _dataBits; i++) { p ^= (t & 1); t >>= 1; }
    if (_parity == 1) p = !p; // even parity: p is odd, flip
    gpio_set_level((gpio_num_t)_txPin, p);
    deadline += _bitTime;
    while (micros() < deadline);
  }
  // Stop bits (HIGH)
  for (uint8_t i = 0; i < _stopBits; i++) {
    gpio_set_level((gpio_num_t)_txPin, 1);
    deadline += _bitTime;
    while (micros() < deadline);
  }
  // Back to input (half-duplex)
  gpio_set_direction((gpio_num_t)_txPin, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)_txPin, GPIO_PULLUP_ONLY);
}

size_t ESP32HalfDuplex::write(uint8_t byte) {
  sendByte(byte);
  return 1;
}

void ESP32HalfDuplex::writeDummyByte() {
  sendByte(0x00);
}

// ----- RX -----

bool ESP32HalfDuplex::tryReceiveByte() {
  // Check for start bit: pin is LOW
  if (gpio_get_level((gpio_num_t)_txPin) != 0)
    return false;

  uint8_t data = 0;

  // Wait 1.5 bit times to center on first data bit
  unsigned long deadline = micros() + _bitTime + (_bitTime >> 1);
  while (micros() < deadline);

  // Sample data bits LSB first
  for (uint8_t i = 0; i < _dataBits; i++) {
    data |= ((uint8_t)gpio_get_level((gpio_num_t)_txPin) << i);
    deadline += _bitTime;
    while (micros() < deadline);
  }

  // Parity (if present) — consume but ignore for now
  if (_parity) {
    deadline += _bitTime;
    while (micros() < deadline);
  }

  // Stop bit(s) — consume
  deadline += _bitTime * _stopBits;
  while (micros() < deadline);

  // Store in circular buffer
  uint8_t next = (uint8_t)(_rxHead + 1) & (ESP32HD_RX_BUF_SIZE - 1);
  if (next == _rxTail) {
    _overflow = true;
    return true;
  }
  _rxBuf[_rxHead] = data;
  _rxHead = next;
  return true;
}

int ESP32HalfDuplex::available() {
  if (!_listening) return 0;
  _receiving = true;
  unsigned long t0 = micros();
  unsigned long startTime = t0;
  bool gotBytes = false;
  while (true) {
    if (tryReceiveByte()) {
      t0 = micros();
      gotBytes = true;
    }
    unsigned long elapsed = micros() - t0;
    if (gotBytes) {
      if (elapsed > 5000) break;
    } else {
      if (elapsed > 150000) break;
    }
    // total timeout failsafe — line may be stuck low
    if (micros() - startTime > 250000) break;
  }
  _receiving = false;
  uint8_t head = _rxHead;
  uint8_t tail = _rxTail;
  return (head >= tail) ? (head - tail) : (ESP32HD_RX_BUF_SIZE + head - tail);
}

int ESP32HalfDuplex::read() {
  if (_rxHead == _rxTail) return -1;
  uint8_t b = _rxBuf[_rxTail];
  _rxTail = (uint8_t)(_rxTail + 1) & (ESP32HD_RX_BUF_SIZE - 1);
  return b;
}

int ESP32HalfDuplex::peek() {
  if (_rxHead == _rxTail) return -1;
  return _rxBuf[_rxTail];
}

void ESP32HalfDuplex::flush() {
  _rxTail = 0;
  _rxHead = 0;
  _overflow = false;
  // Drain pending bytes by waiting for idle line
  unsigned long t = millis();
  unsigned long startTime = t;
  while (millis() - t < 10) {
    if (gpio_get_level((gpio_num_t)_txPin) == 0) {
      t = millis();
    }
    if (millis() - startTime > 20) break;
  }
}

// ----- Error / debug -----

void ESP32HalfDuplex::clearErrors() {
  _errors = esp32hdNoErrors;
}

#ifdef ESP32HD_DEBUG
void ESP32HalfDuplex::dumpReceiveBuffer() {
  Serial.print("ESP32HD RX:");
  uint8_t t = _rxTail;
  while (t != _rxHead) {
    Serial.printf(" %02X", _rxBuf[t]);
    t = (uint8_t)(t + 1) & (ESP32HD_RX_BUF_SIZE - 1);
  }
  Serial.println();
}
#else
void ESP32HalfDuplex::dumpReceiveBuffer() {}
#endif
