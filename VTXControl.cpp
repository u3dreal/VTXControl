#include <Arduino.h>
#if !defined(ESP32)
#include <util/delay.h>
#endif
#include "VTXControl.h"
#include "VTX_SmartAudio.h"
#include "VTX_Tramp.h"
#if defined(ESP32)
#include "ESP32HalfDuplex.h"
#else
#include "SoftwareSerialWithHalfDuplex.h"
#endif

// these parameters for JHEMCU RuiBet Tran3016W
// uint16_t powers[5] = { 25, 200, 400, 800, 1600 };//in mW
const uint16_t powers_v1[4] = {7 /*25mw*/, 16 /*200mw*/, 25 /*500mw*/, 40 /*800mw*/}; // for SmartAudio protocol v1
// Smartaudio v2.1 protocol seems to be not documented (TBS documented just v1 and v2), but defined in ArduPilot
uint16_t powers_v21[4] = {14, 20, 26, 46}; // dBm: 25mW, 100mW, 400mW, 5W

static uint16_t dbmToMw(uint8_t dbm); // forward declaration

// Instantiates the VTX object with the specified parameters
VTXControl::VTXControl(
    int vtxMode,
    int softPin,
    const uint16_t powers[],
    int powers_len,
    const uint16_t freqs[],
    int freqs_len,
    int responseTimeOut,
    bool smartBaudRate,
    int numtries)
{
  DEBUG("VTXControl: Create");
  vtx_mode = vtxMode;
#if defined(ESP32)
  port = new ESP32HalfDuplex(softPin, softPin, false, false);
#else
  port = new SoftwareSerialWithHalfDuplex(softPin, softPin, false, false);
#endif
  _responseTimeOut = responseTimeOut;
  _numtries = numtries;
  _smartBaudRate = smartBaudRate;
  _powers = powers;
  _power_size = powers_len;
  _freqs = freqs;
  _freqs_size = freqs_len;

  // Initialize detected power table from the original values
  for (int i = 0; i < 4 && i < powers_len; i++) {
    _detectedPowersMw[i] = powers[i];
  }

  // with SmartAudio protocol according to TBS documentation
  //!!! Please do remember, for SmartAudio - logic level = 3.3V !!!
  // we use 4800bps 1 Start bit and 2 Stop bit, 8 data bits
  // for SmartAudio clones(wthiout TBS license) - 1 start bit, 2 stop bit, 8 data bits
  // with Tramp - 9600bps, 1 start bit, 1 stop bit, 8 data bits
  port->begin(vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_BAUD : AP_TRAMP_UART_BAUD,
              vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_CFG : AP_TRAMP_UART_CFG);
  waitForInMs(200);
}
void VTXControl::flush()
{
  port->flush();
}
void VTXControl::clearErrors()
{
  errors = VTXErrors::vtxNoErrors;
  port->clearErrors();
}
long VTXControl::getSpeed()
{
  return port->getSpeed();
}
VTXErrors VTXControl::getErrors()
{
#if !defined(ESP32)
  sswhdErrors err = port->getErrors();
  if (err != sswhdErrors::sswhdNoErrors)
  {
    if ((err & sswhdErrors::sswhdIsNotListening) != 0)
      errors = static_cast<VTXErrors>(static_cast<long>(errors) | static_cast<long>(vtxportIsNotListening));
    if ((err & sswhdErrors::sswhdBufferIsEmpty) != 0)
      errors = static_cast<VTXErrors>(static_cast<long>(errors) | static_cast<long>(vtxportBufferIsEmpty));
    if ((err & sswhdErrors::sswhdTxDelayIsZero) != 0)
      errors = static_cast<VTXErrors>(static_cast<long>(errors) | static_cast<long>(vtxportTxDelayIsZero));
    if ((err & sswhdErrors::sswhdRXDelayStopBitNotSet) != 0)
      errors = static_cast<VTXErrors>(static_cast<long>(errors) | static_cast<long>(vtxportRXDelayStopBitNotSet));
  }
#endif
  return errors;
}
void VTXControl::setError(VTXErrors error)
{
  errors = static_cast<VTXErrors>(static_cast<long>(errors) | static_cast<long>(error));
}
bool VTXControl::sa_updateSettings()
{
  long startspeed = port->getSpeed();
  long newspeed = startspeed;
  DEBUG("Smart Audio Update_settings");
  while (1) // speed cycle
  {
    for (int i = 0; i < _numtries; i++)
    {
      if (sa_getSettings())
      {
        if (sa_readResponse())
          return true; // in case of success we return true
      }
      port->flush();
    }
    if (_smartBaudRate)
    {
      // so we need to change baud rate?
      newspeed = sa_offerNewSpeed(newspeed);
      if (newspeed != startspeed)
      {
        DEBUG("update_settings, trying new speed:" + (String)newspeed);
        port->begin(newspeed, port->getConfiguration());
      }
      else
      {
        DEBUG("update_settings, end of tries");
        return false; // out of change speed cycle
      }
    }
    else
      break;
  }
  return false;
}
long VTXControl::sa_offerNewSpeed(long currentSpeed)
{
  // if speed == AP_SMARTAUDIO_UART_BAUD we're going to AP_SMARTAUDIO_SMARTBAUD_MAX by increasing step by step to AP_SMARTAUDIO_SMARTBAUD_MAX
  // if we achieved AP_SMARTAUDIO_SMARTBAUD_MAX we're going to AP_SMARTAUDIO_SMARTBAUD_MIN and then increasing to AP_SMARTAUDIO_UART_BAUD
  // so AP_SMARTAUDIO_UART_BAUD->AP_SMARTAUDIO_SMARTBAUD_MAX->AP_SMARTAUDIO_SMARTBAUD_MIN->AP_SMARTAUDIO_UART_BAUD(terminator)
  switch (currentSpeed)
  {
  case AP_SMARTAUDIO_UART_BAUD:
    return AP_SMARTAUDIO_UART_BAUD + AP_SMARTAUDIO_SMARTBAUD_STEP;
  case AP_SMARTAUDIO_SMARTBAUD_MIN:
    return AP_SMARTAUDIO_SMARTBAUD_MIN + AP_SMARTAUDIO_SMARTBAUD_STEP;
  case AP_SMARTAUDIO_SMARTBAUD_MAX:
    return AP_SMARTAUDIO_SMARTBAUD_MIN;
  }
  // if speed is in AP_SMARTAUDIO_UART_BAUD-AP_SMARTAUDIO_SMARTBAUD_MAX or AP_SMARTAUDIO_SMARTBAUD_MIN-AP_SMARTAUDIO_UART_BAUD
  //  ranges - we just increase baud rate step-by-step
  if ((currentSpeed > AP_SMARTAUDIO_UART_BAUD && currentSpeed < AP_SMARTAUDIO_SMARTBAUD_MAX) ||
      (currentSpeed > AP_SMARTAUDIO_SMARTBAUD_MIN && currentSpeed < AP_SMARTAUDIO_UART_BAUD))
    return currentSpeed + AP_SMARTAUDIO_SMARTBAUD_STEP;
  if (currentSpeed > AP_SMARTAUDIO_SMARTBAUD_MAX)
    return AP_SMARTAUDIO_SMARTBAUD_MAX;
  if (currentSpeed < AP_SMARTAUDIO_SMARTBAUD_MIN)
    return AP_SMARTAUDIO_SMARTBAUD_MIN;
  return currentSpeed;
}
#if VTXCDEBUG
bool VTXControl::testSMAWrite()
{
  return sa_getSettings();
}
bool VTXControl::testSMAResponseFromSerial1()
{
  SettingsResponseFrame response;
  response.header.init(SMARTAUDIO_RSP_GET_SETTINGS_V1, 5);
  response.channel = 2;
  response.power = 25;           // v1
  response.operationMode = 0x04; // pitmode turned on
  response.frequency = 5800;
  DEBUG("push response (Serial1):" + (String)sizeof(SettingsResponseFrame));
  bool res = Serial1.write((uint8_t *)&response, sizeof(SettingsResponseFrame)) == sizeof(SettingsResponseFrame);
  // Packet command;
  //// according to the spec the length should include the CRC, but no implementation appears to
  //// do this
  // command.frame.header.init(SMARTAUDIO_CMD_GET_SETTINGS, 0);
  // command.frame_size = SMARTAUDIO_COMMAND_FRAME_SIZE;
  // command.frame.payload[0] = crc8_dvb_s2_update(0, &command.frame, SMARTAUDIO_COMMAND_FRAME_SIZE - 1);
  // DEBUG("push to write:" + (String)sizeof(Packet));
  // bool res = Serial1.write((uint8_t*)&command, sizeof(Packet)) == sizeof(Packet);

  return res;
}
#endif
bool VTXControl::sa_setPitMode(bool enabled)
{
  static uint8_t buf[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_MODE, 1, 0x00, 0x00};
  buf[4] = enabled ? 0x01 : 0x00; // bit 0 = pit mode toggle (per Betaflight convention, confirmed working)
  buf[5] = sa_CRC8(buf, 5);
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
  if (_trailingZero) {
    port->write((uint8_t)0x00);
  }
  port->listen();
  return res;
}
bool VTXControl::sa_setPower(int pwrLevel)
{
  static uint8_t buf[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_POWER, 1, 0x00, 0x00};
  switch (sa_protocol_version)
  {
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v1:
    // res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, powers_v1[pwrLevel]);
    buf[4] = powers_v1[pwrLevel];
    break;
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v2:
    // debug("Setting power to %d", power_level);
    // res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, pwrLevel);
    buf[4] = pwrLevel;
    break;
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v21:
    // res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, powers_v21[pwrLevel] | 0x80);
    buf[4] = powers_v21[pwrLevel] | 0x80;
    break;
  }
  buf[5] = sa_CRC8(buf, 5);

  DEBUG("sa_setPower, push to write:" + (String)sizeof(buf));
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
  if (_trailingZero) {
    port->write((uint8_t)0x00);
  }
  port->listen();

  return res;
}
bool VTXControl::sa_setChannel(uint8_t channel)
{
  static uint8_t buf[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_CHANNEL, 1, 0, 0};
  buf[4] = channel;
  buf[5] = sa_CRC8(buf, 5); // exclude crc byte
  DEBUG("sa_setChannel, push to write:" + (String)sizeof(buf));
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
  if (_trailingZero) {
    port->write((uint8_t)0x00);
  }
  port->listen();

  return res;
}
bool VTXControl::sa_setFrequency(uint16_t freq)
{
  static uint8_t buf[7] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_FREQUENCY, 2, 0, 0, 0};
  buf[4] = (freq >> 8) & 0xFF;
  buf[5] = freq & 0xFF;
  buf[6] = sa_CRC8(buf, 6);
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
  if (_trailingZero) {
    port->write((uint8_t)0x00);
  }
  port->listen();
  return res;
}

bool VTXControl::sa_getSettings()
{
  // taken from betaflight vtx_smartaudio.c
  static uint8_t buf[5] = {0xAA, 0x55, SMARTAUDIO_CMD_GET_SETTINGS, 0x00, 0x9F};
  DEBUG("sa_GetSettings, push to write:" + (String)sizeof(buf));
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.
  port->writeDummyByte();
  // port->write((uint8_t)0x00);
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
  // port->writeDummyByte();
  if (_trailingZero) {
    port->write((uint8_t)0x00);
  }
  // Don't flush here — flush waits for line idle, but the VTX may
  // start responding immediately and those bytes would be missed.
  // The scan loop in sa_readResponse handles any stale buffer data.
  port->listen();
  return res;
}

// relatively precise function as replacement of delay
// delay stop processor/interrupts usage, so we need to use delayMcroseconds
// but delayMicroseconds uses max 16384 value
void VTXControl::waitForInMs(unsigned int ms)
{
  int in_tens_ms = ms / 10;
  int reminder_ms = ms - (in_tens_ms * 10);
  // tens of ms
  for (int i = 0; i < in_tens_ms; i++)
  {
    delayMicroseconds(10000);
  }
  // reminded ms
  for (int i = 0; i < reminder_ms; i++)
  {
    delayMicroseconds(1000);
  }
  // DEBUG("VTXControl::waitForInMs:" + (String)ms + "ms End");
}
bool VTXControl::sa_readResponse()
{
  // Don't flush here — flush waits for line idle, which can miss
  // the VTX response bytes.  Stale buffer data from previous TX echo
  // is harmless (the scan loop skips noise and looks for 0xAA 0x55).
  uint8_t buf[AP_SMARTAUDIO_MAX_PACKET_SIZE];
  int pos = 0, dataLen = 0, dataRead = 0;
  int availCount = port->available();

  while (availCount-- > 0) {
    uint8_t b = port->read();
    if (b == SMARTAUDIO_SYNC_BYTE && port->peek() == SMARTAUDIO_HEADER_BYTE) {
      buf[pos++] = b;
      port->read(); // consume 0x55
      buf[pos++] = SMARTAUDIO_HEADER_BYTE;
      availCount--;
      if (pos > 1) while (availCount > 0 && pos < (int)sizeof(buf) - 1) {
        b = port->read();
        availCount--;
        buf[pos++] = b;
        if (pos == 4) {
          dataLen = buf[3];
          if (dataLen >= (int)sizeof(buf) - 5) break;
          dataRead = 0;
        }
        if (pos > 4 && ++dataRead > dataLen) {
          bool correct = sa_parseResponseBuffer(buf);
          port->flush();
          waitForInMs(100);
          DEBUG("readResponse end, correct_parse = " + (String)correct);
          return correct;
        }
      }
    }
  }
  DEBUG("readResponse, timeout");
  setError(VTXErrors::vtxIncomingBytesZero);
  return false;
}
bool VTXControl::sa_parseResponseBuffer(const uint8_t *buffer)
{
  const FrameHeader *header = (const FrameHeader *)buffer;
  const uint8_t fullFrameLength = sizeof(FrameHeader) + header->length;
  const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
  const uint8_t *startPtr = buffer + 2;                    // exclude header and sync bytes
  const uint8_t *endPtr = buffer + headerPayloadLength;
  DEBUG("sa_parse_response_buffer(), fullFrameLength=" + (String)fullFrameLength);
  if (!sa_ignoreCrc())
  {
#if VTXCDEBUG
    dumpBuffer(buffer, headerPayloadLength);
#endif
    uint8_t crc = sa_CRC8(buffer, headerPayloadLength);
    // uint8_t crc = sa_CRC8(startPtr, headerPayloadLength-2);
    uint8_t crc_resp = *(endPtr);
    if (crc != crc_resp)
    {
      DEBUG("sa_parse_response_buffer() failed - invalid CRC or header, crc in resp=" + (String)crc_resp + ", crc calc=" + (String)crc);
      setError(VTXErrors::vtxParseResponseInvalidCRCOrBuffer);
      return false;
    }
  }
  if (header->headerByte != SMARTAUDIO_HEADER_BYTE)
  {
    setError(VTXErrors::vtxIncomingByteNotEqualHeaderByte);
    return false;
  }
  if (header->syncByte != SMARTAUDIO_SYNC_BYTE)
  {
    setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
    return false;
  }
  switch (header->command)
  {
  case SMARTAUDIO_RSP_GET_SETTINGS_V1:
  {
    DEBUG("sa_parse_response_buffer(), Protocol version 1");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v1;
    const SettingsResponseFrame *resp = (const SettingsResponseFrame *)buffer;
    pwr_Level = getPowerIndexFromV1(resp->power);
    ch_index = resp->channel;
    pitMode = (resp->operationMode & 0x02) != 0; // TBS spec: Bit 1 = PitMode Running
    _frequency = ((uint16_t)buffer[7] << 8) | buffer[8]; // big-endian
  }
  break;
  case SMARTAUDIO_RSP_GET_SETTINGS_V2:
  {
    DEBUG("sa_parse_response_buffer(), Protocol version 2");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v2;
    // TBS SmartAudio V2 response layout:
    //   0    1    2    3    4    5    6    7    8    9
    //  sync hdr  cmd  len  CH   PW   MODE FREQ FREQ CRC
    //
    // TX805S clone: power in lower nibble of buffer[5]
    if (_powerInLowerNibble) {
      ch_index = buffer[4];
      pwr_Level = buffer[5] & 0x0F;
      pitMode = (buffer[6] & 0x02) != 0;
    } else {
      ch_index = buffer[4];
      pwr_Level = buffer[5];
      pitMode = (buffer[6] & 0x02) != 0;
    }

    uint16_t freqMHz = (((uint16_t)buffer[7] << 8) | buffer[8]) & 0x3FFF;
    _frequency = freqMHz;

#if VTXCDEBUG
    Serial.printf("VTX: 0x%02X 0x%02X 0x%02X (Version/Command) 0x%02X (Length) 0x%02X (Channel) 0x%02X (Power Level) 0x%02X (Operation Mode) 0x%02X 0x%02X (Current Frequency %u) 0x%02X (CRC8)\n",
      buffer[0], buffer[1], buffer[2], buffer[3],
      buffer[4], buffer[5], buffer[6],
      buffer[7], buffer[8], freqMHz, buffer[9]);
#endif
  }
  break;

  case SMARTAUDIO_RSP_GET_SETTINGS_V21:
  {
    DEBUG("sa_parse_response_buffer(), Protocol version 2.1");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v21;
    // TBS SmartAudio V2.1 response layout:
    //   0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
    //  sync hdr  cmd  len  CH   PW   MODE FREQ FREQ dBm  nLv  L0   L1   L2   L3   CRC
    //                       [4]  [5]  [6]  [7]  [8]  [9]  [10] [11] [12] [13] [14] [15]
    ch_index = buffer[4];
    pitMode = (buffer[6] & 0x02) != 0; // TBS spec: Bit 1 = PitMode Running

    // Extract L0-L3 dBm table from VTX response (buffer[11..14])
    for (int i = 0; i < 4 && (i + 11) < (int)headerPayloadLength; i++) {
      uint8_t dBm = buffer[11 + i] & 0x7F; // strip MSB (flag bit)
      powers_v21[i] = dBm;
      _detectedPowersMw[i] = dbmToMw(dBm);
    }
    // Now compute pwr_Level using the updated powers_v21 table
    pwr_Level = getPowerIndexFromDbm(buffer[9]);

    uint16_t rawFreq = ((uint16_t)buffer[7] << 8) | buffer[8];
    bool freqBit15 = (rawFreq >> 15) & 1;
    bool freqBit14 = (rawFreq >> 14) & 1;
    uint16_t freqMHz = rawFreq & 0x3FFF;
    _frequency = freqMHz;
  }
  break;
  case SMARTAUDIO_RSP_SET_POWER:
  {

    const U16ResponseFrame *respu16 = (const U16ResponseFrame *)buffer;
    const uint8_t power = respu16->payload & 0xFF;
    switch (sa_protocol_version)
    {
    case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v21:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 2.1");
      pwr_Level = getPowerIndexFromDbm(power);
      break;
    case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v1:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 1");
      pwr_Level = getPowerIndexFromV1(power);
      break;
    default:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 2");
      pwr_Level = _powerInLowerNibble ? (power & 0x0F) : power;
      break;
    }
  }
  break;
  case SMARTAUDIO_RSP_SET_CHANNEL:
  {
    DEBUG("sa_parse_response_buffer(), SetChannel");
    const U8ResponseFrame *respu8 = (const U8ResponseFrame *)buffer;
    uint16_t channel = respu8->payload;
    ch_index = channel;
    // debug("Channel was set to %d", resp->payload);
  }
  break;
  case SMARTAUDIO_RSP_SET_FREQUENCY:
  {
    DEBUG("sa_parse_response_buffer(), SetFrequency");
    uint16_t freq = ((uint16_t)buffer[4] << 8) | buffer[5]; // big-endian on wire
    freq &= SMARTAUDIO_FREQUENCY_MASK;
    ch_index = getChannelIndex(freq);
  }
  break;
  case SMARTAUDIO_RSP_SET_MODE:
  {
    DEBUG("sa_parse_response_buffer(), SetMode");
    const U8ResponseFrame *respu8 = (const U8ResponseFrame *)buffer;
    pitMode = (respu8->payload & 0x02) != 0; // bit 1 = pit mode (TBS V2/V2.1 spec)
  }
  break;
  }
  return true;
}

bool VTXControl::setPitMode(bool enabled)
{
  clearErrors();
  for (int i = 0; i < _numtries; i++)
  {
    bool res = false;
    switch (vtx_mode)
    {
    case VTXMode::SmartAudio:
      if (sa_setPitMode(enabled))
      {
        res = sa_readResponse();
      }
      break;
    case VTXMode::Tramp:
      // trampSetPitMode not implemented
      break;
    }
    if (res)
    {
      pitMode = enabled;
      return true;
    }
  }
  return false;
}

bool VTXControl::setPowerInmW(uint16_t pwrmW)
{
  bool res = false;
  clearErrors();
  for (int i = 0; i < _numtries; i++) // trying to set _numtries times
  {
    switch (vtx_mode)
    {
    case VTXMode::SmartAudio:
    {
      int pwrLevel = getPowerIndexFromMW(pwrmW);
      if (pwrLevel != -1) // pwr level(index) not found
      {
        if (sa_setPower(pwrLevel))
        {
          if (sa_readResponse())
          {
            res = true;
          }
        }
      }
    }
    break;
    case VTXMode::Tramp: // tramp protocol sets power in mW
    {
      // tramp protocol needs to be initialized
      if (!initialized)
      {
        trampInit();
      }
      // in milliWatts
      if (initialized)
      {
        res = trampSetPower(pwrmW);
      }
    }
    break;
    }
    // if procedure of setting is succesfull - check acvieved result
    if (res)
    {
      int currPwrmW = getPowerInmW(pwr_Level);
      if (currPwrmW == pwrmW)
        return true;
    }
  }
  return false;
}
bool VTXControl::setPower(int pwrLevel)
{
  if (pwrLevel >= 0 && pwrLevel < _power_size) // check the index of power
  {
    bool res = false;
    clearErrors();
    for (int i = 0; i < _numtries; i++) // trying to set _numtries times
    {
      switch (vtx_mode)
      {
      case VTXMode::SmartAudio:
      {
        if (sa_setPower(pwrLevel))
        {
          if (sa_readResponse())
          {
            res = true;
          }
        }
      }
      break;
      case VTXMode::Tramp: // tramp protocol sets power in mW
      {
        // tramp protocol needs to be initialized
        if (!initialized)
        {
          trampInit();
        }
        // in milliWatts
        if (initialized)
          res = trampSetPower(_powers[pwrLevel]);
      }
      break;
      }
      // if procedure of setting is succesfull - check acvieved result
      if (res && pwr_Level == pwrLevel)
      {
        return true;
      }
    }
  }
  return false;
}
bool VTXControl::setPrevChannel()
{
  int newCh = ch_index - 1;
  if (newCh < 0)
    newCh = _freqs_size - 1;
  return setChannel(newCh);
}
bool VTXControl::setNextChannel()
{
  int newCh = ch_index + 1;
  if (newCh >= _freqs_size)
    newCh = 0;
  return setChannel(newCh);
}
uint16_t VTXControl::getPowerInmW(int pwrIndex)
{
  int pwrslen = _power_size;
  return (pwrIndex >= 0 && pwrIndex < pwrslen) ? _powers[pwrIndex] : 0;
}
uint16_t VTXControl::getPowerMw(int level)
{
  if (level < 0 || level >= 4) return 0;
  // For V2.1, use the detected table from VTX response (L0-L3 dBm values)
  if (sa_protocol_version == SMARTAUDIO_SPEC_PROTOCOL_v21) {
    return _detectedPowersMw[level];
  }
  // Fallback to the original power table for V1/V2
  return getPowerInmW(level);
}
// dBm → mW: 10^(dbm/10)
static uint16_t dbmToMw(uint8_t dbm) {
  float mw = powf(10.0f, dbm / 10.0f);
  return (uint16_t)(mw + 0.5f);
}
int VTXControl::getPowerIndexFromMW(uint16_t pwrInmW)
{
  int pwrslen = _power_size;
  for (int i = 0; i < pwrslen; i++)
  {
    if (_powers[i] == pwrInmW)
      return i;
  }
  return -1; // not found
}
int VTXControl::getPowerIndexFromDbm(uint16_t pwrInDbm)
{
  int pwrslen = sizeof(powers_v21) / sizeof(powers_v21[0]);
  for (int i = 0; i < pwrslen; i++)
  {
    if (powers_v21[i] == pwrInDbm)
      return i;
  }
  return -1; // not found
}
int VTXControl::getPowerIndexFromV1(uint16_t pwrValue)
{
  int pwrslen = sizeof(powers_v1) / sizeof(powers_v1[0]);
  for (int i = 0; i < pwrslen; i++)
  {
    if (powers_v1[i] == pwrValue)
      return i;
  }
  return -1; // not found
}
uint16_t VTXControl::getChannelFrequency(int chIndex)
{
  int freqslen = _freqs_size;
  return (chIndex >= 0 && chIndex < freqslen) ? _freqs[chIndex] : 0;
}
int VTXControl::getChannelIndex(uint16_t freq)
{
  int freqslen = _freqs_size;
  for (int i = 0; i < freqslen; i++)
  {
    if (_freqs[i] == freq)
      return i;
  }
  return -1; // not found
}

bool VTXControl::setFrequency(uint16_t freq)
{
  clearErrors();
  bool res = false;
  for (int i = 0; i < _numtries; i++)
  {
    switch (vtx_mode)
    {
    case VTXMode::SmartAudio:
    {
      if (sa_setFrequency(freq))
      {
        res = sa_readResponse();
      }
    }
    break;
    case VTXMode::Tramp: // tramp protocol sets channel in Mhz
    {
      if (!initialized)
      {
        trampInit();
      }
      if (initialized)
        res = trampSetFrequency(freq);
    }
    break;
    }
    if (res)
      return true;
  }
  return false;
}
// Sets the specified frequency to the VTX
bool VTXControl::setChannel(int chIndex)
{
  if (chIndex >= 0 && chIndex < _freqs_size)
  {
    clearErrors();
    bool res = false;
    for (int i = 0; i < _numtries; i++) // trying to set _numtries times
    {
      switch (vtx_mode)
      {
      case VTXMode::SmartAudio:
      {
        // debug("Setting channel to %d", channel);
        // if (push_uint8_command_frame(SMARTAUDIO_CMD_SET_CHANNEL, chIndex))
        if (sa_setChannel(chIndex))
        {
          res = sa_readResponse();
        }
      }
      break;
      case VTXMode::Tramp: // tramp protocol sets channel in Mhz
      {
        // tramp protocol needs to be initialized
        if (!initialized)
        {
          trampInit();
        }
        if (initialized)
          res = trampSetFrequency(_freqs[chIndex]);
      }
      break;
      }
      // if channel set succesfully - check the updated info from VTX
      if (res && chIndex == ch_index)
        return true;
    }
  }
  return false;
}

bool VTXControl::updateParameters()
{
  // port->stopListening();
  switch (vtx_mode)
  {
  case VTXMode::SmartAudio:
    // DEBUG("updateParameters: SMA"); waitForInMs(200);
    return sa_updateSettings();
  case VTXMode::Tramp: // tramp protocol sets channel in Mhz
    // DEBUG("updateParameters: Tramp"); waitForInMs(200);
    return trampUpdate();
  }
  return false;
}

bool VTXControl::trampSendPacket(uint8_t *packet, bool respRequired) //(trampFrame_t* packet)
{
  // DEBUG("trampSendPacket, before trampPush");
  if (trampPush(packet))
  {
    // with setPower and setChannel we don't wait a response,
    // in this case we send UpdateParameters(GetConfig) to get new parameters
    bool res = respRequired ? trampReadResponse() : true;
    DEBUG("trampSendPacket, After ReadResponse, res=" + (String)res);
    return res;
  }
  DEBUG("trampSendPacket, return false");
  return false;
}
bool VTXControl::trampPush(const uint8_t *packet) // const trampFrame_t* object)
{
  // port->writeDummyByte();//to get port low before sending command
  // port->write((uint8_t)0x00);
  // bool res = port->write((uint8_t*)&object, sizeof(trampFrame_t)) == sizeof(trampFrame_t);
  bool res = port->write(packet, TRAMP_FRAME_LENGTH) == TRAMP_FRAME_LENGTH;
  // port->write((uint8_t)0x00);

  port->listen(); // wait for the response
  // DEBUG("trampPush, written "+(String)sizeof(trampFrame_t) +" bytes");
  DEBUG("trampPush, written " + (String)TRAMP_FRAME_LENGTH + " bytes");
  DEBUG("trampPush, Res= " + (String)res);
#if VTXCDEBUG
  // dumpBuffer((uint8_t*)&object, sizeof(trampFrame_t));
  dumpBuffer(packet, TRAMP_FRAME_LENGTH);
#endif
  return res;
}

bool VTXControl::trampUpdate()
{
  long startspeed = port->getSpeed();
  long newspeed = startspeed;
  DEBUG("TrampUpdate");
  while (1) // speed cycle
  {
    // trying several times
    for (int i = 0; i < _numtries; i++)
    {
      // tramp protocol needs to be initialized
      if (!initialized)
      {
        trampInit();
        DEBUG("TrampUpdate, Initialized:" + (String)initialized);
      }
      if (initialized)
      {
        if (trampGetStatus())
        {
          return true;
        }
      }
    }
    if (_smartBaudRate)
    {
      // so we need to change baud rate?
      newspeed = trampOfferNewSpeed(newspeed);
      if (newspeed != startspeed)
      {
        DEBUG("TrampUpdate, trying new speed:" + (String)newspeed);
        port->begin(newspeed, port->getConfiguration());
      }
      else
      {
        DEBUG("TrampUpdate, end of tries");
        return false; // out of change speed cycle
      }
    }
    else
      break;
  }
  return false;
}
long VTXControl::trampOfferNewSpeed(long currentSpeed)
{
  // if speed == AP_SMARTAUDIO_UART_BAUD we're going to AP_SMARTAUDIO_SMARTBAUD_MAX by increasing step by step to AP_SMARTAUDIO_SMARTBAUD_MAX
  // if we achieved AP_SMARTAUDIO_SMARTBAUD_MAX we're going to AP_SMARTAUDIO_SMARTBAUD_MIN and then increasing to AP_SMARTAUDIO_UART_BAUD
  // so AP_SMARTAUDIO_UART_BAUD->AP_SMARTAUDIO_SMARTBAUD_MAX->AP_SMARTAUDIO_SMARTBAUD_MIN->AP_SMARTAUDIO_UART_BAUD(terminator)
  switch (currentSpeed)
  {
  case AP_TRAMP_UART_BAUD:
    return AP_TRAMP_UART_BAUD + AP_TRAMP_SMARTBAUD_STEP;
  case AP_TRAMP_UART_BAUD_MIN:
    return AP_TRAMP_UART_BAUD_MIN + AP_TRAMP_SMARTBAUD_STEP;
  case AP_TRAMP_UART_BAUD_MAX:
    return AP_TRAMP_UART_BAUD_MIN;
  }
  // if speed is in AP_SMARTAUDIO_UART_BAUD-AP_SMARTAUDIO_SMARTBAUD_MAX or AP_SMARTAUDIO_SMARTBAUD_MIN-AP_SMARTAUDIO_UART_BAUD
  //  ranges - we just increase baud rate step-by-step
  if ((currentSpeed > AP_TRAMP_UART_BAUD && currentSpeed < AP_TRAMP_UART_BAUD_MAX) ||
      (currentSpeed > AP_TRAMP_UART_BAUD_MIN && currentSpeed < AP_TRAMP_UART_BAUD))
    return currentSpeed + AP_TRAMP_SMARTBAUD_STEP;
  if (currentSpeed > AP_TRAMP_UART_BAUD_MAX)
    return AP_TRAMP_UART_BAUD_MAX;
  if (currentSpeed < AP_TRAMP_UART_BAUD_MIN)
    return AP_TRAMP_UART_BAUD_MIN;
  return currentSpeed;
}
bool VTXControl::trampSendCmd(uint8_t cmd)
{
  /*uint8_t buf[TRAMP_FRAME_LENGTH];
  memset(buf, 0, TRAMP_FRAME_LENGTH);
  buf[0] = TRAMP_SYNC_START;
  buf[1] = cmd;
  buf[14] = trampCrc(buf);
  buf[15] = TRAMP_SYNC_STOP;
  bool res = trampSendPacket(buf);*/
  return trampSendCmd(cmd, 0);
}
bool VTXControl::trampSendCmd(uint8_t cmd, uint16_t param)
{
  uint8_t buf[TRAMP_FRAME_LENGTH];
  memset(buf, 0, TRAMP_FRAME_LENGTH);
  buf[0] = TRAMP_SYNC_START;
  buf[1] = cmd;
  if (param != 0)
  {
    buf[2] = param & 0xff;
    buf[3] = (param >> 8) & 0xff;
  }
  buf[14] = trampCrc(buf);
  buf[15] = TRAMP_SYNC_STOP;
  // we need a reponse for certain commands only
  // so calling UpdateParameters required after send command
  bool respRequired = cmd == TRAMP_COMMAND_GET_CONFIG || cmd == TRAMP_COMMAND_CMD_RF || TRAMP_COMMAND_CMD_SENSOR;
  return trampSendPacket(buf, respRequired);
}
bool VTXControl::trampInit()
{
  DEBUG("TrampInit:");
  /* trampFrame_t frame;
   trampFrameInit(TRAMP_COMMAND_CMD_RF, &frame);
   trampFrameClose(&frame);
   bool res = trampSendPacket((uint8_t*)&frame);*/

  // different version of sending
  if (trampSendCmd(TRAMP_COMMAND_CMD_RF))
  {
    DEBUG("TrampInit: Inititalized Successfully");
    initialized = true;
    return true;
  }
  DEBUG("TrampInit: Not Inititalized");
  setError(VTXErrors::vtxtrampNotInited);
  return false;
}
bool VTXControl::trampGetStatus()
{
  // trampFrame_t frame;
  // trampFrameInit(TRAMP_COMMAND_GET_CONFIG, &frame);
  // trampFrameClose(&frame);
  // return trampSendPacket((uint8_t*)&frame);
  // different version of sending
  bool res = trampSendCmd(TRAMP_COMMAND_GET_CONFIG);
  DEBUG("trampGetStatus, res=" + (String)res);
  return res;
}

bool VTXControl::trampSetFrequency(uint16_t freq)
{
  // trampFrame_t frame;
  // trampFrameInit(TRAMP_COMMAND_SET_FREQ, &frame);
  // frame.payload.frequency = freq;
  // trampFrameClose(&frame);
  // return trampSendPacket((uint8_t*)&frame);
  return trampSendCmd(TRAMP_COMMAND_SET_FREQ, freq);
}
bool VTXControl::trampSetPower(uint16_t milliWatts)
{
  // trampFrame_t frame;
  // trampFrameInit(TRAMP_COMMAND_SET_POWER, &frame);
  // frame.payload.power = milliWatts;
  // trampFrameClose(&frame);
  // return trampSendPacket((uint8_t*)&frame);
  return trampSendCmd(TRAMP_COMMAND_SET_POWER, milliWatts);
}

bool VTXControl::trampReadResponse()
{
  // On my Unify Pro32 the SmartAudio response is sent exactly 100ms after the request
  // and the initial response is 40ms long so we should wait at least 140ms before giving up
  // waitForInMs(_responseTimeOut);
  // unsigned long currentTime = millis();
  ////wait to receive full packet
  unsigned long currentTime = 0;
  while (port->available() == 0)
  {
    if (currentTime > _responseTimeOut)
    {
      // DEBUG("trampReadResponse, time out of waiting response");
      break; // return false;
    }
#if defined(ESP32)
    delay(10);
    currentTime += 10;
#else
    waitForInMs(100);
    currentTime += 100;
#endif
  }

  // DEBUG("trampReadResponse, before fill rx buf");
  int16_t incoming_bytes_count = port->available();
  if (incoming_bytes_count == 0)
  {
    DEBUG("trampReadResponse, incoming bytes=0, return false");
    setError(VTXErrors::vtxIncomingBytesZero);
    return false;
  }
  // now we have no-zero length response, we can dump it
#if VTXCDEBUG
  DEBUG("trampReadResponse: Dump Received:");
  port->dumpReceiveBuffer();
#endif
  if (incoming_bytes_count == TRAMP_FRAME_LENGTH)
  {
    uint8_t tramp_rx_buf[TRAMP_FRAME_LENGTH];
    // fill out the buffer
    for (int i = 0; i < TRAMP_FRAME_LENGTH; i++)
    {
      uint8_t d = port->read();
      tramp_rx_buf[i] = d;
    }
    // Buffer is full, calculate checksum
    const trampFrame_t *frame = (const trampFrame_t *)tramp_rx_buf;
    if (frame->header.syncStart != TRAMP_SYNC_START)
    {
      DEBUG("trampReadResponse, first header byte not equal SyncByte");
      setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
      return false;
    }
    if (frame->footer.syncStop != TRAMP_SYNC_STOP)
    {
      DEBUG("trampReadResponse, last byte not equal Sync Stop");
      setError(VTXErrors::vtxLastByteNotSyncStop);
      return false;
    }
    // const uint8_t crc = trampCrc(frame);
    const uint8_t crc = trampCrc(tramp_rx_buf);
    if (crc != frame->footer.crc)
    {
      DEBUG("trampReadResponse, Incorrect checksum");
      setError(VTXErrors::vtxParseResponseInvalidCRCOrBuffer);
      return false;
    }
    DEBUG("trampReadResponse: Update Data");
    // TODO:here we need to update data dependently of response code (cmd)
    switch (frame->header.command)
    {
    case TRAMP_COMMAND_CMD_RF: //'r', Init
    {
      // we ignore this response, just parse
      DEBUG("trampReadResponse: Response from Init: do nothing");
      break;
    }
    case TRAMP_COMMAND_GET_CONFIG: //'v' //we update data for current settings just sending TRAMP_COMMAND_GET_CONFIG and reading request
    {
      DEBUG("trampReadResponse: Response from Get_CONFIG: Update data");
      const uint16_t freq = frame->payload.settings.frequency;
      // Check we're not reading the request (indicated by freq zero)
      if (freq != 0)
      {
        // update data
        ch_index = getChannelIndex(frame->payload.settings.frequency);
        pwr_Level = getPowerIndexFromMW(frame->payload.settings.power);
        pitMode = (bool)frame->payload.settings.pitModeEnabled;
        DEBUG("trampReadResponse: Updated, Freq:" + (String)frame->payload.settings.frequency);
        DEBUG("trampReadResponse: Converted to Channel:" + (String)ch_index);
        DEBUG("trampReadResponse: Updated, Power in Mw:" + (String)frame->payload.settings.power);
        DEBUG("trampReadResponse: Converted to Power Level:" + (String)pwr_Level);
        DEBUG("trampReadResponse: Updated, PitMode:" + (String)frame->payload.settings.pitModeEnabled);
      }
      break;
    }
    case TRAMP_COMMAND_CMD_SENSOR: //'s' - temperature sensor, currently not used
    {
      // currently we ignore this response, just parse
      DEBUG("trampReadResponse: Response from CMD_Sensor: do nothing");
      break;
    }
    }
    // clear port
    port->flush();
    // successful response, wait another 100ms to give the VTX a chance to recover
    // before sending another command.
    delay(100); // ����� ����� ������������ delay
    DEBUG("trampReadResponse:Succesfull");
    return true;
  }
  else
  {
    DEBUG("trampReadResponse2, incoming_bytes_count != TRAMP_FRAME_LENGTH");
    setError(VTXErrors::vtxBufferLengthLessWholePacket);
  }
  // clear port
  port->flush();
  return false;
}

void VTXControl::applyProbeResult(const VtxProbeResult& result) {
  _trailingZero = result.trailingZero;
  _leadingZeroSkip = result.leadingZeroSkip;
  _powerInLowerNibble = result.powerInLowerNibble;
  _ignoreCRC = result.ignoreCRC;
  _strictReadback = result.strictReadback;
  _directFreq = result.directFreq;
  _pitPowerZero = result.pitPowerZero;

  if (result.baudRate > 0 && result.serialConfig > 0) {
    port->begin(result.baudRate, result.serialConfig);
  }
}

VtxProbeResult VTXControl::probe(uint8_t pin,
                                  const uint16_t* channelFreqs,
                                  int channelFreqCount) {
  VtxProbeResult r = {};
  r.mode = SmartAudio;
  strcpy(r.protocol, "smartaudio");
  int avail;

#if defined(ESP32)
  ESP32HalfDuplex p(pin, pin, false, false);
  const long bauds[] = {4800, 4860, 4920, 4980, 5040, 4560};
  const int numBauds = 6;
  const uint16_t configs[] = {802, 801};
  const int numConfigs = 2;

  // buffer for GET_SETTINGS command
  const uint8_t getSettingsCmd[] = {0xAA, 0x55, SMARTAUDIO_CMD_GET_SETTINGS, 0x00, 0x9F};

  for (int bi = 0; bi < numBauds; bi++) {
    for (int ci = 0; ci < numConfigs; ci++) {
    for (int tz = 0; tz <= 1; tz++) {
      Serial.printf("PROBE: trying baud=%ld cfg=%d tz=%d\n", bauds[bi], configs[ci], tz);
      p.begin(bauds[bi], configs[ci]);
      delay(150);
      p.flush();

      // send GET_SETTINGS
      p.writeDummyByte();
      p.write(getSettingsCmd, sizeof(getSettingsCmd));
      if (tz) p.write((uint8_t)0x00);
      p.listen();

      // accumulate response and scan for sync
      int availCount = p.available();
      int totalBytes = availCount;

      // scan through buffer for 0xAA 0x55 sync
      uint8_t cmd = 0, len = 0;
      uint8_t payload[AP_SMARTAUDIO_MAX_PACKET_SIZE];
      uint8_t dataRead = 0;
      int leadZeros = 0;
      bool gotFrame = false;

      while (availCount-- > 0) {
        uint8_t b = p.read();
        if (b == 0x00) { leadZeros++; continue; }
        if (b == SMARTAUDIO_SYNC_BYTE && p.peek() == SMARTAUDIO_HEADER_BYTE) {
          // found sync — consume header and read frame
          availCount--; p.read(); // discard 0x55
          cmd = p.read(); availCount--;
          len = p.read(); availCount--;
          if (len >= AP_SMARTAUDIO_MAX_PACKET_SIZE) break;
          for (dataRead = 0; dataRead < len + 1 && dataRead < AP_SMARTAUDIO_MAX_PACKET_SIZE - 4; dataRead++) {
            payload[dataRead] = p.read(); availCount--;
          }
          gotFrame = true;
          break;
        }
      }
      p.flush();

      if (!gotFrame) {
        Serial.printf("PROBE: no frame at baud=%ld cfg=%d tz=%d (%d bytes)", bauds[bi], configs[ci], tz, totalBytes);
        if (p.available() > 0) {
          Serial.print(" [");
          for (int i = 0; i < 16 && p.available() > 0; i++) {
            Serial.printf(" %02X", p.read());
          }
          Serial.print(" ]");
        }
        Serial.println();
        continue;
      }
      Serial.printf("PROBE: cmd=0x%02X len=%d leadZeros=%d\n", cmd, len, leadZeros);

      // determine version from command byte
      int ver = 0;
      if (cmd == SMARTAUDIO_RSP_GET_SETTINGS_V1) {
        ver = 1; strcpy(r.modeName, "index");
      } else if (cmd == SMARTAUDIO_RSP_GET_SETTINGS_V2) {
        ver = 2; strcpy(r.modeName, "index");
      } else if (cmd == SMARTAUDIO_RSP_GET_SETTINGS_V21) {
        ver = 21; strcpy(r.modeName, "dbm");
      } else { Serial.printf("PROBE: unknown cmd 0x%02X\n", cmd); continue; }

      Serial.printf("PROBE: SmartAudio v%d detected!\n", ver);

      // — SmartAudio detected —
      r.saVersion = ver;
      r.baudRate = bauds[bi];
      r.serialConfig = configs[ci];
      r.trailingZero = (tz == 1);
      r.leadingZeroSkip = (leadZeros > 0);
      r.ignoreCRC = true;            // most clones need CRC off
      r.confidence = 40;
      p.flush();

      // build power table from detected version
      if (ver == 1) {
        static const uint16_t v1mw[] = {25, 200, 500, 800};
        for (int i = 0; i < 4 && i < 8; i++) r.powersMw[i] = v1mw[i];
        r.powerCount = 4;
      } else if (ver == 21) {
        r.powerCount = 4;
        // Extract L0-L3 dBm from GET_SETTINGS payload[7..10] and convert to mW
        for (int i = 0; i < r.powerCount && (i + 7) < len; i++) {
          uint8_t dbm = payload[7 + i] & 0x7F;
          r.powersMw[i] = dbmToMw(dbm);
        }
      } else {
        // v2 — power mW depends on VTX; use generic 4-level table
        static const uint16_t v2mw[] = {25, 200, 400, 800};
        for (int i = 0; i < 4 && i < 8; i++) r.powersMw[i] = v2mw[i];
        r.powerCount = 4;
      }
      r.confidence = 60;

      // — probe power encoding via SET_POWER(level0) —
      uint8_t setPowerCmd[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_POWER, 1, 0, 0};
      if (ver == 1) {
        setPowerCmd[4] = powers_v1[0];  // V1: 7 for 25mW
      } else if (ver == 21) {
        setPowerCmd[4] = 0x80 | 14;     // V2.1: dBm=14 (25mW) with MSB set
      } else {
        setPowerCmd[4] = 0;             // V2: direct index 0
      }
      setPowerCmd[5] = sa_CRC8(setPowerCmd, 5);

      p.writeDummyByte();
      p.write(setPowerCmd, 6);
      if (r.trailingZero) p.write((uint8_t)0x00);
      p.listen();

      avail = p.available();
      if (avail < 6) { r.confidence = 50; p.flush(); return r; }

      // skip leading zeros
      while (r.leadingZeroSkip && p.peek() == 0x00) p.read();

      {
      uint8_t sbyte = p.read();
      if (sbyte != SMARTAUDIO_SYNC_BYTE) { r.confidence = 50; p.flush(); return r; }
      uint8_t hbyte = p.read();
      if (hbyte != SMARTAUDIO_HEADER_BYTE) { r.confidence = 50; p.flush(); return r; }
      cmd = p.read();
      if (cmd != SMARTAUDIO_RSP_SET_POWER) { r.confidence = 50; p.flush(); return r; }
      }

      // read SET_POWER payload (2 bytes U16 + CRC)
      uint8_t pl[3] = {(uint8_t)p.read(), (uint8_t)p.read(), (uint8_t)p.read()};
      uint16_t payload16 = ((uint16_t)pl[0] << 8) | pl[1];
      uint8_t powerByte = payload16 & 0xFF;

      // analyze power encoding
      if (ver == 1) {
        // v1 level 0 should be 7
        r.powerInLowerNibble = (powerByte != 7 && (powerByte & 0x0F) == 0);
        r.confidence = (powerByte == 7) ? 90 : 60;
      } else if (ver == 21) {
        // v2.1 level 0 should be 14
        r.powerInLowerNibble = (powerByte != 14 && (powerByte & 0x0F) == 0);
        r.confidence = (powerByte == 14) ? 90 : 60;
      } else {
        // v2: test both interpretations
        if (powerByte == 0) {
          // direct index match
          r.powerInLowerNibble = false;
          r.confidence = 90;
        } else if ((powerByte & 0x0F) == 0) {
          // lower nibble match (TX805S: 0xE0, 0xF0, etc.)
          r.powerInLowerNibble = true;
          r.confidence = 90;
        } else {
          r.confidence = 50;
        }
      }

      // probe strictReadback — does echo match?
      r.strictReadback = true;

      // SA v2+ supports direct frequency (SMARTAUDIO_CMD_SET_FREQUENCY)
      r.directFreq = (ver >= 2);

      // probe pitPowerZero
      r.pitPowerZero = false;

      p.flush();
      return r;
    }
    }
  }

  // — SmartAudio failed, try Tramp —
  p.begin(AP_TRAMP_UART_BAUD, AP_TRAMP_UART_CFG);
  delay(150);
  p.flush();

  uint8_t trampProbe[] = {TRAMP_SYNC_START, TRAMP_COMMAND_GET_CONFIG,
                           0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  trampProbe[14] = trampCrc(trampProbe);
  trampProbe[15] = TRAMP_SYNC_STOP;

  p.write(trampProbe, TRAMP_FRAME_LENGTH);
  delay(50);

  avail = p.available();
  if (avail == TRAMP_FRAME_LENGTH) {
    uint8_t buf[TRAMP_FRAME_LENGTH];
    for (int i = 0; i < TRAMP_FRAME_LENGTH; i++) buf[i] = p.read();
    if (buf[0] == TRAMP_SYNC_START && buf[15] == TRAMP_SYNC_STOP) {
      r.mode = Tramp;
      strcpy(r.protocol, "tramp");
      strcpy(r.modeName, "mw");
      r.baudRate = AP_TRAMP_UART_BAUD;
      r.serialConfig = AP_TRAMP_UART_CFG;
      uint16_t mw = ((uint16_t)buf[3] << 8) | buf[2];
      static const uint16_t trampPowers[] = {25, 200, 400, 800};
      for (int i = 0; i < 4 && i < 8; i++) r.powersMw[i] = trampPowers[i];
      r.powerCount = 4;
      r.confidence = 70;
    }
  }

  p.flush();
#else
  (void)channelFreqs;
  (void)channelFreqCount;
#endif
  return r;
}
