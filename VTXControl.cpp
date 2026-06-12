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
const uint16_t powers_v21[4] = {14 /*25mw*/, 20 /*200mw*/, 26 /*500mw*/, 46 /*5w*/}; // for SmartAudio protocol v2.1 in dbm

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
  buf[4] = enabled ? 0x01 : 0x00;
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
  // On my Unify Pro32 the SmartAudio response is sent exactly 100ms after the request
  // and the initial response is 40ms long so we should wait at least 140ms before giving up
#if defined(ESP32)
  // ESP32HalfDuplex::available() polls for up to 500ms; no blocking wait needed
#else
  waitForInMs(_responseTimeOut);
#endif

  int16_t incoming_bytes_count = port->available();
  // check if it is a response in the wire
  if (incoming_bytes_count < 0)
  {
    DEBUG("sa readResponse, return error on Avaialble");
    return false;
  }
  else if (incoming_bytes_count == 0)
  {
    DEBUG("sa readResponse, incoming bytes=0, return false");
    setError(VTXErrors::vtxIncomingBytesZero); //
    return false;
  }
  // now we have no-zero length response, we can dump it
#if VTXCDEBUG
  port->dumpReceiveBuffer();
#endif
  // skip leading zero bytes if expected (Eachine TX5258 and similar clones)
  uint8_t b = port->peek();
  int skippedbytes = 0;
  while (_leadingZeroSkip && b == 0x00)
  {
    port->read(); // skip 0x00 byte
    skippedbytes++;
    b = port->peek();
  }
  if (skippedbytes > 0)
  {
    incoming_bytes_count = port->available(); // refresh incoming_bytes_count
    DEBUG("sa readResponse, skipped zero bytes =" + (String)skippedbytes);
  }
  const uint8_t response_header_size = sizeof(FrameHeader);
  // wait until we have enough bytes to read a header
  if (incoming_bytes_count < response_header_size)
  {
    DEBUG("sa readResponse, incoming_bytes_count < response_header_size: " + (String)incoming_bytes_count + ", return false");
    setError(VTXErrors::vtxIncomingBytesLessHeaderSize);
    return false;
  }
  DEBUG("sa readResponse, incoming_bytes_count = " + (String)incoming_bytes_count);
  // allocate response buffer
  uint8_t response_buffer[AP_SMARTAUDIO_MAX_PACKET_SIZE];
  uint8_t buffer_length = 0;
  // expected packet size
  uint8_t packet_size = 0;
  // now have at least the header, read it if necessary

  // read the first non-zero byte
  b = port->read();
  // didn't see a sync byte, discard and go around again
  if (b != SMARTAUDIO_SYNC_BYTE)
  {
    DEBUG("readResponse, b != SMARTAUDIO_SYNC_BYTE (b=" + (String)b + "), return false");
    setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
    return false;
  }
  response_buffer[buffer_length++] = b;
  // read header byte
  b = port->read();
  // didn't see a header byte, discard and reset
  if (b != SMARTAUDIO_HEADER_BYTE)
  {
    buffer_length = 0;
    DEBUG("readResponse, b != SMARTAUDIO_HEADER_BYTE, return false");
    setError(VTXErrors::vtxIncomingByteNotEqualHeaderByte);
    return false;
  }
  response_buffer[buffer_length++] = b;
  // read the rest of the header
  for (; buffer_length < response_header_size; buffer_length++)
  {
    b = port->read();
    response_buffer[buffer_length] = b;
  }

  FrameHeader *header = (FrameHeader *)response_buffer;
  incoming_bytes_count -= response_header_size;

  // implementations that ignore the CRC also appear to not account for it in the frame length
  // if (sa_ignoreCrc())
  //{
  //  header->length++;
  //}
  packet_size = header->length;

  // read the rest of the packet
  // check for overflow
  if (packet_size >= AP_SMARTAUDIO_MAX_PACKET_SIZE)
  {
    DEBUG("readResponse, rest packet size >= AP_SMARTAUDIO_MAX_PACKET_SIZE, return false");
    setError(VTXErrors::vtxPacketSizeGreaterMaxPacketSize);
    return false;
  }
  for (uint8_t i = 0; i < packet_size; i++)
  {
    uint8_t b = port->read();
    response_buffer[buffer_length++] = b;
  }

    // didn't get the whole packet
    if (buffer_length < packet_size + response_header_size)
    {
        DEBUG("readResponse, buffer_length < packet_size + response_header_size, return false");
        setError(VTXErrors::vtxBufferLengthLessWholePacket);
        return false;
    }

    bool correct_parse = sa_parseResponseBuffer(response_buffer);
  port->flush(); // clear the read buffer
  // successful response, wait another 100ms to give the VTX a chance to recover
  // before sending another command.
  waitForInMs(100);
  DEBUG("readResponse end, correct_parse = " + (String)correct_parse);
  return correct_parse;
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
    pitMode = (resp->operationMode & 0x04) != 0;
  }
  break;
  case SMARTAUDIO_RSP_GET_SETTINGS_V2:
  {
    DEBUG("sa_parse_response_buffer(), Protocol version 2");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v2;
    // Standard: AA 55 09 05 [VER] [CH] [PW] [MODE] [FREQ_L] [FREQ_H] [CRC]
    // TX805S:   AA 55 09 06 [CH] [??] [PW] [??] [??] [??] [CRC]
    // _powerInLowerNibble discriminates byte layout:
    //   true  → buffer[4] = global channel, buffer[6] = raw power
    //   false → buffer[5] = global channel, buffer[6] = raw power (standard)
    if (_powerInLowerNibble)
      ch_index = buffer[4];
    else
      ch_index = buffer[5];
    pwr_Level = buffer[6];
    pitMode = (buffer[7] & 0x04) != 0;
    Serial.print("V2 RAW:");
    for (int _i = 0; _i < 11; _i++) { Serial.print(' '); Serial.print(buffer[_i], HEX); }
    Serial.println();
  }
  break;

  case SMARTAUDIO_RSP_GET_SETTINGS_V21:
  {
    DEBUG("sa_parse_response_buffer(), Protocol version 2.1");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v21;
    // Response: AA 55 11 xx [VER] [CH] [PW] [MODE] [FREQ_H] [FREQ_L] [PWR_DBM] [NUM_LVLS] [LVL0-7] [CRC]
    ch_index = buffer[5];
    pwr_Level = getPowerIndexFromDbm(buffer[10]);
    pitMode = (buffer[7] & 0x04) != 0;
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
    const U16ResponseFrame *respu16 = (const U16ResponseFrame *)buffer;
    uint16_t freq = respu16->payload & SMARTAUDIO_FREQUENCY_MASK;
    ch_index = getChannelIndex(freq);
  }
  break;
  case SMARTAUDIO_RSP_SET_MODE:
  {
    DEBUG("sa_parse_response_buffer(), SetMode");
    const U8ResponseFrame *respu8 = (const U8ResponseFrame *)buffer;
    pitMode = (respu8->payload & 0x04) != 0;
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
  for (int i = 0; i < _numtries; i++) // trying to set _numtries times
  {
    switch (vtx_mode)
    {
    case VTXMode::SmartAudio:
    {
      int chIndex = getChannelIndex(freq);
      if (chIndex != -1)
      {
        if (sa_setChannel(chIndex))
        {
          res = sa_readResponse();
        }
      }
      else
      {
        if (sa_setFrequency(freq))
        {
          res = sa_readResponse();
        }
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
        res = trampSetFrequency(freq);
    }
    break;
    }
    if (res)
    {
      uint16_t curr_freq = getChannelFrequency(ch_index);
      if (curr_freq == freq)
        return true;
      // frequency was set directly (non-table freq via sa_setFrequency)
      if (curr_freq == 0 && ch_index < 0)
        return true;
    }
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

  if (result.baudRate > 0) {
    port->setSpeed(result.baudRate);
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

  // buffer for GET_SETTINGS command
  const uint8_t getSettingsCmd[] = {0xAA, 0x55, SMARTAUDIO_CMD_GET_SETTINGS, 0x00, 0x9F};

  for (int bi = 0; bi < numBauds; bi++) {
    for (int tz = 0; tz <= 1; tz++) {
      p.begin(bauds[bi], AP_SMARTAUDIO_UART_CFG);
      delay(150);
      p.flush();

      // send GET_SETTINGS
      p.writeDummyByte();
      p.write(getSettingsCmd, sizeof(getSettingsCmd));
      if (tz) p.write((uint8_t)0x00);
      p.listen();

      avail = p.available();
      Serial.printf("PROBE: baud=%ld tz=%d avail=%d\n", bauds[bi], tz, avail);
      if (avail < 4) { Serial.println("PROBE: avail<4, skip"); continue; }

      // detect leading zeros
      int leadZeros = 0;
      while (p.peek() == 0x00) { p.read(); leadZeros++; }
      if (leadZeros > 0) {
        avail = p.available();
        if (avail < 4) continue;
      }

      uint8_t sync = p.read();
      if (sync != SMARTAUDIO_SYNC_BYTE) { Serial.printf("PROBE: sync=0x%02X != 0xAA\n", sync); continue; }
      uint8_t hdr = p.read();
      if (hdr != SMARTAUDIO_HEADER_BYTE) { Serial.printf("PROBE: hdr=0x%02X != 0x55\n", hdr); continue; }

      uint8_t cmd = p.read();
      uint8_t len = p.read();
      if (len >= AP_SMARTAUDIO_MAX_PACKET_SIZE) { Serial.printf("PROBE: len=%d too big\n", len); continue; }
      Serial.printf("PROBE: sync=0x%02X hdr=0x%02X cmd=0x%02X len=%d\n", sync, hdr, cmd, len);

      // read payload + CRC
      uint8_t payload[AP_SMARTAUDIO_MAX_PACKET_SIZE];
      uint8_t plen = 0;
      for (; plen < len + 1 && plen < AP_SMARTAUDIO_MAX_PACKET_SIZE - 4; plen++) {
        payload[plen] = p.read();
      }

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
        static const uint16_t v21mw[] = {25, 200, 500, 5000};
        for (int i = 0; i < 4 && i < 8; i++) r.powersMw[i] = v21mw[i];
        r.powerCount = 4;
      } else {
        // v2 — power mW depends on VTX; use generic 4-level table
        static const uint16_t v2mw[] = {25, 200, 400, 800};
        for (int i = 0; i < 4 && i < 8; i++) r.powersMw[i] = v2mw[i];
        r.powerCount = 4;
      }
      r.confidence = 60;

      // — probe power encoding via SET_POWER(0) —
      uint8_t setPowerCmd[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_POWER, 1, 0, 0};
      setPowerCmd[4] = 0;           // level 0
      setPowerCmd[5] = sa_CRC8(setPowerCmd, 5);

      p.writeDummyByte();
      p.write(setPowerCmd, 6);
      if (r.trailingZero) p.write((uint8_t)0x00);
      p.listen();

      avail = p.available();
      if (avail < 6) { r.confidence = 50; p.flush(); return r; }

      // skip leading zeros
      while (r.leadingZeroSkip && p.peek() == 0x00) p.read();

      sync = p.read();
      if (sync != SMARTAUDIO_SYNC_BYTE) { r.confidence = 50; p.flush(); return r; }
      hdr = p.read();
      if (hdr != SMARTAUDIO_HEADER_BYTE) { r.confidence = 50; p.flush(); return r; }
      cmd = p.read();
      if (cmd != SMARTAUDIO_RSP_SET_POWER) { r.confidence = 50; p.flush(); return r; }

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
