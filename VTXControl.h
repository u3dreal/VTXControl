#include <Arduino.h>
#if defined(ESP32)
#include "ESP32HalfDuplex.h"
#else
#include <SoftwareSerialWithHalfDuplex.h>
#endif
#include "VTX_SmartAudio.h"
#include "VTX_Tramp.h"
#ifndef VTXControl_h
#define VTXControl_h
//--------------------------
#define VTXCDEBUG 0 //Uncomment this define to see the diagnostics
#define ESP32HD_DEBUG 0
//--------------------------
#if VTXCDEBUG
#define DEBUG(x) Serial.println(x)
#else
#define DEBUG(x)
#endif
#if VTXCDEBUG
static void dumpBuffer(const uint8_t* data, const int8_t len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.print((uint8_t)data[i] < 0x10 ? " 0" : " ");
    Serial.print((uint8_t)data[i], HEX);
  }
  Serial.println("");
}
#endif

enum VTXMode
{
  SmartAudio = 1,
  Tramp = 2,
};

struct VtxProbeResult {
  char protocol[16];
  char modeName[8];
  VTXMode mode;
  int saVersion;

  long baudRate;
  uint16_t serialConfig;
  uint16_t responseWaitMs;

  uint16_t powersMw[8];
  uint8_t powerCount;

  bool trailingZero;
  bool leadingZeroSkip;
  bool powerInLowerNibble;
  bool ignoreCRC;
  bool strictReadback;
  bool directFreq;
  bool pitPowerZero;

  int confidence;
};
enum VTXErrors
{
  vtxNoErrors = 0,  
  vtxParseResponseInvalidCRCOrBuffer = 0x01,
  vtxIncomingBytesZero = 0x02,
  vtxIncomingBytesLessHeaderSize = 0x04,
  vtxIncomingByteNotEqualSyncByte = 0x08,
  vtxIncomingByteNotEqualHeaderByte = 0x10,
  vtxPacketSizeGreaterMaxPacketSize = 0x20,
  vtxBufferLengthLessWholePacket = 0x40,
  //--
  vtxportIsNotListening = 0x0100,
  vtxportBufferIsEmpty = 0x0200,
  vtxportTxDelayIsZero = 0x0400,
  vtxportRXDelayStopBitNotSet = 0x8000,
  vtxtrampNotInited = 0x10000,
  vtxLastByteNotSyncStop = 0x20000,
};


class VTXControl
{
   
public:  
  VTXControl(
    int vtxMode, 
    int softPin, 
    const uint16_t powers[],
      int powers_len,
    const uint16_t freqs[],
      int freqs_len,
    int responseTimeOut = 1000, 
    bool smartBaudRate = true, 
    int numtries = 3);
  void flush();
  void waitForInMs(unsigned int ms);
  bool setPitMode(bool enabled);
  bool setChannel(int freqIndex);//sets frequency by channel index
  bool setFrequency(uint16_t freq);//set frequency by freq value
  bool setPower(int pwrLevel);//sets power by power index in table of powers
  bool setPowerInmW(uint16_t pwrmW);//sets power by value in mW
  bool setNextChannel();
  bool setPrevChannel();
  bool updateParameters();
  int getPowerLevel() { return pwr_Level; }
  int getChannelIndex() { return ch_index; }
  bool getPitMode() { return pitMode; }
  uint16_t getFrequency() { return _frequency; }
  uint16_t getPowerMw(int level); // returns mW for level (0-3), uses detected V2.1 table if available
  bool sa_readResponse();
  void clearErrors();
  VTXErrors getErrors();
  long getSpeed();

  void applyProbeResult(const VtxProbeResult& result);

  // Runtime flag getters/setters
  void setTrailingZero(bool enable) { _trailingZero = enable; }
  bool getTrailingZero() { return _trailingZero; }
  void setLeadingZeroSkip(bool enable) { _leadingZeroSkip = enable; }
  bool getLeadingZeroSkip() { return _leadingZeroSkip; }
  void setPowerInLowerNibble(bool enable) { _powerInLowerNibble = enable; }
  bool getPowerInLowerNibble() { return _powerInLowerNibble; }
  void setIgnoreCRC(bool enable) { _ignoreCRC = enable; }
  bool getIgnoreCRC() { return _ignoreCRC; }
  void setStrictReadback(bool enable) { _strictReadback = enable; }
  bool getStrictReadback() { return _strictReadback; }
  void setDirectFreq(bool enable) { _directFreq = enable; }
  bool getDirectFreq() { return _directFreq; }
  void setPitPowerZero(bool enable) { _pitPowerZero = enable; }
  bool getPitPowerZero() { return _pitPowerZero; }

  static VtxProbeResult probe(
    uint8_t pin,
    const uint16_t* channelFreqs,
    int channelFreqCount);
#if VTXCDEBUG
  bool testSMAWrite();
  bool testSMAResponseFromSerial1();
#endif
private:
#if defined(ESP32)
  ESP32HalfDuplex* port;
#else
  SoftwareSerialWithHalfDuplex* port;
#endif
  int vtx_mode = VTXMode::SmartAudio;//default
  VTXErrors errors = VTXErrors::vtxNoErrors;
  long sa_offerNewSpeed(long currentSpeed);//tries to offer other baud rate to work with vtx
  void setError(VTXErrors error);
  ProtocolVersion sa_protocol_version;//smart audio protocol version
  int pwr_Level = -1;//default value -1 means not updated (not requested from VTX)
  int ch_index = -1;//default value -1 means not updated (not requested from VTX)
  bool pitMode = false;
  uint16_t _frequency = 0;
  bool initialized = false;//tramp protocol needs to be initialized, so this var reflects state of Tramp initialization
  int _responseTimeOut = 1000;//in ms
  int _numtries = 3;//num tries to send request and receive response, after that we try to change baud rate and try again
  bool _smartBaudRate = true;//tries to find apporpriated baud rate to communicate with VTX, if false- just work on fixed initial baudrate

  // Runtime behavioral flags (set by probe or applyProbeResult)
  bool _trailingZero = false;
  bool _leadingZeroSkip = true;
  bool _powerInLowerNibble = true;
  bool _ignoreCRC = true;
  bool _strictReadback = true;
  bool _directFreq = false;
  bool _pitPowerZero = false;

  const uint16_t* _powers;//table of powers in mW
  int _power_size;
  const uint16_t* _freqs;//table of frequencies in MHz
  int _freqs_size;
  uint16_t _detectedPowersMw[4]; // populated from V2.1 response L0-L3 dBm table

  //some utility functions  
  int getChannelIndex(uint16_t freq);
  uint16_t getChannelFrequency(int chIndex);
  int getPowerIndexFromMW(uint16_t pwrInmW);
  int getPowerIndexFromDbm(uint16_t pwrInDbm);
  int getPowerIndexFromV1(uint16_t pwrValue);
  uint16_t getPowerInmW(int pwrIndex);
    
  bool sa_updateSettings(); 
  bool sa_ignoreCrc() const { return _ignoreCRC; }  
  bool sa_parseResponseBuffer(const uint8_t* buffer);
  // command functions
  bool sa_setPitMode(bool enabled);
  bool sa_getSettings();
  bool sa_setChannel(uint8_t channel);
  bool sa_setFrequency(uint16_t freq);
  bool sa_setPower(int pwrLevel);      
  bool trampSendPacket(uint8_t* packet, bool respRequired);//trampFrame_t* packet);
  bool trampPush(const uint8_t* packet);//const trampFrame_t* object);
  bool trampSendCmd(uint8_t cmd);
  bool trampSendCmd(uint8_t cmd, uint16_t param);

  //uint8_t trampGetState();
  bool trampSetFrequency(uint16_t freq);
  bool trampSetPower(uint16_t milliWatts);
  bool trampInit();
  //bool trampSetPitMode(bool enabled);
  bool trampUpdate();
  long trampOfferNewSpeed(long currentSpeed);//tries to offer other baud rate to work with vtx
  bool trampGetStatus();  
  bool trampReadResponse();    
};
#endif
