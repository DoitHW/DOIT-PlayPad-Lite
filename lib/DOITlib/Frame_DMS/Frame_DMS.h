#pragma once

#ifndef FRAME_DMS_H
#define FRAME_DMS_H

#include <Arduino.h>
#include <defines_DMS/defines_DMS.h>
#include <stdint.h>
#include <vector>

extern volatile bool frameReceived;
extern std::vector<byte> uartBuffer;

void IRAM_ATTR onUartInterrupt();

struct TARGETNS {
  byte mac01;
  byte mac02;
  byte mac03;
  byte mac04;
  byte mac05;
};

constexpr TARGETNS NS_ZERO = {0, 0, 0, 0, 0};

struct LAST_ENTRY_FRAME_T {
  byte room = 0;
  byte origin = 0;
  TARGETNS originNS = NS_ZERO;
  byte targetType = 0;
  TARGETNS targetNS = NS_ZERO;
  byte function = 0;
  std::vector<byte> data;
};

struct FRAME_T {
  byte start = NEW_START;
  byte frameLengthMsb = 0;
  byte frameLengthLsb = 0;
  byte room = DEFAULT_ROOM;
  byte origin = DEFAULT_BOTONERA;
  TARGETNS originNS = NS_ZERO;
  byte targetType = BROADCAST;
  TARGETNS targetNS = NS_ZERO;
  byte function = 0;
  byte dataLengthMsb = 0;
  byte dataLengthLsb = 0;
  std::vector<byte> data;
  byte checksum = 0;
  byte end = NEW_END;
};

LAST_ENTRY_FRAME_T extract_info_from_frameIn(const std::vector<uint8_t> &frame);
void send_frame(const FRAME_T &framein);

void setLocalNS(const TARGETNS &ns);
void setLocalRoom(uint8_t room);

FRAME_T frameMaker_SEND_COLOR(byte originin, byte targetType, TARGETNS targetNS,
                              byte colorin);
FRAME_T frameMaker_SEND_COMMAND(byte originin, byte targetType,
                                TARGETNS targetNS, byte commandin);
FRAME_T frameMaker_SEND_FLAG_BYTE(byte originin, byte targetType,
                                  TARGETNS targetNS, byte flagin);
FRAME_T frameMaker_SEND_PATTERN_NUM(byte originin, byte targetType,
                                    TARGETNS targetNS, byte patternin);

#endif
