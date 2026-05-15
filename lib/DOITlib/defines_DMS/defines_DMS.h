#ifndef DEFINES_DMS_H
#define DEFINES_DMS_H

#include <Arduino.h>

#if defined(DOIT_LITE)
  #ifndef ADXL
    #define ADXL
  #endif
#endif

constexpr byte DEFAULT_ROOM = 0x01;
constexpr byte ROOM_BROADCAST = 0xFF;

#define ENC_BUTTON 7

#if defined(DEVKIT)
  #define RF_TX_PIN 17
  #define RF_RX_PIN 18
  #define BOTONERA_DATA_PIN 35
#else
  #define RF_TX_PIN 18
  #define RF_RX_PIN 17
  #define BOTONERA_DATA_PIN 21
#endif

#define RF_BAUD_RATE 115200

#define NEW_START 0xE1
#define NEW_END 0xBB
#define BROADCAST 0xFF
#define DEFAULT_BOTONERA 0xDB
#define DEFAULT_DEVICE 0xDD

#define F_SEND_COLOR 0xC1
#define F_SEND_PATTERN_NUM 0xCD
#define F_SEND_FLAG_BYTE 0xCE
#define F_SEND_COMMAND 0xCF
#define F_RETURN_ELEM_SECTOR 0xD0

#define MAX_FRAME_LENGTH 0xFFFF
#define MIN_FRAME_LENGTH 0x15
#define MAX_BUFFER_SIZE 0xFF

#define WHITE 0x00
#define YELLOW 0x01
#define ORANGE 0x02
#define RED 0x03
#define VIOLET 0x04
#define BLUE 0x05
#define LIGHT_BLUE 0x06
#define GREEN 0x07
#define RELAY 0x09

#define DEFAULT_BASIC_MODE 0x00

constexpr uint8_t BLACKOUT = 0x00;
constexpr uint8_t START_CMD = 0x01;
constexpr uint8_t SLEEP_SERIAL_WAKEUP_CMD = 0x02;

#define NUM_LEDS 9

constexpr uint8_t ELEM_SERIAL_SECTOR = 0x03;

#endif
