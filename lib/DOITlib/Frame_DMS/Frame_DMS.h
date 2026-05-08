#pragma once

#ifndef FRAME_DMS_H
#define FRAME_DMS_H

#include <defines_DMS/defines_DMS.h>
#include <Arduino.h>
#include <stdint.h>
#include <vector>
#include <FastLED.h>

//extern volatile bool startFrameReceived;
//extern volatile bool frameInProgress;
extern volatile bool partialFrameReceived;
extern volatile bool frameReceived;

extern std::vector<byte> uartBuffer;

//extern volatile unsigned long last_received_time;
//extern int lastReceivedTime;

void IRAM_ATTR onUartInterrupt();

struct TARGETNS { byte mac01, mac02, mac03, mac04, mac05; };
constexpr TARGETNS NS_ZERO = {0,0,0,0,0};   // para 0xDB, 0xDC, 0xFF...

struct LAST_ENTRY_FRAME_T{

    byte      room;
    byte      origin;
    TARGETNS  originNS;
    byte      targetType;
    TARGETNS  targetNS;
    byte      function;
    std::vector<byte> data;
    LAST_ENTRY_FRAME_T()
        : room(0),
          origin(0),
          originNS{0,0,0,0,0},
          targetType(0),
          targetNS{0,0,0,0,0},
          function(0),
          data() {}
};

struct FRAME_T{
  byte start;
  byte frameLengthMsb;
  byte frameLengthLsb;
  byte room;        // identificador de sala
  byte origin;       // 0xDB botonera, 0xDC consola, ...
  TARGETNS originNS; // 5 bytes de serie del emisor
  byte targetType;   // 0xDD dispositivo, 0xFF broadcast, ...
  TARGETNS targetNS; // 5 bytes de serie del objetivo
  byte function;
  byte dataLengthMsb;
  byte dataLengthLsb;
  std::vector<byte> data;
  byte checksum;
  byte end;
};

enum ELEM_UNIT_CONF : uint8_t {

    NO_ELEM= 0,
    COLOR,
    ACTION,
    COLOR_ACTION
};


// struct ELEM_UNIT{

//     ELEM_UNIT_CONF conf;
//     TARGETNS ns;
// };

struct SENSOR_VALUE_T{
    byte msb_min;
    byte lsb_min;
    byte msb_max;
    byte lsb_max;
    byte msb_val;
    byte lsb_val;
};

struct SENSOR_DOUBLE_T{
    byte msb_min;
    byte lsb_min;
    byte msb_max;
    byte lsb_max;
    byte msb_val;
    byte lsb_val;
    byte msb_min2;
    byte lsb_min2;
    byte msb_max2;
    byte lsb_max2;
    byte msb_val2;
    byte lsb_val2;
};

struct COLOR_T{
    byte red;
    byte green;
    byte blue;
};

struct MODE_T{
    
    byte name[24];
    byte desc[192];
    byte config[2];
};

struct INFO_PACK_T{

    byte name[32];
    byte desc[192];
    byte serialNum[5];
    byte ID;
    byte currentMode;
    MODE_T mode[16];
    uint16_t icono[ICON_ROWS][ICON_COLUMNS];
    byte situacion;
};

// struct ICON_PACK_T{
//     uint16_t icono[ICON_ROWS][ICON_COLUMNS]; 
// };

enum BTN_FX : uint8_t {
    NO_FX = 0,
    SOLID,               // Color fijo
    SLOW_WAVE,           // Onda lenta (fade in/out)
    FAST_WAVE,           // Onda rápida
    RAINBOWLOOP,         // Arcoíris cíclico
    BUBBLES_FX,          // Burbujas mejoradas
    BREATHING,           // Respiración suave
    FIRE,                // Efecto fuego
    SPARKLE,             // Destellos aleatorios
    COMET,               // Cometa con estela
    PLASMA,              // Plasma dinámico
    HEARTBEAT,           // Latido de corazón
    AURORA_FX,           // Aurora boreal
    STROBE,              // Estroboscópico
    COLOR_WIPE,          // Barrido de color
    THEATER_CHASE,        // Persecución teatral
    RELAY_FX,
    LOADING,
    LOADING_CIRCULAR,
    LOADING_LINEAL
};

static constexpr uint8_t BTN_RGB_DIRECT = 0x80;  

// Estructura individual de botón (corresponde a un LED físico)
struct BUTTON {
    // 0..8 en la tira
    bool     active;
    uint8_t  numColor;
    uint8_t  r, g, b;
    BTN_FX   fx;
};

// Estructura que agrupa los 9 botones de la botonera
struct COLORPAD_BTNMAP {
    BTN_FX PADFX;
    BUTTON Button_00;   // LED índice 0 (pulsador RELE)
    BUTTON Button_01;   // LED índice 1 (BLANCO)
    BUTTON Button_02;   // LED índice 2 (ROJO)
    BUTTON Button_03;   // LED índice 3 (CELESTE)
    BUTTON Button_04;   // LED índice 4 (AMARILLO)
    BUTTON Button_05;   // LED índice 5 (NARANJA)
    BUTTON Button_06;   // LED índice 6 (VERDE)
    BUTTON Button_07;   // LED índice 7 (VIOLETA)
    BUTTON Button_08;   // LED índice 8 (AZUL)
};

LAST_ENTRY_FRAME_T extract_info_from_frameIn (const std::vector<uint8_t> &frame);
byte checksum_calc                           (const FRAME_T &framein);
void send_frame                              (const FRAME_T &framein);
//void sendRawFrame                            (const std::vector<byte>& rawData);
void setLocalNS                              (const TARGETNS& ns);
const TARGETNS& getLocalNS                   ();
void setLocalRoom                            (uint8_t room);
uint8_t getLocalRoom                         ();

//void  get_sector_data(byte *sector_data, byte lang, byte sector);

// === NUEVAS FIRMAS (todos con origin, targetType, TARGETNS) ===
FRAME_T frameMaker_SET_ELEM_MODE        (byte originin, byte targetType, TARGETNS targetNS, byte modein);
FRAME_T frameMaker_SET_ELEM_ID          (byte originin, byte targetType, TARGETNS targetNS, byte IDin);
FRAME_T frameMaker_SET_ELEM_DEAF        (byte originin, byte targetType, TARGETNS targetNS, byte timein);

FRAME_T frameMaker_SEND_COLOR           (byte originin, byte targetType, TARGETNS targetNS, byte colorin);
FRAME_T frameMaker_SEND_RGB             (byte originin, byte targetType, TARGETNS targetNS, COLOR_T colorin);
FRAME_T frameMaker_SEND_COMMAND         (byte originin, byte targetType, TARGETNS targetNS, byte commandin);
FRAME_T frameMaker_SEND_SENSOR_VALUE    (byte originin, byte targetType, TARGETNS targetNS, SENSOR_DOUBLE_T sensorin);
FRAME_T frameMaker_SEND_SENSOR_VALUE_2  (byte originin, byte targetType, TARGETNS targetNS, SENSOR_VALUE_T sensorin);
FRAME_T frameMaker_SEND_FLAG_BYTE       (byte originin, byte targetType, TARGETNS targetNS, byte flagin);
FRAME_T frameMaker_SEND_PATTERN_NUM     (byte originin, byte targetType, TARGETNS targetNS, byte patternin);
FRAME_T frameMaker_SEND_FILE_NUM        (byte originin, byte targetType, TARGETNS targetNS, byte bankin, byte filein);
FRAME_T frameMaker_SEND_RESPONSE        (byte originin, byte targetType, TARGETNS targetNS, byte response);

FRAME_T frameMaker_RETURN_ELEM_SECTOR   (uint8_t originin, uint8_t targetType, const TARGETNS& targetNS, const uint8_t* sector_data, uint8_t sectorin); 
FRAME_T frameMaker_REQ_ELEM_SECTOR      (byte originin, byte targetType, TARGETNS targetNS, byte idiomain, byte sectorin);
FRAME_T frameMaker_SET_BUTTONS_EXTMAP(
    uint8_t               originType,   // DC
    uint8_t               targetType,   // DB
    const TARGETNS&       destNS,       // 00:00:00:00:00 = broadcast
    const COLORPAD_BTNMAP &map          // 9 botones ya con led_indx 0..8
);

// void buildColorpadFromBtnIds(
//     const BUTTON& b1, const BUTTON& b2, const BUTTON& b3,
//     const BUTTON& b4, const BUTTON& b5, const BUTTON& b6,
//     const BUTTON& b7, const BUTTON& b8, const BUTTON& b9,
//     COLORPAD_BTNMAP& outMap
// );

// Debug
void send_old_color(uint8_t color);
inline uint8_t old_color_checksum(uint8_t color, uint8_t node, uint8_t func, uint8_t dl, uint8_t room );

void rfDebugPrint(const char* msg);
void rfDebugPrintf(const char* fmt, ...);

#endif


