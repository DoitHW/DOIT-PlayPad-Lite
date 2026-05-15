#ifndef DEFINES_DMS_H
#define DEFINES_DMS_H

#include <Arduino.h>

#pragma once

// --- Reset y estilos básicos ---
#define COLOR_RESET          "\033[0m"
#define COLOR_BOLD           "\033[1m"
#define COLOR_DIM            "\033[2m"
#define COLOR_UNDERLINE      "\033[4m"
#define COLOR_BLINK          "\033[5m"
#define COLOR_REVERSE        "\033[7m"
#define COLOR_HIDDEN         "\033[8m"

// --- Colores de texto estándar ---
#define COLOR_BLACK          "\033[30m"
#define COLOR_RED            "\033[31m"
#define COLOR_GREEN          "\033[32m"
#define COLOR_YELLOW         "\033[33m"
#define COLOR_BLUE           "\033[34m"
#define COLOR_MAGENTA        "\033[35m"
#define COLOR_CYAN           "\033[36m"
#define COLOR_WHITE          "\033[37m"

// --- Colores de texto intensos (brillantes) ---
#define COLOR_BRIGHT_BLACK   "\033[90m"
#define COLOR_BRIGHT_RED     "\033[91m"
#define COLOR_BRIGHT_GREEN   "\033[92m"
#define COLOR_BRIGHT_YELLOW  "\033[93m"
#define COLOR_BRIGHT_BLUE    "\033[94m"
#define COLOR_BRIGHT_MAGENTA "\033[95m"
#define COLOR_BRIGHT_CYAN    "\033[96m"
#define COLOR_BRIGHT_WHITE   "\033[97m"

// --- Colores de fondo estándar ---
#define BG_BLACK             "\033[40m"
#define BG_RED               "\033[41m"
#define BG_GREEN             "\033[42m"
#define BG_YELLOW            "\033[43m"
#define BG_BLUE              "\033[44m"
#define BG_MAGENTA           "\033[45m"
#define BG_CYAN              "\033[46m"
#define BG_WHITE             "\033[47m"

// --- Colores de fondo intensos ---
#define BG_BRIGHT_BLACK      "\033[100m"
#define BG_BRIGHT_RED        "\033[101m"
#define BG_BRIGHT_GREEN      "\033[102m"
#define BG_BRIGHT_YELLOW     "\033[103m"
#define BG_BRIGHT_BLUE       "\033[104m"
#define BG_BRIGHT_MAGENTA    "\033[105m"
#define BG_BRIGHT_CYAN       "\033[106m"
#define BG_BRIGHT_WHITE      "\033[107m"


// DELFINES GLOBALES
// #define BOTONERA_NEW           // -> BOTONERA_OLD / BOTONERA_NEW
#define NOELEM

#if defined(DOIT_LITE)
  #ifndef BOTONERA
    #define BOTONERA
  #endif
  #ifndef NONFC
    #define NONFC
  #endif
  #ifndef NOPLAYER
    #define NOPLAYER
  #endif
  #ifndef NOMIC
    #define NOMIC
  #endif
  #ifndef ADXL
    #define ADXL
  #endif
#else
  #define BOTONERA
  #define PLAYER                 // -> PLAYER / NOPLAYER
  #define NONFC                  // -> NFC / NONFC
  #define MIC                    // -> MIC / NOMIC
  #define ADXL                   // -> ADXL / NOADXL
#endif

constexpr byte DEFAULT_TABLET       = 0xD1;
constexpr byte DEFAULT_COLOR_PAD    = 0xD2;
constexpr byte DEFAULT_MOTION_MAT   = 0XD3;
constexpr byte DEFAULT_VOICE_MAT    = 0xD4;
constexpr byte DEFAULT_VOICE_RECON  = 0xD5;
constexpr byte DEFAULT_COLOR_MAT    = 0xD6;
constexpr byte DEFAULT_SELECTOR_MAT = 0xD7;
constexpr byte DEFAULT_JOYSTICK     = 0xD8;
constexpr byte DEFAULT_ROOM         = 0x01;
constexpr byte ROOM_BROADCAST       = 0xFF;
constexpr byte DEFAULT_PAD          = 0xD9;

#define AP_ELEM_IP                    200, 200, 200, 1
#if !defined(BOTONERA_NEW) && !defined(BOTONERA_OLD)
  #error "Debe definirse BOTONERA_NEW o BOTONERA_OLD desde platformio.ini"
#elif defined(BOTONERA_NEW) && defined(BOTONERA_OLD)
  #error "No defina BOTONERA_NEW y BOTONERA_OLD a la vez"
#endif

#if defined(BOTONERA_NEW)
  #define AP_SSID_NAME  "DOIT_BOTONERA_LINK_NEW"
#elif defined(BOTONERA_OLD)
  #define AP_SSID_NAME  "DOIT_BOTONERA_LINK_OLD"
#endif

#define AP_SSID_PASS                  "12345678"
#define OTA_PASS                      "wakeupneo" 

#define LEGACY_COLOR_SUPPORT

#if !defined(DOIT_LITE)
  #define DEBUG
  #define DEBUG_
#endif
#if defined(DEBUG_) && !defined(NODEBUG)
  #define DEBUG__________ln(...)          Serial.println(__VA_ARGS__)
  #define DEBUG__________(...)            Serial.print(__VA_ARGS__)
  #define DEBUG__________printf(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
  #define DEBUG__________ln(...)
  #define DEBUG__________(...)
  #define DEBUG__________printf(fmt, ...)
#endif

                                                
/*                                                                                                 
                                     .-+***+-:....                                                  
                                      .+@@@@@@@@@+:......:::.....                                   
                                        .=@@@@@@@@@@@@@@@@@@@@@@@@@@+:.                             
                                         .-@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%*:...                      
                                      ..=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#:.                     
                                    ..#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+..                  
                                  ..#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+.                 
                                 .*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#.                
                              ..-@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ .@@@@@@@@@+...             
                              .+@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+..           
                             .#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*.           
                           ..#@@@@@@@@@@@@@@@@@@@%#+:......+@@@@@@@-.....................           
                           .#@@@@@@@@@@@@@@@@=....         -@@@@@@%.                                
                          .+@@@@@@@@@@@@@*:               .#@@@@@%..                                
                         .:@@@@@@@@@@@%:..              ..*@@@@@=..                                 
                         .@@@@@@@@@@%.                  .=@@#:                                      
                         +@@@@@@@@%-.                                                               
                       ..%@@@@@@@+.                                                                 
                       .=@@@@@@%:                                                                   
                       .@@@@@@+..                                                                   
                       :@@@@@=.                                                                     
                      .=@@@@=                                                                       
                  ..*%@@@@@@-..                                                                     
             ...-#@@@@@@@@@@@@=.                                                                    
            ..-@@@@@@@@@@@@@@@@=                                                                    
            .%@@@%=:......-#@@@%.                                                                   
           .*%-..          ..+@@.                                                                   
           ..                .:=.                                                                                                                                                                  
*/
//

#define I2C2_SDA          40
#define I2C2_SCL          41

#if defined(BOTONERA_OLD)
  #define I2S_WS   11 //16 //orig 16
  #define I2S_SD   12 //47 //orig 47
  #define I2S_SCK  13 //48 //orig 48

#elif defined(BOTONERA_NEW)
  #define I2S_WS   16 //16 //orig 16
  #define I2S_SD   47 //47 //orig 47
  #define I2S_SCK  48 //48 //orig 48
#endif

#if defined(DOIT_LITE)
  #if defined(BOTONERA_NEW)
    #define ENC_A      34
    #define ENC_B      33
    #if defined(DEVKIT)
      #define ENC_BUTTON 7
    #else
      #define ENC_BUTTON 33
    #endif
  #elif defined(BOTONERA_OLD)
    #define ENC_A      26
    #define ENC_B      34
    #if defined(DEVKIT)
      #define ENC_BUTTON 7
    #else
      #define ENC_BUTTON 33
    #endif
  #endif
#endif

#if defined(DOIT_LITE)
  #if defined(DEVKIT)
    // Cableado Lite devkit actual.
    #define RF_TX_PIN       17
    #define RF_RX_PIN       18
  #else
    // PCB personalizada: TX del ESP32 hacia RX del modulo, RX desde TX del modulo.
    #define RF_TX_PIN       18
    #define RF_RX_PIN       17
  #endif
#else
  #define RF_TX_PIN       18
  #define RF_RX_PIN       17
#endif
#define RF_CONFIG_PIN     46 //botonera marc pin 45
#ifndef FAST_RF
  #define FAST_RF
#endif
#if defined (FAST_RF)
  #define RF_BAUD_RATE      115200
#else
  #define RF_BAUD_RATE      9600
#endif

#define UART_RX_BUFFER_SIZE 1024


#define SAMPLES             256
#define SAMPLING_FREQUENCY  44100


#define MIC_SENS 1500

// DEFINES FRAME
#define NEW_START             0xE1
#define OLD_START             0x3A
#define NEW_END               0xBB
#define OLD_END               0x3F
#define BROADCAST             0xFF  
#define OLD_FRAME             0x0F
#define NEW_FRAME             0x01
#define INVALID_FRAME         0x00  
#define UNDEF_DEVICE_ID       0x00  
#define DEFAULT_BOTONERA      0xDB  
#define DEFAULT_CONSOLE       0xDC 
#define DEFAULT_DICE          0xDA
#define DEFAULT_DEVICE        0xDD 

#define DEFAULT_ERROR_ID      0xDE
#define DEFAULT_TECH_TOOL_ID  0xDF


#define F_REQ_ELEM_SECTOR     0xA0
#define L_REQ_ELEM_SECTOR     0x02


#define F_SET_ELEM_ID         0xB1
#define L_SET_ELEM_ID         0x01
#define F_SET_ELEM_MODE       0xB2
#define L_SET_ELEM_MODE       0x01
#define F_SET_ELEM_DEAF       0xB3
#define L_SET_ELEM_DEAF       0x01


#define F_SEND_RESPONSE           0xC5
#define L_SEND_RESPONSE           0x01
#define F_SEND_COLOR              0xC1
#define L_SEND_COLOR              0x01 
#define F_SEND_RGB                0xC2
#define L_SEND_RGB                0x03
#define F_SEND_BRIGHTNESS         0xC3
#define L_SEND_BRIGHTNESS         0x02
#define F_SET_BUTTONS_INDEXED_RGB 0xC5
#define L_SET_BUTTONS_INDEXED_RGB 0x04
#define F_SET_BUTTONS_EXTMAP      0xC7
#define BTN_FLAG_RGB              0x01
#define BTN_FLAG_GLOBAL           0x80
#define F_SEND_SENSOR_VALUE_1     0xCA
#define L_SEND_SENSOR_VALUE_1     0x0C
#define F_SEND_SENSOR_VALUE_2     0xCB
#define L_SEND_SENSOR_VALUE_2     0x06
#define F_SEND_FILE_NUM           0xCC
#define L_SEND_FILE_NUM           0x02
#define F_SEND_PATTERN_NUM        0xCD  
#define L_SEND_PATTERN_NUM        0x01  
#define F_SEND_FLAG_BYTE          0xCE
#define L_SEND_FLAG_BYTE          0x01
#define F_SEND_COMMAND            0xCF
#define L_SEND_COMMAND            0x01

#define F_RETURN_ELEM_SECTOR     0xD0
#define L_RETURN_ELEM_SECTOR_01  0x01
#define L_RETURN_ELEM_SECTOR_02  0x02
#define L_RETURN_ELEM_SECTOR_03  0x03
#define L_RETURN_ELEM_SECTOR_04  0x04
#define L_RETURN_ELEM_SECTOR_05  0x05
#define L_RETURN_ELEM_SECTOR_24  0x18
#define L_RETURN_ELEM_SECTOR_64  0x40
#define L_RETURN_ELEM_SECTOR_128 0x80
#define L_RETURN_ELEM_SECTOR_192 0xC0


#define F_RETURN_ELEM_ERROR      0xDE
#define L_RETURN_ELEM_ERROR      0x60 


#define MAX_BRIGHTNESS        0xFF
#define MID_BRIGHTNESS        0x7F
#define OLD_COLOR_FUNCTION    0xCB
#define OLD_RELAY_FUNCTION    0xD3
#define NO_COLOR              0x00

#define WIN             1
#define FAIL            0
#define TRY_AGAIN       2

// Definiciones de cosas del manejo de los tramas y esas cosas
// NO AGREGAR RES sense consultar a los dioses del Olimpo
#define MAX_BYTES_PER_INTERRUPT 0xFF
#define MAX_FRAME_LENGTH        0xFFFF
#define MIN_FRAME_LENGTH        0x15    // Trama mínima: START + LEN + encabezado (origin..checksum) + END
#define MAX_BUFFER_SIZE         0xFF
#define MAX_ALLOWED_TARGETS     0xFF
#define MAX_VALID_DEVICE_ID     0xFF

#define MAX_INTERRUPT_DURATION   0x10
#define MAX_FRAME_RECEPTION_TIME 0x64
#define FRAME_RECEPTION_TIMEOUT  0x0F

// COLORES 

#define WHITE                    0x00  //0xFFFFF0
#define YELLOW                   0x01  //0xFF9B00
#define ORANGE                   0x02  //0xFF4600
#define RED                      0x03  //0xFF0000
#define VIOLET                   0x04  //0xFF00D2
#define BLUE                     0x05  //0x0000FF
#define LIGHT_BLUE               0x06  //0x00FFC8
#define GREEN                    0x07  //0x00FF00
#define BLACK                    0x08  //0x000000
#define NEGRO                    0x00
#define RELAY                    0x09
////////////////////////////////
#define CREMA                    0x0F //0x806600 //0x8b4513 //0xFEFBEA //0xffcd80
#define ROSA                     0x1F //0xFF00D2 //0xFF8080
#define LILA                     0x1E //0x8F00FF //0xFF80E9 
#define CELESTE_CLARO            0x1B //0x5080FF //0x8080FF
#define TURQUESA                 0x19 //0x40E0D0 //0x80FFE4
#define VERDE_CLARO              0x10 //0x66FF00 //0x80FF80
#define NARANJA                  0x02 //0xFF4500 //0xFF7F00
#define ROSA_OSCURO              0x20 //0xFF1493 //0xFF4FD1
#define VERDE_OLIVA              0x12 //0x758A00 //0x808000 //0x7F7F80
#define VERDE_LIMA               0x15 //0x7FFF64 //0x7FFF64
#define VERDE_AMARILLO           0x0E //0xFFDF00 //0x7FFF00
#define MAGENTA                  0x22 //0xD9017A //0xFF0081
#define MORADO                   0x1D //0x9400d3 //0x9F00C5 //0x9B26B6 //0x800080
#define VERDE_AZULADO            0x17 //0x0DBA98 //0x80FFC4
#define MARRON                   0x0B //0xff4005 //0x8b4513
#define INDIGO                   0x1C //0x330099 //0x8000F1
#define AZUL_VERDOSO             0x18 //0x1f3438 //0x80FFD5
#define ESMERALDA                0x16 //0x50C878 //0x80FF81
#define CYAN                     0x1A //0x00FFFF
#define VERDE_MENTA              0x15 //0x98FF98 //0x80FFC8
#define AMARILLO_ANARANJADO      0x0D //0xFF7E00 //0xFFA300
#define ROJO_ANARANJADO          0x09 //0xFF2300
#define ROSA_FUERTE              0x21 //0xe4007c //0xFF23D1
#define MARRON_OSCURO            0x0A //0xff2500 //0x7F2380
#define VERDE_GRISACEO           0x13 //0x7FA364
#define VERDE_OLIVA_CLARO        0x11 //0x7FA300
#define SALMON                   0x23 //0xe73410
//

#define ICON_COLUMNS      64
#define ICON_ROWS         64
#define ICON_LENGTH      ICON_COLUMNS*ICON_ROWS

// BANKS & FILES
#define RESERVED_BANK   1
#define SYSTEM_FX_BANK  99

#define WIN_RESP_BANK   2
#define FAIL_RESP_BANK  4
#define AMBIENTS_BANK   8

#define WOMAN_VOICE     0
#define MAN_VOICE       1
// DEFINES INFO_ELEMENTS
// #define SPANISH_LANG        0x01
// #define ENGLISH_LANG        0x02
// #define FRENCH_LANG         0x03
// #define GERMAN_LANG         0x04
// #define CATALAN_LANG        0x05
// #define MEXICAN_LANG        0x06
// #define EUSKERA_LANG        0x07

#define SPANISH_FILE_OFFSET   10
#define ENGLISH_FILE_OFFSET   20
#define GERMAN_FILE_OFFSET    30
#define FRENCH_FILE_OFFSET    40
#define MEXICAN_FILE_OFFSET   50
#define CATALAN_FILE_OFFSET   60
#define EUSKERA_FILE_OFFSET   70

#define MAN_VOICE_BANK_OFFSET  1

#define MSB 0x00
#define LSB 0x01

#define ELEM_NAME           0x01
#define ELEM_DESC           0x02
#define ELEM_MODE_0_NAME    0x03
#define ELEM_MODE_0_DESC    0x04
#define ELEM_MODE_1_NAME    0x05
#define ELEM_MODE_1_DESC    0x06
#define ELEM_MODE_2_NAME    0x07
#define ELEM_MODE_2_DESC    0x08
#define ELEM_MODE_3_NAME    0x09
#define ELEM_MODE_3_DESC    0x0A
#define ELEM_MODE_4_NAME    0x0B
#define ELEM_MODE_4_DESC    0x0C
#define ELEM_MODE_5_NAME    0x0D
#define ELEM_MODE_5_DESC    0x0E
#define ELEM_MODE_6_NAME    0x0F
#define ELEM_MODE_6_DESC    0x10
#define ELEM_MODE_7_NAME    0x11
#define ELEM_MODE_7_DESC    0x12
#define ELEM_MODE_8_NAME    0x13
#define ELEM_MODE_8_DESC    0x15
#define ELEM_MODE_9_NAME    0x15
#define ELEM_MODE_9_DESC    0x16
#define ELEM_MODE_10_NAME   0x17
#define ELEM_MODE_10_DESC   0x18
#define ELEM_MODE_11_NAME   0x19
#define ELEM_MODE_11_DESC   0x1A
#define ELEM_MODE_12_NAME   0x1B
#define ELEM_MODE_12_DESC   0x1C
#define ELEM_MODE_13_NAME   0x1D
#define ELEM_MODE_13_DESC   0x1E
#define ELEM_MODE_14_NAME   0x1F    
#define ELEM_MODE_14_DESC   0x20    
#define ELEM_MODE_15_NAME   0x21
#define ELEM_MODE_15_DESC   0x22


#define DEFAULT_BASIC_MODE  0x00

#define TYPE_BOTONERA       0xAB
#define TYPE_COLUMN         0x01
#define TYPE_LIGHTSOURCE    0x02
#define TYPE_LEDSTRIP       0x03
#define TYPE_ESCALERA       0x04
#define TYPE_BLACKLIGHT     0x05
#define TYPE_FAN            0x06
#define TYPE_TUNNEL         0x07
#define TYPE_DOT_WALL       0x08
#define TYPE_STORM_PANEL    0x09
#define TYPE_STAR_CEILING   0x0A
#define TYPE_SHOWER         0x0B
#define TYPE_CURTAIN        0x0C
#define GENERIC_TYPE        0x00 

// TESTS
enum COMMANDS_ { 
  BLACKOUT= 0,
  START_CMD,
  SLEEP_SERIAL_WAKEUP_CMD,
  SLEEP_RANDOM_WAKEUP_CMD,
  SEND_REG_RF_CMD,
  SEND_STATS_RF_CMD,
  ERR_DBG_ON,
  ERR_DBG_OFF,
  SET_ELEM_DEAF,
  SET_ELEM_LONG_DEAF,
  MAGIC_TEST_CMD,
  MAGIC_TEST_2_CMD,
  ALTERNATE_MODE_ON,
  ALTERNATE_MODE_OFF,
  OTA_AP_ON,
  OTA_AP_OFF,
  COG_ACT_ON,
  COG_ACT_OFF,
  SHOW_ID_CMD, 
  WIN_CMD,
  FAIL_CMD,
  MOST_USED_MODE_RF_REQ,
  MOST_USED_COLOR_RF_REQ,
  MOST_USED_AMBIENT_RF_REQ,
  LIFETIME_TOTAL_RF_REQ,
  WORKTIME_TOTAL_RF_REQ,
  CURRENT_SESSION_TIME_RF_REQ,
  CURRENT_SESSION_FILENAME_RF_REQ,
  EVENTS_LOG_RF_REQ,
  LAST_EVENT_LOG_RF_REQ,
  LIST_SESSIONS_RF_REQ,
  CLEAR_SESSIONS_CMD,
  RESET_ALL_STATS_CMD,
  SEND_LAST_ORIGIN_CMD,
  SEND_SESSION_LOG_RF_CMD,
  FORMAT_LITTLEFS_CMD,
  AVERAGE_SESSION_TIME_RF_REQ,
  MOST_SELECTED_MODE_RF_REQ,
  MOST_SELECTED_COLOR_RF_REQ,
  MOST_SELECTED_AMBIENT_RF_REQ,
  USAGE_GRAPH_RF_REQ,
  LITTLEFS_MEM_STATS,
  INTER_SESSION_TIMES
};

enum TOKEN_TYPE_{
  TOKEN_FX, 
  TOKEN_NOFX
};

enum TOKEN_CONFIG_{
  TEMP_COLOR_CONF,
  PERM_COLOR_CONF,
  NO_COLOR_CONF,
  AMBIENT_CONF
};

#define RELAY_1_FLAG     0x00
#define RELAY_2_FLAG     0x01
#define RELAY_3_FLAG     0x02
#define RELAY_4_FLAG     0x03

#define SET_RELAY        0x01
#define RESET_RELAY      0x00

// DEFINES COLUMNA
#define COLUMN_LED_DATA_PIN 21
#define COLUMN_RELAY_PIN    42

// DEFINES FIBRAS
#define LIGHTSOURCE_LED_DATA_PIN    21 
#define LIGHTSOURCE_FAN_RELAY_PIN   42

// DEFINES LEDSTRIPS
#define LEDSTRIP_LED_DATA_PIN       46 // 21= oficial 0 45 per tires super long

// DEFINES BOTONERA
#if defined(DEVKIT)
  #define BOTONERA_DATA_PIN         35
#else
  #define BOTONERA_DATA_PIN         21
#endif

//DEFINES ESCALERA
#define VUMETER_SEL_MODE_PIN_01   1
#define VUMETER_SEL_MODE_PIN_02   2
#define VUMETER_SEL_MODE_PIN_03   3
#define VUMETER_SEL_MODE_PIN_04   4
#define VUMETER_SEL_MODE_PIN_05   5
#define VUMETER_SEL_MODE_PIN_06   6


#define VUMETER_LED_DATA_PIN        21
#if defined(DEVKIT)
  #define BOTONERA_DATA_PIN         35
#else
  #define BOTONERA_DATA_PIN         21
#endif

//DEFINES ESCALERA
#define VUMETER_SEL_MODE_PIN_01   1
#define VUMETER_SEL_MODE_PIN_02   2
#define VUMETER_SEL_MODE_PIN_03   3
#define VUMETER_SEL_MODE_PIN_04   4
#define VUMETER_SEL_MODE_PIN_05   5
#define VUMETER_SEL_MODE_PIN_06   6


#define VUMETER_LED_DATA_PIN        21

#if   defined (COLUMNA)
  #define NUM_STEPS  1 
  #define LEDS_STEP  1
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
  #define NUM_STEPS  1 
  #define LEDS_STEP  1
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
#elif defined (FIBRAS)
  #define NUM_STEPS  1 
  #define LEDS_STEP  1
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
  #define NUM_STEPS  1 
  #define LEDS_STEP  1
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
#elif defined (WALLWASHER)
  #define NUM_STEPS  1 
  #define LEDS_STEP  576
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
#elif defined (ESCALERA)
  #define NUM_STEPS  10 // 11
  #define LEDS_STEP  27
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
// botonera sempre al final dels elifs
  #define NUM_STEPS  1 
  #define LEDS_STEP  576
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
#elif defined (ESCALERA)
  #define NUM_STEPS  10 // 11
  #define LEDS_STEP  27
  #define NUM_LEDS   NUM_STEPS*LEDS_STEP 
// botonera sempre al final dels elifs
#elif defined (BOTONERA)
  #define NUM_LEDS 9
  #define NUM_STEPS  1 
  #define LEDS_STEP  1
#endif




enum COLUMN_MODE_LIST{
    COLUMN_CONTEST_MODE= 0,
    COLUMN_BASIC_MODE,
    COLUMN_SLOW_MODE,
    COLUMN_MOTION_LIGHT_MODE,
    COLUMN_MOTION_COLOR_MODE,
    COLUMN_MIX_MODE,
    COLUMN_PULSE_MODE,
    COLUMN_PASSIVE_MODE,
    COLUMN_SLOW_PASSIVE_MODE,
    COLUMN_VOICE_LIGHT_MODE,
    COLUMN_VOICE_REVERSE_LIGHT_MODE,
    COLUMN_VOICE_COLOR_MODE,
    COLUMN_VOICE_BUBBLES_MODE,
    COLUMN_VOICE_REVERSE_BUBBLES_MODE,
    COLUMN_PATTERN_MODE 
};

enum LIGHTSOURCE_MODE_LIST{
  LIGHTSOURCE_CONTEST_MODE= 0,
  LIGHTSOURCE_BASIC_MODE,
  LIGHTSOURCE_SLOW_MODE,
  LIGHTSOURCE_MOTION_LIGHT_MODE,
  LIGHTSOURCE_MOTION_COLOR_MODE,
  LIGHTSOURCE_MIX_MODE,
  LIGHTSOURCE_PULSE_MODE,
  LIGHTSOURCE_PASSIVE_MODE,
  LIGHTSOURCE_SLOW_PASSIVE_MODE,
  LIGHTSOURCE_VOICE_LIGHT_MODE,
  LIGHTSOURCE_VOICE_REVERSE_LIGHT_MODE,
  LIGHTSOURCE_VOICE_COLOR_MODE,
  LIGHTSOURCE_VOICE_BUBBLES_MODE,
  LIGHTSOURCE_VOICE_REVERSE_BUBBLES_MODE,
  LIGHTSOURCE_PATTERN_MODE 
};

enum LEDSTRIP_MODE_LIST{
    LEDSTRIP_HIDDEN_MODE= 0,
    LEDSTRIP_BASIC_MODE,
    LEDSTRIP_SLOW_MODE,
    LEDSTRIP_MOTION_MODE,
    LEDSTRIP_RB_MOTION_MODE,
    LEDSTRIP_MIX_MODE,
    LEDSTRIP_PASSIVE_MODE,
    LEDSTRIP_PATTERN_MODE,
    LEDSTRIP_MIC_MODE
};

enum VUMETER_MODE_LIST{
    VUMETER_HIDDEN_MODE= 0,
    VUMETER_BASIC_MODE,
    VUMETER_SLOW_MODE,
    VUMETER_MOTION_MODE,
    VUMETER_RB_MOTION_MODE,
    VUMETER_MIX_MODE,
    VUMETER_PASSIVE_MODE,
    VUMETER_PATTERN_MODE,
    VUMETER_MODE_8,
    VUMETER_MODE_9,
    VUMETER_SIMON_GAME_MODE,
    VUMETER_SECUENCER_GAME_MODE,
    VUMETER_SPEAK_GAME_MODE,
    VUMETER_BLOCK_SPEAK_MODE,
    VUMETER_TONE_DETECT_MODE,
    VUMETER_METEOR_VOICE_MODE
};

enum PATTERN_LIST{
    NO_PATTERN= 0,
    COLOR_PATT,
    FIRE_PATT,
    METEOR_PATT,
    BOUNCING_PATT, 
    RAINBOW_PATT,
    SNOW_PATT,
    CLOUD_PATT
};

enum POSITION{

  GROUND,
  WALL_H,
  WALL_V,
  CEILING
};

enum SECTOR_LIST{

  ELEM_NAME_SECTOR, // = 0
  ELEM_DESC_SECTOR,
  ELEM_LOCATION_SECTOR,
  ELEM_SERIAL_SECTOR,
  ELEM_UNUSED_SECTOR,
  ELEM_CMODE_SECTOR,
  ELEM_MODE_0_NAME_SECTOR,
  ELEM_MODE_0_DESC_SECTOR,
  ELEM_MODE_0_FLAG_SECTOR,
  ELEM_MODE_1_NAME_SECTOR,
  ELEM_MODE_1_DESC_SECTOR,
  ELEM_MODE_1_FLAG_SECTOR,
  ELEM_MODE_2_NAME_SECTOR,
  ELEM_MODE_2_DESC_SECTOR,
  ELEM_MODE_2_FLAG_SECTOR,
  ELEM_MODE_3_NAME_SECTOR,
  ELEM_MODE_3_DESC_SECTOR,
  ELEM_MODE_3_FLAG_SECTOR,
  ELEM_MODE_4_NAME_SECTOR,
  ELEM_MODE_4_DESC_SECTOR,
  ELEM_MODE_4_FLAG_SECTOR,
  ELEM_MODE_5_NAME_SECTOR,
  ELEM_MODE_5_DESC_SECTOR,
  ELEM_MODE_5_FLAG_SECTOR,
  ELEM_MODE_6_NAME_SECTOR,
  ELEM_MODE_6_DESC_SECTOR,
  ELEM_MODE_6_FLAG_SECTOR,
  ELEM_MODE_7_NAME_SECTOR,
  ELEM_MODE_7_DESC_SECTOR,
  ELEM_MODE_7_FLAG_SECTOR,
  ELEM_MODE_8_NAME_SECTOR,
  ELEM_MODE_8_DESC_SECTOR,
  ELEM_MODE_8_FLAG_SECTOR,
  ELEM_MODE_9_NAME_SECTOR,
  ELEM_MODE_9_DESC_SECTOR,
  ELEM_MODE_9_FLAG_SECTOR,
  ELEM_MODE_10_NAME_SECTOR,
  ELEM_MODE_10_DESC_SECTOR,
  ELEM_MODE_10_FLAG_SECTOR,
  ELEM_MODE_11_NAME_SECTOR,
  ELEM_MODE_11_DESC_SECTOR,
  ELEM_MODE_11_FLAG_SECTOR,
  ELEM_MODE_12_NAME_SECTOR,
  ELEM_MODE_12_DESC_SECTOR,
  ELEM_MODE_12_FLAG_SECTOR,
  ELEM_MODE_13_NAME_SECTOR,
  ELEM_MODE_13_DESC_SECTOR,
  ELEM_MODE_13_FLAG_SECTOR,
  ELEM_MODE_14_NAME_SECTOR,
  ELEM_MODE_14_DESC_SECTOR,
  ELEM_MODE_14_FLAG_SECTOR,
  ELEM_MODE_15_NAME_SECTOR,
  ELEM_MODE_15_DESC_SECTOR,
  ELEM_MODE_15_FLAG_SECTOR, //53
  ELEM_ICON_ROW_0_SECTOR,
  ELEM_ICON_ROW_1_SECTOR,
  ELEM_ICON_ROW_2_SECTOR,
  ELEM_ICON_ROW_3_SECTOR,
  ELEM_ICON_ROW_4_SECTOR,
  ELEM_ICON_ROW_5_SECTOR,
  ELEM_ICON_ROW_6_SECTOR,
  ELEM_ICON_ROW_7_SECTOR,
  ELEM_ICON_ROW_8_SECTOR,
  ELEM_ICON_ROW_9_SECTOR,
  ELEM_ICON_ROW_10_SECTOR,
  ELEM_ICON_ROW_11_SECTOR,
  ELEM_ICON_ROW_12_SECTOR,
  ELEM_ICON_ROW_13_SECTOR,
  ELEM_ICON_ROW_14_SECTOR,
  ELEM_ICON_ROW_15_SECTOR,
  ELEM_ICON_ROW_16_SECTOR,
  ELEM_ICON_ROW_17_SECTOR,
  ELEM_ICON_ROW_18_SECTOR,
  ELEM_ICON_ROW_19_SECTOR,
  ELEM_ICON_ROW_20_SECTOR,
  ELEM_ICON_ROW_21_SECTOR,
  ELEM_ICON_ROW_22_SECTOR,
  ELEM_ICON_ROW_23_SECTOR,
  ELEM_ICON_ROW_24_SECTOR,
  ELEM_ICON_ROW_25_SECTOR,
  ELEM_ICON_ROW_26_SECTOR,
  ELEM_ICON_ROW_27_SECTOR,
  ELEM_ICON_ROW_28_SECTOR,
  ELEM_ICON_ROW_29_SECTOR,
  ELEM_ICON_ROW_30_SECTOR,
  ELEM_ICON_ROW_31_SECTOR,
  ELEM_ICON_ROW_32_SECTOR,
  ELEM_ICON_ROW_33_SECTOR,
  ELEM_ICON_ROW_34_SECTOR,
  ELEM_ICON_ROW_35_SECTOR,
  ELEM_ICON_ROW_36_SECTOR,
  ELEM_ICON_ROW_37_SECTOR,
  ELEM_ICON_ROW_38_SECTOR,
  ELEM_ICON_ROW_39_SECTOR,
  ELEM_ICON_ROW_40_SECTOR,
  ELEM_ICON_ROW_41_SECTOR,
  ELEM_ICON_ROW_42_SECTOR,
  ELEM_ICON_ROW_43_SECTOR,
  ELEM_ICON_ROW_44_SECTOR,
  ELEM_ICON_ROW_45_SECTOR,
  ELEM_ICON_ROW_46_SECTOR,
  ELEM_ICON_ROW_47_SECTOR,
  ELEM_ICON_ROW_48_SECTOR,
  ELEM_ICON_ROW_49_SECTOR,
  ELEM_ICON_ROW_50_SECTOR,
  ELEM_ICON_ROW_51_SECTOR,
  ELEM_ICON_ROW_52_SECTOR,
  ELEM_ICON_ROW_53_SECTOR,
  ELEM_ICON_ROW_54_SECTOR,
  ELEM_ICON_ROW_55_SECTOR,
  ELEM_ICON_ROW_56_SECTOR,
  ELEM_ICON_ROW_57_SECTOR,
  ELEM_ICON_ROW_58_SECTOR,
  ELEM_ICON_ROW_59_SECTOR,
  ELEM_ICON_ROW_60_SECTOR,
  ELEM_ICON_ROW_61_SECTOR,
  ELEM_ICON_ROW_62_SECTOR,
  ELEM_ICON_ROW_63_SECTOR, // = 117
  ELEM_MOST_USED_MODE_SECTOR,
  ELEM_MOST_USED_COLOR_SECTOR,
  ELEM_MOST_USED_PATTERN_SECTOR,
  ELEM_TOTAL_SESSION_TIME_SECTOR,
  ELEM_FLAG_STATE_SECTOR,
  ELEM_CURRENT_COLOR_SECTOR,
  ELEM_CURRENT_RED_SECTOR,
  ELEM_CURRENT_GREEN_SECTOR,
  ELEM_CURRENT_BLUE_SECTOR,
  ELEM_CURRENT_BRIGHTNESS_SECTOR,
  ELEM_CURRENT_FLAGS_SECTOR,
  ELEM_CURRENT_PATTERN_SECTOR,
  ELEM_CURRENT_FILE_SECTOR,
  ELEM_CURRENT_XMANAGER_SECTOR,
  ACTIVE_ELEM_LIST,
  ELEM_INFO,     // nombre + desc
  ELEM_M0_FULL,
  ELEM_M1_FULL,  // nombreM + descM + configM 
  ELEM_M2_FULL,   
  ELEM_M3_FULL,   
  ELEM_M4_FULL,   
  ELEM_M5_FULL,   
  ELEM_M6_FULL,   
  ELEM_M7_FULL,   
  ELEM_M8_FULL,   
  ELEM_M9_FULL,   
  ELEM_M10_FULL,  
  ELEM_M11_FULL,  
  ELEM_M12_FULL,  
  ELEM_M13_FULL,  
  ELEM_M14_FULL,  
  ELEM_M15_FULL,    
  ELEM_ROOM_NS_PACK,
  ELEM_BASIC_CONFIG,
  ELEM_COLORPAD_BTN_MAP
};




#define TT1 &TomThumb

#define FM9 &FreeMono9pt7b
#define FM12 &FreeMono12pt7b
#define FM18 &FreeMono18pt7b
#define FM24 &FreeMono24pt7b

#define FMB9 &FreeMonoBold9pt7b
#define FMB12 &FreeMonoBold12pt7b
#define FMB18 &FreeMonoBold18pt7b
#define FMB24 &FreeMonoBold24pt7b

#define FMO9 &FreeMonoOblique9pt7b
#define FMO12 &FreeMonoOblique12pt7b
#define FMO18 &FreeMonoOblique18pt7b
#define FMO24 &FreeMonoOblique24pt7b

#define FMBO9 &FreeMonoBoldOblique9pt7b
#define FMBO12 &FreeMonoBoldOblique12pt7b
#define FMBO18 &FreeMonoBoldOblique18pt7b
#define FMBO24 &FreeMonoBoldOblique24pt7b

#define FSS9 &FreeSans9pt7b
#define FSS12 &FreeSans12pt7b
#define FSS18 &FreeSans18pt7b
#define FSS24 &FreeSans24pt7b

#define FSSB9 &FreeSansBold9pt7b
#define FSSB12 &FreeSansBold12pt7b
#define FSSB18 &FreeSansBold18pt7b
#define FSSB24 &FreeSansBold24pt7b

#define FSSO9 &FreeSansOblique9pt7b
#define FSSO12 &FreeSansOblique12pt7b
#define FSSO18 &FreeSansOblique18pt7b
#define FSSO24 &FreeSansOblique24pt7b

#define FSSBO9 &FreeSansBoldOblique9pt7b
#define FSSBO12 &FreeSansBoldOblique12pt7b
#define FSSBO18 &FreeSansBoldOblique18pt7b
#define FSSBO24 &FreeSansBoldOblique24pt7b

#define FS9 &FreeSerif9pt7b
#define FS12 &FreeSerif12pt7b
#define FS18 &FreeSerif18pt7b
#define FS24 &FreeSerif24pt7b

#define FSI9 &FreeSerifItalic9pt7b
#define FSI12 &FreeSerifItalic12pt7b
#define FSI19 &FreeSerifItalic18pt7b
#define FSI24 &FreeSerifItalic24pt7b

#define FSB9 &FreeSerifBold9pt7b
#define FSB12 &FreeSerifBold12pt7b
#define FSB18 &FreeSerifBold18pt7b
#define FSB24 &FreeSerifBold24pt7b

#define FSBI9 &FreeSerifBoldItalic9pt7b
#define FSBI12 &FreeSerifBoldItalic12pt7b
#define FSBI18 &FreeSerifBoldItalic18pt7b
#define FSBI24 &FreeSerifBoldItalic24pt7b

#define FF0 NULL //ff0 reserved for GLCD
#define FF1 &FreeMono9pt7b
#define FF2 &FreeMono12pt7b
#define FF3 &FreeMono18pt7b
#define FF4 &FreeMono24pt7b

#define FF5 &FreeMonoBold9pt7b
#define FF6 &FreeMonoBold12pt7b
#define FF7 &FreeMonoBold18pt7b
#define FF8 &FreeMonoBold24pt7b

#define FF9 &FreeMonoOblique9pt7b
#define FF10 &FreeMonoOblique12pt7b
#define FF11 &FreeMonoOblique18pt7b
#define FF12 &FreeMonoOblique24pt7b

#define FF13 &FreeMonoBoldOblique9pt7b
#define FF14 &FreeMonoBoldOblique12pt7b
#define FF15 &FreeMonoBoldOblique18pt7b
#define FF16 &FreeMonoBoldOblique24pt7b

#define FF17 &FreeSans9pt7b
#define FF18 &FreeSans12pt7b
#define FF19 &FreeSans18pt7b
#define FF20 &FreeSans24pt7b

#define FF21 &FreeSansBold9pt7b
#define FF22 &FreeSansBold12pt7b
#define FF23 &FreeSansBold18pt7b
#define FF24 &FreeSansBold24pt7b

#define FF25 &FreeSansOblique9pt7b
#define FF26 &FreeSansOblique12pt7b
#define FF27 &FreeSansOblique18pt7b
#define FF28 &FreeSansOblique24pt7b

#define FF29 &FreeSansBoldOblique9pt7b
#define FF30 &FreeSansBoldOblique12pt7b
#define FF31 &FreeSansBoldOblique18pt7b
#define FF32 &FreeSansBoldOblique24pt7b

#define FF33 &FreeSerif9pt7b
#define FF34 &FreeSerif12pt7b
#define FF35 &FreeSerif18pt7b
#define FF36 &FreeSerif24pt7b

#define FF37 &FreeSerifItalic9pt7b
#define FF38 &FreeSerifItalic12pt7b
#define FF39 &FreeSerifItalic18pt7b
#define FF40 &FreeSerifItalic24pt7b

#define FF41 &FreeSerifBold9pt7b
#define FF42 &FreeSerifBold12pt7b
#define FF43 &FreeSerifBold18pt7b
#define FF44 &FreeSerifBold24pt7b

#define FF45 &FreeSerifBoldItalic9pt7b
#define FF46 &FreeSerifBoldItalic12pt7b
#define FF47 &FreeSerifBoldItalic18pt7b
#define FF48 &FreeSerifBoldItalic24pt7b

#endif
