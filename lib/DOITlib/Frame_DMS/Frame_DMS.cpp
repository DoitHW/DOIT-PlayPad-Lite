#include <defines_DMS/defines_DMS.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>
#include <Arduino.h>

std::vector<uint8_t> printTargetID;
extern LAST_ENTRY_FRAME_T  LEF;

extern byte xManager;
//volatile bool startFrameReceived=    false;
//volatile bool frameInProgress=       false;
volatile bool partialFrameReceived=  false;
volatile bool frameReceived=         false;

std::vector<byte> uartBuffer;
bool BCframe= false;
//volatile unsigned long last_received_time = 0;
//int lastReceivedTime = 0;

//volatile bool uartInterruptTriggered= false;
extern uint16_t SERIAL_NUM;
constexpr uint16_t FRAME_HEADER_BASE_LEN = 18;   // bytes desde room hasta checksum (sin datos)
constexpr uint16_t FRAME_MIN_TOTAL_BYTES = 21;   // trama mí­nima completa (sin datos)
static TARGETNS s_localNS = {0,0,0,0,0};
static uint8_t  s_localRoom = DEFAULT_ROOM;

void setLocalNS(const TARGETNS& ns) {
    s_localNS = ns;
}

const TARGETNS& getLocalNS() {
    return s_localNS;
}

void setLocalRoom(uint8_t room) {
    s_localRoom = room;
}

uint8_t getLocalRoom() {
    return s_localRoom;
}

// void IRAM_ATTR onUartInterrupt() {
//     static enum {
//         WAITING_START,
//         RECEIVING_LENGTH_MSB,
//         RECEIVING_LENGTH_LSB,
//         RECEIVING_FRAME
//     } receiveState = WAITING_START;

//     static uint16_t expectedFrameLength = 0;  // bytes desde origin .. checksum+end
//     static uint16_t totalFrameLength    = 0;  // expected + 3 (start + 2 bytes length)
//     static uint16_t receivedBytes       = 0;
//     static uint16_t calculatedChecksum  = 0;
//     static uint8_t  receivedChecksum    = 0;
//     static unsigned long frameStartTime = 0;

//     uint8_t bytesProcessed = 0;
//     const unsigned long currentTime = millis();

//     while (Serial1.available()) {
//         byte receivedByte = Serial1.read();
//         bytesProcessed++;
//         DEBUG__________((" 0x"+String(receivedByte, HEX)).c_str());
//         //lastReceivedTime = currentTime;

//         // Filtro de START
//         if (receiveState == WAITING_START && receivedByte != NEW_START) {
//             continue;
//         }

//         // Protección de tamaño de buffer/longitud esperada
//         if ((uartBuffer.size() > 0xFF) || (expectedFrameLength > 0xFF)) {
//             receiveState = WAITING_START;
//             uartBuffer.clear();
//             expectedFrameLength = 0;
//             return;
//         }

//         switch (receiveState) {
//             case WAITING_START: {
//                 // Inicio de una nueva trama
//                 uartBuffer.reserve(MAX_FRAME_LENGTH);
//                 uartBuffer.clear();
//                 uartBuffer.push_back(receivedByte);

//                 calculatedChecksum = receivedByte; // incluye START en la suma, como haces ahora
//                 receiveState       = RECEIVING_LENGTH_MSB;
//                 expectedFrameLength = 0;
//                 receivedBytes       = 1;
//                 frameStartTime      = currentTime;
//                 break;
//             }

//             case RECEIVING_LENGTH_MSB: {
//                 uartBuffer.push_back(receivedByte);
//                 calculatedChecksum += receivedByte;
//                 while (calculatedChecksum > 0xFF)
//                     calculatedChecksum = (calculatedChecksum & 0xFF) + (calculatedChecksum >> 8);
//                 expectedFrameLength = uint16_t(receivedByte) << 8;
//                 receiveState = RECEIVING_LENGTH_LSB;
//                 break;
//             }

//             case RECEIVING_LENGTH_LSB: {
//                 uartBuffer.push_back(receivedByte);
//                 calculatedChecksum += receivedByte;
//                 while (calculatedChecksum > 0xFF)
//                     calculatedChecksum = (calculatedChecksum & 0xFF) + (calculatedChecksum >> 8);

//                 expectedFrameLength |= receivedByte;      // len = origin..checksum+end
//                 totalFrameLength = expectedFrameLength + 3; // + start + 2 bytes de len

//                 if (totalFrameLength < MIN_FRAME_LENGTH) {
//                     // Longitud inválida 
//                     receiveState = WAITING_START;
//                     uartBuffer.clear();
//                     return;
//                 }

//                 receiveState = RECEIVING_FRAME;
//                 receivedBytes = 3; // ya hemos leí­do start + 2 len
//                 break;
//             }

//             case RECEIVING_FRAME: {
//                 uartBuffer.push_back(receivedByte);
//                 receivedBytes++;

//                 // Sumamos TODO menos el byte de checksum (penúltimo)
//                 if (receivedBytes != totalFrameLength - 1) {
//                     calculatedChecksum += receivedByte;
//                 } else {
//                     receivedChecksum = receivedByte; // penúltimo byte
//                 }
//                 while (calculatedChecksum > 0xFF)
//                     calculatedChecksum = (calculatedChecksum & 0xFF) + (calculatedChecksum >> 8);

//                 // ¿Hemos cerrado la trama?
//                 if (receivedBytes == totalFrameLength) {
//                     // Último byte debe ser NEW_END
//                     if (receivedByte == NEW_END) {
//                         if (calculatedChecksum == receivedChecksum) {
//                             // ========= NUEVO BLOQUE DE DESTINATARIO =========
//                             // Con el layout actual:
//                             // [10] targetType
//                             // [11..15] targetNS (5 bytes)
//                             bool acceptFrame = false;
//                             bool isBCFrame   = false;
//                             printTargetID.clear();

//                             if (uartBuffer.size() >= FRAME_MIN_TOTAL_BYTES) {
//                                 const uint8_t frameRoom  = uartBuffer[3];
//                                 const uint8_t localRoom  = getLocalRoom();
//                                 const bool    roomMatches = (frameRoom == ROOM_BROADCAST) || (frameRoom == localRoom);
//                                 const uint8_t targetType  = uartBuffer[10];

//                                 if (roomMatches) {
//                                     for (int i = 0; i < 5; ++i) {
//                                         printTargetID.push_back(uartBuffer[11 + i]);
//                                     }

//                                     // Reglas de targeting para la BOTONERA:
//                                     // - Broadcast -> siempre nos aplica (dentro de la misma sala o broadcast de sala)
//                                     // - TargetType BOTONERA (0xDB) -> nos aplica
//                                     if (targetType == BROADCAST) {
//                                         acceptFrame = true;
//                                         isBCFrame   = true;
//                                     } else if (targetType == DEFAULT_BOTONERA) {
//                                         acceptFrame = true;
//                                     }
//                                 }
//                             }

//                             if (acceptFrame) {
//                                 frameReceived = true;
//                                 BCframe       = isBCFrame;
//                             } else {
//                                 frameReceived = false;
//                                 BCframe       = false;
//                             }
//                             // ========= FIN BLOQUE DESTINATARIO =========

//                         } else {
//                             Serial.println("Checksum inválido. Ignorar");
//                         }
//                     } else {
//                         Serial1.println("Byte END inválido. Ignorar");
//                     }

//                     // Reset para la siguiente trama
//                     receiveState = WAITING_START;
//                     return;
//                 }

//                 // Protección adicional de buffer
//                 if (uartBuffer.size() >= MAX_BUFFER_SIZE) {
//                     receiveState = WAITING_START;
//                     uartBuffer.clear();
//                     return;
//                 }

//                 break;
//             }
//         } // switch
//     } // while Serial1.available()

//     // Aviso (opcional) de frame parcial
//     if (Serial1.available() && bytesProcessed >= MAX_BYTES_PER_INTERRUPT) {
//         partialFrameReceived = true;
//     }
// }
extern unsigned long lastDisplayInteraction; 

void IRAM_ATTR onUartInterrupt() {
    lastDisplayInteraction = millis(); 
    static enum {
        WAITING_START,
        RECEIVING_LENGTH_MSB,
        RECEIVING_LENGTH_LSB,
        RECEIVING_FRAME
    } receiveState = WAITING_START;

    static uint16_t expectedFrameLength = 0;  // origin..checksum+end
    static uint16_t totalFrameLength    = 0;  // expected + 3 (start + 2 len)
    static uint16_t receivedBytes       = 0;
    static uint16_t calculatedChecksum  = 0;
    static uint8_t  receivedChecksum    = 0;
    static unsigned long frameStartTime = 0;

    uint8_t bytesProcessed = 0;
    const unsigned long currentTime = millis();

    while (Serial1.available()) {
        byte receivedByte = Serial1.read();
        bytesProcessed++;
        DEBUG__________((" 0x"+String(receivedByte, HEX)).c_str());

        // START filter
        if (receiveState == WAITING_START && receivedByte != NEW_START) {
            continue;
        }

        // Protección de tamaños para evitar estados corruptos
        if ((uartBuffer.size() > 0xFF) || (expectedFrameLength > 0xFF)) {
            receiveState        = WAITING_START;
            expectedFrameLength = 0;
            uartBuffer.clear();          // NO libera capacidad
            return;
        }

        switch (receiveState) {

            case WAITING_START: {
                // Inicio de nueva trama
                uartBuffer.clear();                       // capacidad ya pre-reservada en setup
                if (uartBuffer.size() < MAX_FRAME_LENGTH) // push seguro
                    uartBuffer.push_back(receivedByte);

                calculatedChecksum  = receivedByte;       // incluye START
                receiveState        = RECEIVING_LENGTH_MSB;
                expectedFrameLength = 0;
                receivedBytes       = 1;
                frameStartTime      = currentTime;
            } break;

            case RECEIVING_LENGTH_MSB: {
                if (uartBuffer.size() < MAX_FRAME_LENGTH)
                    uartBuffer.push_back(receivedByte);

                calculatedChecksum += receivedByte;
                while (calculatedChecksum > 0xFF)
                    calculatedChecksum = (calculatedChecksum & 0xFF) + (calculatedChecksum >> 8);

                expectedFrameLength  = uint16_t(receivedByte) << 8;
                receiveState         = RECEIVING_LENGTH_LSB;
            } break;

            case RECEIVING_LENGTH_LSB: {
                if (uartBuffer.size() < MAX_FRAME_LENGTH)
                    uartBuffer.push_back(receivedByte);

                calculatedChecksum += receivedByte;
                while (calculatedChecksum > 0xFF)
                    calculatedChecksum = (calculatedChecksum & 0xFF) + (calculatedChecksum >> 8);

                expectedFrameLength |= receivedByte;             // origin..checksum+end
                totalFrameLength     = expectedFrameLength + 3;  // + start + 2 len

                if (totalFrameLength < MIN_FRAME_LENGTH || totalFrameLength > MAX_FRAME_LENGTH) {
                    // Longitud inválida o mayor que nuestro buffer máximo
                    receiveState        = WAITING_START;
                    expectedFrameLength = 0;
                    uartBuffer.clear();
                    return;
                }

                receiveState  = RECEIVING_FRAME;
                receivedBytes = 3; // start + len(2)
            } break;

            case RECEIVING_FRAME: {
                // Guard extra para no rebasar el buffer aun si llega corrupta
                if (uartBuffer.size() < MAX_FRAME_LENGTH)
                    uartBuffer.push_back(receivedByte);
                else {
                    // Overflow defensivo: descarta y resetea
                    receiveState = WAITING_START;
                    uartBuffer.clear();
                    return;
                }

                receivedBytes++;

                // Sumar todo menos penúltimo (checksum)
                if (receivedBytes != totalFrameLength - 1) {
                    calculatedChecksum += receivedByte;
                } else {
                    receivedChecksum = receivedByte; // penúltimo byte
                }
                while (calculatedChecksum > 0xFF)
                    calculatedChecksum = (calculatedChecksum & 0xFF) + (calculatedChecksum >> 8);

                // Fin de trama
                if (receivedBytes == totalFrameLength) {
                    if (receivedByte == NEW_END) {
                        if (calculatedChecksum == receivedChecksum) {
                            // ========= Targeting =========
                            bool acceptFrame = false;
                            bool isBCFrame   = false;

                            // ⚠️ Evita realocaciones aquí: no uses push_back si printTargetID
                            // no tiene capacidad. Pre-resérvalo a 5 en el setup (ver nota abajo).
                            printTargetID.clear();

                            if (uartBuffer.size() >= FRAME_MIN_TOTAL_BYTES) {
                                const uint8_t frameRoom   = uartBuffer[3];
                                const uint8_t localRoom   = getLocalRoom();
                                const bool roomMatches    = (frameRoom == ROOM_BROADCAST) || (frameRoom == localRoom);
                                const uint8_t targetType  = uartBuffer[10];

                                if (roomMatches) {
                                    // Copia directa de 5 bytes (sin crecer capacidad)
                                    if (printTargetID.capacity() >= 5) {
                                        // garantizamos tamaño lógico sin nueva reserva
                                        // (reserve hecho en setup); evita resize si no confías
                                        for (int i = 0; i < 5; ++i) {
                                            printTargetID.push_back(uartBuffer[11 + i]);
                                        }
                                    }

                                    if (targetType == BROADCAST) {
                                        acceptFrame = true;  isBCFrame = true;
                                    } else /*if (targetType == DEFAULT_BOTONERA)*/ {
                                        acceptFrame = true;
                                    }
                                }
                            }

                            if (acceptFrame) { frameReceived = true; BCframe = isBCFrame; }
                            else              { frameReceived = false; BCframe = false;  }

                            // ======= fin targeting =======

                        } else {
                            Serial.println("Checksum inválido. Ignorar");
                        }
                    } else {
                        Serial.println("Byte END inválido. Ignorar");
                    }

                    receiveState = WAITING_START;
                    return;
                }

                // Protección adicional de buffer por seguridad
                if (uartBuffer.size() >= MAX_BUFFER_SIZE) {
                    receiveState = WAITING_START;
                    uartBuffer.clear();
                    return;
                }
            } break;
        } // switch
    } // while

    // Frame parcial (opcional)
    if (Serial1.available() && bytesProcessed >= MAX_BYTES_PER_INTERRUPT) {
        partialFrameReceived = true;
    }
}

byte checksum_calc(const FRAME_T &f) {
    uint16_t sum = 0;

    sum += f.start;
    sum += f.frameLengthMsb;
    sum += f.frameLengthLsb;
    sum += f.room;
    sum += f.origin;
    sum += f.originNS.mac01;
    sum += f.originNS.mac02;
    sum += f.originNS.mac03;
    sum += f.originNS.mac04;
    sum += f.originNS.mac05;
    sum += f.targetType;
    sum += f.targetNS.mac01;
    sum += f.targetNS.mac02;
    sum += f.targetNS.mac03;
    sum += f.targetNS.mac04;
    sum += f.targetNS.mac05;
    sum += f.function;
    sum += f.dataLengthMsb;
    sum += f.dataLengthLsb;
    for (byte b : f.data) sum += b;
    sum += NEW_END;

    while (sum > 0xFF) sum = (sum & 0xFF) + (sum >> 8);
    return (byte)sum;
}

// Layout del frame (nuevo):
// [0]   start (NEW_START)
// [1]   frameLengthMsb
// [2]   frameLengthLsb
// [3]   room
// [4]   origin
// [5..9]originNS (5 bytes)
// [10]  targetType
// [11..15]targetNS (5 bytes)
// [16]  function
// [17]  dataLengthMsb
// [18]  dataLengthLsb
// [19..(19+dataLen-1)] data
// [19+dataLen]   checksum
// [20+dataLen]   end (NEW_END)
LAST_ENTRY_FRAME_T extract_info_from_frameIn(const std::vector<uint8_t> &frame) {
    LAST_ENTRY_FRAME_T result = {};

    // Formato mí­nimo:
    // 0:START, 1:lenMSB, 2:lenLSB,
    // 3:room, 4:origin, 5..9:originNS(5),
    // 10:targetType, 11..15:targetNS(5),
    // 16:function, 17..18:dataLen, 19.. data, checksum, end.
    //     // Mínimo total sin data: 21 bytes
    if (frame.size() < FRAME_MIN_TOTAL_BYTES) return result;

    const size_t IDX_ROOM          = 3;
    const size_t IDX_ORIGIN        = 4;
    const size_t IDX_ORIGIN_NS     = 5;
    const size_t IDX_TARGET_TYPE   = 10;
    const size_t IDX_TARGET_NS     = 11;
    const size_t IDX_FUNCTION      = 16;
    const size_t IDX_DLEN_MSB      = 17;
    const size_t IDX_DLEN_LSB      = 18;
    const size_t IDX_DATA_START    = 19;

    result.room      = frame[IDX_ROOM];
    result.origin    = frame[IDX_ORIGIN];
    result.originNS  = {
        frame[IDX_ORIGIN_NS + 0],
        frame[IDX_ORIGIN_NS + 1],
        frame[IDX_ORIGIN_NS + 2],
        frame[IDX_ORIGIN_NS + 3],
        frame[IDX_ORIGIN_NS + 4]
    };
    result.targetType = frame[IDX_TARGET_TYPE];
    result.targetNS   = {
        frame[IDX_TARGET_NS + 0],
        frame[IDX_TARGET_NS + 1],
        frame[IDX_TARGET_NS + 2],
        frame[IDX_TARGET_NS + 3],
        frame[IDX_TARGET_NS + 4]
    };
    result.function  = frame[IDX_FUNCTION];

    uint16_t dataLength = (static_cast<uint16_t>(frame[IDX_DLEN_MSB]) << 8) |
                           static_cast<uint16_t>(frame[IDX_DLEN_LSB]);

    // Comprobar lí­mites antes de copiar
    if (IDX_DATA_START + dataLength <= frame.size()) {
        result.data.clear();
        result.data.reserve(dataLength);
        result.data.insert(result.data.begin(),
                           frame.begin() + IDX_DATA_START,
                           frame.begin() + IDX_DATA_START + dataLength);
    }

    return result;
}

extern SemaphoreHandle_t uartTxMutex;

#ifdef DOIT_LITE
namespace {
const char *liteFunctionName(byte function) {
  switch (function) {
  case F_REQ_ELEM_SECTOR:
    return "F_REQ_ELEM_SECTOR";
  case F_SET_ELEM_ID:
    return "F_SET_ELEM_ID";
  case F_SET_ELEM_MODE:
    return "F_SET_ELEM_MODE";
  case F_SET_ELEM_DEAF:
    return "F_SET_ELEM_DEAF";
  case F_SEND_COLOR:
    return "F_SEND_COLOR";
  case F_SEND_RGB:
    return "F_SEND_RGB";
  case F_SEND_BRIGHTNESS:
    return "F_SEND_BRIGHTNESS";
  case F_SEND_SENSOR_VALUE_1:
    return "F_SEND_SENSOR_VALUE_1";
  case F_SEND_SENSOR_VALUE_2:
    return "F_SEND_SENSOR_VALUE_2";
  case F_SEND_FILE_NUM:
    return "F_SEND_FILE_NUM";
  case F_SEND_PATTERN_NUM:
    return "F_SEND_PATTERN_NUM";
  case F_SEND_FLAG_BYTE:
    return "F_SEND_FLAG_BYTE";
  case F_SEND_COMMAND:
    return "F_SEND_COMMAND";
  case F_RETURN_ELEM_SECTOR:
    return "F_RETURN_ELEM_SECTOR";
  case F_RETURN_ELEM_ERROR:
    return "F_RETURN_ELEM_ERROR";
  default:
    return "UNKNOWN";
  }
}

void litePrintFrameByte(byte value) {
  Serial.printf("%02X ", value);
}

void litePrintFrameTx(const FRAME_T &f) {
  const uint16_t dataLength =
      (static_cast<uint16_t>(f.dataLengthMsb) << 8) | f.dataLengthLsb;

  Serial.printf("[RF TX] fn=%s(0x%02X) room=0x%02X targetType=0x%02X "
                "targetNS=%02X%02X%02X%02X%02X dataLen=%u data=",
                liteFunctionName(f.function), f.function, f.room, f.targetType,
                f.targetNS.mac01, f.targetNS.mac02, f.targetNS.mac03,
                f.targetNS.mac04, f.targetNS.mac05, dataLength);
  for (byte b : f.data) {
    litePrintFrameByte(b);
  }
  Serial.print("hex=");
  litePrintFrameByte(f.start);
  litePrintFrameByte(f.frameLengthMsb);
  litePrintFrameByte(f.frameLengthLsb);
  litePrintFrameByte(f.room);
  litePrintFrameByte(f.origin);
  litePrintFrameByte(f.originNS.mac01);
  litePrintFrameByte(f.originNS.mac02);
  litePrintFrameByte(f.originNS.mac03);
  litePrintFrameByte(f.originNS.mac04);
  litePrintFrameByte(f.originNS.mac05);
  litePrintFrameByte(f.targetType);
  litePrintFrameByte(f.targetNS.mac01);
  litePrintFrameByte(f.targetNS.mac02);
  litePrintFrameByte(f.targetNS.mac03);
  litePrintFrameByte(f.targetNS.mac04);
  litePrintFrameByte(f.targetNS.mac05);
  litePrintFrameByte(f.function);
  litePrintFrameByte(f.dataLengthMsb);
  litePrintFrameByte(f.dataLengthLsb);
  for (byte b : f.data) {
    litePrintFrameByte(b);
  }
  litePrintFrameByte(f.checksum);
  litePrintFrameByte(f.end);
  Serial.println();
}
} // namespace
#endif

void send_frame(const FRAME_T &f) {
  int  i      = 0;
  byte dTime  = 0;
  
  if (uartTxMutex) { xSemaphoreTake(uartTxMutex, portMAX_DELAY); }

  #ifdef DOIT_LITE
    litePrintFrameTx(f);
  #endif


  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_WHITE) + "======================================" + COLOR_RESET);
    DEBUG__________ln(String(COLOR_BRIGHT_WHITE) + " #### Trama enviada ####" + COLOR_RESET);
  #endif

  // START
  Serial1.write(f.start); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_GREEN) + "[" + String(++i) + "] START = " + String(f.start, HEX) + COLOR_RESET);
  #endif

  // Frame Length MSB
  Serial1.write(f.frameLengthMsb); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_DIM) + "[" + String(++i) + "] Frame Length Msb = " + String(f.frameLengthMsb, HEX) + COLOR_RESET);
  #endif

  // Frame Length LSB
  Serial1.write(f.frameLengthLsb); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_DIM) + "[" + String(++i) + "] Frame Length Lsb = " + String(f.frameLengthLsb, HEX) + COLOR_RESET);
  #endif

  // ROOM
  Serial1.write(f.room); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_RED) + "[" + String(++i) + "] Room = " + String(f.room, HEX) + COLOR_RESET);
  #endif

  // ORIGIN (byte)
  Serial1.write(f.origin); // delay(dTime);
  #ifdef DEBUG
   DEBUG__________ln(String(COLOR_BRIGHT_YELLOW) + "[" + String(++i) + "] Origin = " + String(f.origin, HEX) + COLOR_RESET);
  #endif

  // ORIGIN MAC[0..4]
  Serial1.write(f.originNS.mac01); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_MAGENTA) + "[" + String(++i) + "] Origin MAC[0] = " + String(f.originNS.mac01, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.originNS.mac02); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_MAGENTA) + "[" + String(++i) + "] Origin MAC[1] = " + String(f.originNS.mac02, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.originNS.mac03); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_MAGENTA) + "[" + String(++i) + "] Origin MAC[2] = " + String(f.originNS.mac03, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.originNS.mac04); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_MAGENTA) + "[" + String(++i) + "] Origin MAC[3] = " + String(f.originNS.mac04, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.originNS.mac05); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_MAGENTA) + "[" + String(++i) + "] Origin MAC[4] = " + String(f.originNS.mac05, HEX) + COLOR_RESET);
  #endif

  // TARGET TYPE
  Serial1.write(f.targetType); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_CYAN) + "[" + String(++i) + "] Target Type = " + String(f.targetType, HEX) + COLOR_RESET);
  #endif

  // TARGET MAC[0..4]
  Serial1.write(f.targetNS.mac01); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_BLUE) + "[" + String(++i) + "] MAC[0] = " + String(f.targetNS.mac01, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.targetNS.mac02); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_BLUE) + "[" + String(++i) + "] MAC[1] = " + String(f.targetNS.mac02, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.targetNS.mac03); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_BLUE) + "[" + String(++i) + "] MAC[2] = " + String(f.targetNS.mac03, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.targetNS.mac04); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_BLUE) + "[" + String(++i) + "] MAC[3] = " + String(f.targetNS.mac04, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.targetNS.mac05); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_BLUE) + "[" + String(++i) + "] MAC[4] = " + String(f.targetNS.mac05, HEX) + COLOR_RESET);
  #endif

  // FUNCTION
  Serial1.write(f.function); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_GREEN) + "[" + String(++i) + "] Function = " + String(f.function, HEX) + COLOR_RESET);
  #endif

  // DATA LENGTH MSB/LSB
  Serial1.write(f.dataLengthMsb); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_DIM) + "[" + String(++i) + "] Data Length Msb = " + String(f.dataLengthMsb, HEX) + COLOR_RESET);
  #endif

  Serial1.write(f.dataLengthLsb); // delay(dTime);
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_DIM) + "[" + String(++i) + "] Data Length Lsb = " + String(f.dataLengthLsb, HEX) + COLOR_RESET);
  #endif

  // DATA[i]
  {
    int dataIndex = 0;
    for (byte b : f.data) {
      Serial1.write(b); // delay(dTime);
      #ifdef DEBUG
        DEBUG__________ln(String(COLOR_DIM) + "[" + String(++i) + "] Data[" + String(dataIndex++) + "] = " + String(b, HEX) + COLOR_RESET);
      #endif
    }
  }

  // CHECKSUM
  Serial1.write(f.checksum); // delay(dTime);
  #ifdef DEBUG
     DEBUG__________ln(String(COLOR_BRIGHT_MAGENTA) + "[" + String(++i) + "] Checksum = " + String(f.checksum, HEX) + COLOR_RESET);
  #endif

  // END
  Serial1.write(f.end); // delay(dTime);
  Serial1.flush();
  #ifdef DOIT_LITE
    Serial.printf("[RF TX DONE] UART flushed fn=%s(0x%02X) bytes=%u baud=%lu\n",
                  liteFunctionName(f.function), f.function,
                  static_cast<unsigned>(21 + f.data.size()),
                  static_cast<unsigned long>(RF_BAUD_RATE));
  #endif
  #ifdef DEBUG
    DEBUG__________ln(String(COLOR_BRIGHT_GREEN) + "[" + String(++i) + "] End = " + String(f.end, HEX) + COLOR_RESET);
    DEBUG__________ln(String(COLOR_BRIGHT_WHITE) + "======================================" + COLOR_RESET);
  #endif

  if (uartTxMutex) { xSemaphoreGive(uartTxMutex); }
}

/*
////////////////////////////////////////////////////////////////////////////////////////////
                                           FRAME MAKERS
////////////////////////////////////////////////////////////////////////////////////////////
*/

FRAME_T frameMaker_REQ_ELEM_SECTOR(byte originin, byte targetType, TARGETNS targetNS, byte idiomain, byte sectorin){
    FRAME_T f{};
    constexpr uint16_t DL = L_REQ_ELEM_SECTOR;             // = 0x02 (idioma, sector)
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start           = NEW_START;
    f.frameLengthLsb  = frameLength & 0xFF;
    f.frameLengthMsb  = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS        = getLocalNS();
    f.targetType      = targetType;
    f.targetNS        = targetNS;
    f.function        = F_REQ_ELEM_SECTOR;
    f.dataLengthMsb   = (DL >> 8) & 0xFF;
    f.dataLengthLsb   = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0] = idiomain;
    f.data[1] = sectorin;
    f.checksum        = checksum_calc(f);
    f.end             = NEW_END;
    return f;
}

FRAME_T frameMaker_SET_ELEM_ID(byte originin, byte targetType, TARGETNS targetNS, byte IDin){
    FRAME_T f{};
    constexpr uint16_t DL = L_SET_ELEM_ID;            // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SET_ELEM_ID;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = IDin;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SET_ELEM_MODE(byte originin, byte targetType, TARGETNS targetNS, byte modein){
    FRAME_T f{};
    constexpr uint16_t DL = L_SET_ELEM_MODE;          // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SET_ELEM_MODE;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = modein;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SET_ELEM_DEAF(byte originin, byte targetType, TARGETNS targetNS, byte timein){
    FRAME_T f{};
    constexpr uint16_t DL = L_SET_ELEM_DEAF;          // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SET_ELEM_DEAF;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = timein;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_COLOR(byte originin, byte targetType, TARGETNS targetNS, byte colorin){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_COLOR;             // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_COLOR;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = colorin;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_RGB(byte originin, byte targetType, TARGETNS targetNS, COLOR_T colorin){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_RGB;               // = 3
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_RGB;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = colorin.red;
    f.data[1]        = colorin.green;
    f.data[2]        = colorin.blue;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_SENSOR_VALUE(byte originin, byte targetType, TARGETNS targetNS, SENSOR_DOUBLE_T s){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_SENSOR_VALUE_1;    // = 12
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_SENSOR_VALUE_1;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data = {
        s.msb_min, s.lsb_min, s.msb_max, s.lsb_max, s.msb_val, s.lsb_val,
        s.msb_min2, s.lsb_min2, s.msb_max2, s.lsb_max2, s.msb_val2, s.lsb_val2
    };
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_SENSOR_VALUE_2(byte originin, byte targetType, TARGETNS targetNS, SENSOR_VALUE_T s){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_SENSOR_VALUE_2;    // = 6
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_SENSOR_VALUE_2;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data = { s.msb_min, s.lsb_min, s.msb_max, s.lsb_max, s.msb_val, s.lsb_val };
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_FLAG_BYTE(byte originin, byte targetType, TARGETNS targetNS, byte flagin){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_FLAG_BYTE;         // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_FLAG_BYTE;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = flagin;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_PATTERN_NUM(byte originin, byte targetType, TARGETNS targetNS, byte patternin){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_PATTERN_NUM;       // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_PATTERN_NUM;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = patternin;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_FILE_NUM(byte originin, byte targetType, TARGETNS targetNS, byte bankin, byte filein){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_FILE_NUM;          // = 2
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_FILE_NUM;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = bankin;
    f.data[1]        = filein;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_SEND_COMMAND(byte originin, byte targetType, TARGETNS targetNS, byte commandin){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_COMMAND;           // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin         = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_COMMAND;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = commandin;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

FRAME_T frameMaker_RETURN_ELEM_SECTOR (uint8_t originin,
                                       uint8_t targetType,
                                       const TARGETNS& targetNS,
                                       const uint8_t* sector_data,
                                       uint8_t sectorin) 
{
    FRAME_T frame{};
    frame.start    = NEW_START;
    frame.room     = getLocalRoom();
    frame.origin   = originin;
    frame.originNS = getLocalNS();
    frame.targetType = targetType;
    frame.targetNS   = targetNS;
    frame.function = F_RETURN_ELEM_SECTOR;

    // Primero metemos el identificador de sector en la data
    frame.data.clear();
    frame.data.push_back(sectorin);

    auto appendBytes = [&](int count){
        if (count <= 0) return;
        if (sector_data) {
            frame.data.insert(frame.data.end(), sector_data, sector_data + count);
        } else {
            // Si no hay puntero válido, rellena con ceros
            frame.data.insert(frame.data.end(), count, 0x00);
        }
    };

    // Selecciona longitud de payload por tipo de sector y añade bytes
    switch (sectorin) {
        // 5 bytes
        case ELEM_SERIAL_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_05);
            break;

        // 1 byte
        case ELEM_UNUSED_SECTOR:
        case ELEM_CMODE_SECTOR:
        case ELEM_MOST_USED_MODE_SECTOR:
        case ELEM_MOST_USED_COLOR_SECTOR:
        case ELEM_MOST_USED_PATTERN_SECTOR:
        case ELEM_CURRENT_RED_SECTOR:
        case ELEM_CURRENT_GREEN_SECTOR:
        case ELEM_CURRENT_BLUE_SECTOR:
        case ELEM_CURRENT_BRIGHTNESS_SECTOR:
        case ELEM_CURRENT_PATTERN_SECTOR:
        case ELEM_CURRENT_FLAGS_SECTOR:
        case ELEM_LOCATION_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_01);
            break;

        // 192 bytes
        case ELEM_DESC_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_192);
            break;

        // 24 bytes
        case ELEM_NAME_SECTOR:
        case ELEM_MODE_0_NAME_SECTOR:
        case ELEM_MODE_1_NAME_SECTOR:
        case ELEM_MODE_2_NAME_SECTOR:
        case ELEM_MODE_3_NAME_SECTOR:
        case ELEM_MODE_4_NAME_SECTOR:
        case ELEM_MODE_5_NAME_SECTOR:
        case ELEM_MODE_6_NAME_SECTOR:
        case ELEM_MODE_7_NAME_SECTOR:
        case ELEM_MODE_8_NAME_SECTOR:
        case ELEM_MODE_9_NAME_SECTOR:
        case ELEM_MODE_10_NAME_SECTOR:
        case ELEM_MODE_11_NAME_SECTOR:
        case ELEM_MODE_12_NAME_SECTOR:
        case ELEM_MODE_13_NAME_SECTOR:
        case ELEM_MODE_14_NAME_SECTOR:
        case ELEM_MODE_15_NAME_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_24);
            break;

        // 192 bytes
        case ELEM_MODE_0_DESC_SECTOR:
        case ELEM_MODE_1_DESC_SECTOR:
        case ELEM_MODE_2_DESC_SECTOR:
        case ELEM_MODE_3_DESC_SECTOR:
        case ELEM_MODE_4_DESC_SECTOR:
        case ELEM_MODE_5_DESC_SECTOR:
        case ELEM_MODE_6_DESC_SECTOR:
        case ELEM_MODE_7_DESC_SECTOR:
        case ELEM_MODE_8_DESC_SECTOR:
        case ELEM_MODE_9_DESC_SECTOR:
        case ELEM_MODE_10_DESC_SECTOR:
        case ELEM_MODE_11_DESC_SECTOR:
        case ELEM_MODE_12_DESC_SECTOR:
        case ELEM_MODE_13_DESC_SECTOR:
        case ELEM_MODE_14_DESC_SECTOR:
        case ELEM_MODE_15_DESC_SECTOR:
        case ELEM_ROOM_NS_PACK:
            appendBytes(L_RETURN_ELEM_SECTOR_192);
            break;

        // 2 bytes
        case ELEM_MODE_0_FLAG_SECTOR:
        case ELEM_MODE_1_FLAG_SECTOR:
        case ELEM_MODE_2_FLAG_SECTOR:
        case ELEM_MODE_3_FLAG_SECTOR:
        case ELEM_MODE_4_FLAG_SECTOR:
        case ELEM_MODE_5_FLAG_SECTOR:
        case ELEM_MODE_6_FLAG_SECTOR:
        case ELEM_MODE_7_FLAG_SECTOR:
        case ELEM_MODE_8_FLAG_SECTOR:
        case ELEM_MODE_9_FLAG_SECTOR:
        case ELEM_MODE_10_FLAG_SECTOR:
        case ELEM_MODE_11_FLAG_SECTOR:
        case ELEM_MODE_12_FLAG_SECTOR:
        case ELEM_MODE_13_FLAG_SECTOR:
        case ELEM_MODE_14_FLAG_SECTOR:
        case ELEM_MODE_15_FLAG_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_02);
            break;

        // 128 bytes
        case ELEM_ICON_ROW_0_SECTOR:  case ELEM_ICON_ROW_1_SECTOR:
        case ELEM_ICON_ROW_2_SECTOR:  case ELEM_ICON_ROW_3_SECTOR:
        case ELEM_ICON_ROW_4_SECTOR:  case ELEM_ICON_ROW_5_SECTOR:
        case ELEM_ICON_ROW_6_SECTOR:  case ELEM_ICON_ROW_7_SECTOR:
        case ELEM_ICON_ROW_8_SECTOR:  case ELEM_ICON_ROW_9_SECTOR:
        case ELEM_ICON_ROW_10_SECTOR: case ELEM_ICON_ROW_11_SECTOR:
        case ELEM_ICON_ROW_12_SECTOR: case ELEM_ICON_ROW_13_SECTOR:
        case ELEM_ICON_ROW_14_SECTOR: case ELEM_ICON_ROW_15_SECTOR:
        case ELEM_ICON_ROW_16_SECTOR: case ELEM_ICON_ROW_17_SECTOR:
        case ELEM_ICON_ROW_18_SECTOR: case ELEM_ICON_ROW_19_SECTOR:
        case ELEM_ICON_ROW_20_SECTOR: case ELEM_ICON_ROW_21_SECTOR:
        case ELEM_ICON_ROW_22_SECTOR: case ELEM_ICON_ROW_23_SECTOR:
        case ELEM_ICON_ROW_24_SECTOR: case ELEM_ICON_ROW_25_SECTOR:
        case ELEM_ICON_ROW_26_SECTOR: case ELEM_ICON_ROW_27_SECTOR:
        case ELEM_ICON_ROW_28_SECTOR: case ELEM_ICON_ROW_29_SECTOR:
        case ELEM_ICON_ROW_30_SECTOR: case ELEM_ICON_ROW_31_SECTOR:
        case ELEM_ICON_ROW_32_SECTOR: case ELEM_ICON_ROW_33_SECTOR:
        case ELEM_ICON_ROW_34_SECTOR: case ELEM_ICON_ROW_35_SECTOR:
        case ELEM_ICON_ROW_36_SECTOR: case ELEM_ICON_ROW_37_SECTOR:
        case ELEM_ICON_ROW_38_SECTOR: case ELEM_ICON_ROW_39_SECTOR:
        case ELEM_ICON_ROW_40_SECTOR: case ELEM_ICON_ROW_41_SECTOR:
        case ELEM_ICON_ROW_42_SECTOR: case ELEM_ICON_ROW_43_SECTOR:
        case ELEM_ICON_ROW_44_SECTOR: case ELEM_ICON_ROW_45_SECTOR:
        case ELEM_ICON_ROW_46_SECTOR: case ELEM_ICON_ROW_47_SECTOR:
        case ELEM_ICON_ROW_48_SECTOR: case ELEM_ICON_ROW_49_SECTOR:
        case ELEM_ICON_ROW_50_SECTOR: case ELEM_ICON_ROW_51_SECTOR:
        case ELEM_ICON_ROW_52_SECTOR: case ELEM_ICON_ROW_53_SECTOR:
        case ELEM_ICON_ROW_54_SECTOR: case ELEM_ICON_ROW_55_SECTOR:
        case ELEM_ICON_ROW_56_SECTOR: case ELEM_ICON_ROW_57_SECTOR:
        case ELEM_ICON_ROW_58_SECTOR: case ELEM_ICON_ROW_59_SECTOR:
        case ELEM_ICON_ROW_60_SECTOR: case ELEM_ICON_ROW_61_SECTOR:
        case ELEM_ICON_ROW_62_SECTOR: case ELEM_ICON_ROW_63_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_128);
            break;

        // 4 bytes
        case ELEM_TOTAL_SESSION_TIME_SECTOR:
        case ELEM_CURRENT_COLOR_SECTOR:
            appendBytes(L_RETURN_ELEM_SECTOR_04);
            break;
        case ELEM_M1_FULL: case ELEM_M2_FULL:
        case ELEM_M3_FULL: case ELEM_M4_FULL:
        case ELEM_M5_FULL: case ELEM_M6_FULL:
        case ELEM_M7_FULL: case ELEM_M8_FULL:
        case ELEM_M9_FULL: case ELEM_M10_FULL:
        case ELEM_M11_FULL: case ELEM_M12_FULL:
        case ELEM_M13_FULL: case ELEM_M14_FULL:
        case ELEM_M15_FULL: case ELEM_M0_FULL:
            //appendBytes(L_RETURN_ELEM_SECTOR_218);
            break;
        default:
        #ifdef DEBUG
            Serial.println("Sector no valido");
        #endif
            break;
    }

    // Calcula longitudes (base 18 + dataLen)
    const uint16_t dataLen = static_cast<uint16_t>(frame.data.size());
    const uint16_t frameLength = static_cast<uint16_t>(FRAME_HEADER_BASE_LEN + dataLen);

    frame.frameLengthLsb = frameLength & 0xFF;
    frame.frameLengthMsb = (frameLength >> 8) & 0xFF;
    frame.dataLengthMsb  = (dataLen >> 8) & 0xFF;
    frame.dataLengthLsb  = (dataLen) & 0xFF;

    frame.checksum = checksum_calc(frame);
    frame.end      = NEW_END;
    return frame;
}

FRAME_T frameMaker_SEND_RESPONSE(byte originin, byte targetType, TARGETNS targetNS, byte response){
    FRAME_T f{};
    constexpr uint16_t DL = L_SEND_RESPONSE;          // = 1
    const uint16_t frameLength = FRAME_HEADER_BASE_LEN + DL;

    f.start          = NEW_START;
    f.frameLengthLsb = frameLength & 0xFF;
    f.frameLengthMsb = (frameLength >> 8) & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originin;
    f.originNS       = getLocalNS();
    f.targetType     = targetType;
    f.targetNS       = targetNS;
    f.function       = F_SEND_RESPONSE;
    f.dataLengthMsb  = (DL >> 8) & 0xFF;
    f.dataLengthLsb  = (DL     ) & 0xFF;
    f.data.resize(DL);
    f.data[0]        = response;
    f.checksum       = checksum_calc(f);
    f.end            = NEW_END;
    return f;
}

// Mapeo canónico: botón (1..9)  í­ndice LED fí­sico (0..8)
static constexpr uint8_t kBtnIdToLedIdx[10] = {
  0xFF, // [0] no usado
  8,    // 1  LED8 (AZUL)
  6,    // 2  LED6 (VERDE)
  4,    // 3  LED4 (AMARILLO)
  2,    // 4  LED2 (ROJO)
  0,    // 5  LED0 (RELE)
  7,    // 6  LED7 (VIOLETA)
  5,    // 7  LED5 (NARANJA)
  3,    // 8  LED3 (CELESTE)
  1     // 9  LED1 (BLANCO)
};


// Coloca cada BUTTON en Button_00..08 según el LED fí­sico
static inline void putByLedIndex(COLORPAD_BTNMAP& m, uint8_t ledIndex, const BUTTON& b) {
    switch (ledIndex) {
        case 0: m.Button_00 = b; break;
        case 1: m.Button_01 = b; break;
        case 2: m.Button_02 = b; break;
        case 3: m.Button_03 = b; break;
        case 4: m.Button_04 = b; break;
        case 5: m.Button_05 = b; break;
        case 6: m.Button_06 = b; break;
        case 7: m.Button_07 = b; break;
        case 8: m.Button_08 = b; break;
        default: break;
    }
}

// Construye el COLORPAD_BTNMAP desde 9 botones lógicos (1..9)
// void buildColorpadFromBtnIdsV2(
//     const BUTTON& b1, const BUTTON& b2, const BUTTON& b3,
//     const BUTTON& b4, const BUTTON& b5, const BUTTON& b6,
//     const BUTTON& b7, const BUTTON& b8, const BUTTON& b9,
//     COLORPAD_BTNMAP& outMap
// ){
//     const BUTTON v[9] = { b1,b2,b3,b4,b5,b6,b7,b8,b9 };
//     for (uint8_t id = 1; id <= 9; ++id) {
//         BUTTON bi = v[id-1];
//         const uint8_t ledIndex = kBtnIdToLedIdx[id];
//         if (ledIndex <= 8) putByLedIndex(outMap, ledIndex, bi);
//     }
// }

// === Reemplaza COMPLETO tu frameMaker_SET_BUTTONS_EXTMAP antiguo ===
FRAME_T frameMaker_SET_BUTTONS_EXTMAP(
    uint8_t               originType,   // p.ej. DC (Consola)
    uint8_t               targetType,   // p.ej. DB (Botonera)
    const TARGETNS&       destNS,       // 00:00:00:00:00 = broadcast
    const COLORPAD_BTNMAP &map
){
    FRAME_T f{};

    // ----- DATA: 1 (PADFX) + 9*6 (active,numColor,r,g,b,fx) = 55 (0x37) -----
    const uint16_t dlen = 1u + (9u * 6u); // 0x37
    f.data.clear();
    f.data.resize(dlen);

    // Serialización en orden fijo 0..8
    const BUTTON btns[9] = {
        map.Button_00, map.Button_01, map.Button_02,
        map.Button_03, map.Button_04, map.Button_05,
        map.Button_06, map.Button_07, map.Button_08
    };

    size_t off = 0;
    f.data[off++] = static_cast<uint8_t>(map.PADFX);

    for (int i = 0; i < 9; ++i) {
        const BUTTON &b = btns[i];
        f.data[off++] = (b.active ? 1 : 0);
        f.data[off++] = b.numColor;
        f.data[off++] = b.r;
        f.data[off++] = b.g;
        f.data[off++] = b.b;
        f.data[off++] = static_cast<uint8_t>(b.fx);
    }

    // ----- Cabecera NS (tu formato NS-only) -----
    const uint16_t len = FRAME_HEADER_BASE_LEN + dlen;    // room..checksum+end

    f.start           = NEW_START;
    f.frameLengthMsb  = (len >> 8) & 0xFF;
    f.frameLengthLsb  =  len       & 0xFF;
    f.room           = getLocalRoom();
    f.origin          = originType;
    f.originNS        = getLocalNS();
    f.targetType      = targetType;
    f.targetNS        = destNS;           // 5 bytes NS
    f.function        = F_SET_BUTTONS_EXTMAP;
    f.dataLengthMsb   = (dlen >> 8) & 0xFF;
    f.dataLengthLsb   =  dlen       & 0xFF;

    f.checksum        = checksum_calc(f);
    f.end             = NEW_END;
    return f;
}

//////////////////////////////////////////////////////////////////////////////////////

constexpr uint8_t OLD_NODE     = 0x01;
constexpr uint8_t OLD_FUNC     = 0xCB;
constexpr uint8_t OLD_DL       = 0x02;
constexpr uint8_t OLD_ROOM     = 0x01;

// Ajustes finos del checksum legacy (si tu firmware difiere, toca aquí­)
constexpr bool     OLD_CHK_INCLUDE_START_END = true;   // incluye START y END en la suma
constexpr uint8_t  OLD_CHK_EXTRA_ADDEND      = 0x01;   // +1 final (para que con COLOR=0x04 -> 0x4D)

inline uint8_t old_color_checksum(uint8_t color,
                                  uint8_t node = OLD_NODE,
                                  uint8_t func = OLD_FUNC,
                                  uint8_t dl   = OLD_DL,
                                  uint8_t room = OLD_ROOM)
{
    uint16_t sum = 0;
    if (OLD_CHK_INCLUDE_START_END) sum += OLD_START;
    sum += node + func + dl + room + color;
    if (OLD_CHK_INCLUDE_START_END) sum += OLD_END;
    sum += OLD_CHK_EXTRA_ADDEND;       // para cuadrar 0x4D en tu ejemplo
    return static_cast<uint8_t>(sum & 0xFF);
}

void send_old_color(uint8_t color)
{
    int  i     = 0;
    byte dTime = 5;

    const uint8_t chk = old_color_checksum(color);

    #ifdef DEBUG
    DEBUG__________ln(" #### Trama LEGACY enviada ####");
    #endif

    Serial1.write(OLD_START);            // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] Start(OLD) = " + String(OLD_START, HEX));
    #endif

    Serial1.write(OLD_NODE);             // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] NODE = " + String(OLD_NODE, HEX));
    #endif

    Serial1.write(OLD_FUNC);             // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] FunctionCode = " + String(OLD_FUNC, HEX));
    #endif

    Serial1.write(OLD_DL);               // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] DL = " + String(OLD_DL, HEX));
    #endif

    Serial1.write(OLD_ROOM);             // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] ROOM = " + String(OLD_ROOM, HEX));
    #endif

    Serial1.write(color);                // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] COLOR = " + String(color, HEX));
    #endif

    Serial1.write(chk);                  // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] CHK = " + String(chk, HEX));
    #endif

    Serial1.write(OLD_END);              // delay(dTime);
    #ifdef DEBUG
    DEBUG__________ln("[" + String(++i) + "] END(OLD) = " + String(OLD_END, HEX));
    DEBUG__________ln("======================================");
    #endif
}

#define RF_UART Serial1

void rfDebugPrint(const char* msg)
{
    if (!msg || !*msg) return;
    delay(100);
    //RF_UART.write('\n'); // separador entre logs
}

void rfDebugPrintf(const char* fmt, ...)
{
    static char buf[256]; // tamaño razonable para logs
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    rfDebugPrint(buf);
}
