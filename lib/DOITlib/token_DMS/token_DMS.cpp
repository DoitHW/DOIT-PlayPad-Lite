#include <Arduino.h>
#include <defines_DMS/defines_DMS.h>
#include <token_DMS/token_DMS.h>
#include <play_DMS/play_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <encoder_handler/encoder_handler.h>
#include <Pulsadores_handler/Pulsadores_handler.h>
#include <SPIFFS_handler/SPIFFS_handler.h>

// Definición de pines I2C para ESP32 (ajústalos según tu hardware)
#define SDA_NFC_PIN 40
#define SCL_NFC_PIN 41

// Definición de pines para el PN532 en modo I2C con IRQ
//#define PN532_IRQ   (42)    // Ajusta este valor según tu conexión
#define PN532_RESET (-1)    // Normalmente -1 si no se utiliza reset

// Instancia global del PN532 utilizando la librería Adafruit_PN532
//static Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
TwoWire PN532_Wire = TwoWire(1);
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET, &PN532_Wire);



// Variables para manejo de IRQ (para uso interno de la lectura)
extern int irqCurr;
extern int irqPrev;
extern bool cardIsRead;
extern TOKEN_ token;

TOKEN_::TOKEN_() : genre(0), lang(static_cast<uint8_t>(currentLanguage)) {
    currentUID = "";
    lastProcessedUID = "";
}


void TOKEN_::begin() {
  const int MAX_RETRIES = 5;
  int retryCount = 0;

  PN532_Wire.begin(SDA_NFC_PIN, SCL_NFC_PIN);  // SDA_2 / SCL_2
  PN532_Wire.setClock(100000);
  delay(100);

  while (retryCount < MAX_RETRIES) {
      nfc.begin();
      uint32_t versiondata = nfc.getFirmwareVersion();
      if (versiondata) {
          nfc.SAMConfig();
          startListeningToNFC();
          return;
      }

      DEBUG__________printf("ERROR: No se encontró el módulo PN532 (intento %d de %d). Reintentando...\n", retryCount + 1, MAX_RETRIES);
      retryCount++;
      delay(500);
  }

  DEBUG__________ln("FATAL: No se pudo inicializar el PN532 tras múltiples intentos. Reiniciando dispositivo...");
  delay(2000);
  ESP.restart();
}

bool TOKEN_::isCardPresent() {
  uint8_t _uid[7] = {0};
  uint8_t uidLength = 0;
  
  // Usamos un timeout muy corto para un sondeo rápido
  bool result = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, _uid, &uidLength, 100);
  
  return result;
}

void TOKEN_::resetReader() {
  DEBUG__________ln("⚠️ Forzando reinicio completo del lector NFC...");
  
  // Cancelar cualquier operación pendiente
  nfc.inListPassiveTarget();
  
  // Pausa para estabilizar
  delay(100);
  
  // Reiniciar configuración completa
  nfc.begin();
  delay(100);
  nfc.SAMConfig();
  delay(100);
  
  // Reiniciar indicadores de IRQ
  irqPrev = HIGH;
  irqCurr = HIGH;
  
  // Iniciar detección pasiva
  startListeningToNFC();
}

void TOKEN_::startListeningToNFC() {
  // Reiniciar los indicadores de IRQ
  irqPrev = HIGH;
  irqCurr = HIGH;

  // Cancelar cualquier operación pendiente primero
  nfc.inListPassiveTarget();
  
  // Pequeña pausa para estabilizar
  delay(5);
    
  // Reintentar varias veces si falla
  bool success = false;
 
    success = nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
    if (success) {
      DEBUG__________ln("Detección iniciada correctamente.");
    }
      
  if (!success) {
    DEBUG__________ln("No card found. Waiting...");
  } else {
    DEBUG__________ln("Card already present.");
    handleCardDetected();
  }
}

void TOKEN_::handleCardDetected() {
  uint8_t success = false;
  uint8_t uidBuffer[7] = {0};
  uint8_t uidLength;
  
  // Intentar leer hasta 3 veces si falla
  for (int attempt = 0; attempt < 3; attempt++) {
      success = nfc.readDetectedPassiveTargetID(uidBuffer, &uidLength);
      if (success) break;
      delay(10);
  }
  
  DEBUG__________ln(success ? "Read successful" : "Read failed (not a card?)");
  
  if (success) {
    String newUID = "";
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uidBuffer[i] < 0x10) newUID += "0";
      newUID += String(uidBuffer[i], HEX);
    }
    newUID.toUpperCase();
    currentUID = newUID;
    uidDetected = true;
    mensajeLeido = false;
    uidDetectionTime = millis();

    DEBUG__________("Found an ISO14443A card, UID: ");
    DEBUG__________ln(currentUID);
  } else {
    // Si falló la lectura, reiniciar la detección
    startListeningToNFC();
  }

}

bool TOKEN_::readCard(String &uid) {
  // Se utiliza un timeout corto (por ejemplo, 100 ms) para la lectura
  uint8_t _uid[7] = {0};
  uint8_t uidLength = 0;

  // Intentar leer la tarjeta en modo pasivo usando la función readPassiveTargetID (aprovecha IRQ internamente)
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, _uid, &uidLength, 100);
  if (!success || uidLength == 0) {
      currentUID = "";
      return false;
  }

  // Convertir el UID a cadena hexadecimal
  String newUID = "";
  for (uint8_t i = 0; i < uidLength; i++) {
    if (_uid[i] < 0x10) newUID += "0";
    newUID += String(_uid[i], HEX);
  }
  newUID.toUpperCase();

  uid = newUID;
  currentUID = newUID;

  DEBUG__________("✅ Tarjeta detectada con UID: ");
  DEBUG__________ln(currentUID);
  return true;
}

String TOKEN_::decodeNdefText(const byte* payload, int payloadLength) {
  if (payloadLength < 1) return "[Payload vacío]";
  byte statusByte = payload[0];
  int langCodeLength = statusByte & 0x3F;
  if (1 + langCodeLength >= payloadLength) return "[Payload inválido]";
  int textLength = payloadLength - 1 - langCodeLength;
  String decodedText = "";
  for (int i = 0; i < textLength && i < 255; i++) {
      decodedText += (char)payload[i + 1 + langCodeLength];
  }
  return decodedText;
}

// Función modificada para leer el mensaje NDEF (token) desde las páginas 6 a 18
// bool TOKEN_::leerMensajeNFC(String &mensaje) {
//   DEBUG__________ln("DEBUG: Procesando mensaje NDEF para la tarjeta con UID: " + currentUID);

//   uint8_t rawBuffer[128] = {0};
//   int len = 0;
//   // Leer de la página 6 a la 18 (13 páginas * 4 bytes = 52 bytes)
//   for (uint8_t page = 6; page <= 30; page++) {
//     uint8_t pageBuffer[4] = {0};
//     if (!nfc.ntag2xx_ReadPage(page, pageBuffer)) {
//       DEBUG__________("⚠️ ERROR: Falló la lectura de la página ");
//       DEBUG__________ln(page);
//       mensaje = "";
//       return false;
//     }
//     memcpy(rawBuffer + len, pageBuffer, 4);
//     len += 4;
//   }
  
//   // Convertir el buffer a una cadena completa (cada byte se interpreta como carácter)
//   String fullData = "";
//   for (int i = 0; i < len; i++) {
//     fullData += (char)rawBuffer[i];
//   }
//   DEBUG__________ln("Dump completo (páginas 6 a 18): " + fullData);
  
//   // Buscar los delimitadores '#' en la cadena
//   int firstHash = fullData.indexOf('#');
//   if (firstHash < 0) {
//     DEBUG__________ln("⚠️ ERROR: No se encontró el delimitador de inicio '#'.");
//     mensaje = "";
//     return false;
//   }
//   int secondHash = fullData.indexOf('#', firstHash + 1);
//   if (secondHash < 0) {
//     DEBUG__________ln("⚠️ ERROR: No se encontró el delimitador de fin '#'.");
//     mensaje = "";
//     return false;
//   }
  
//   // Extraer la cadena entre los dos delimitadores
//   String tokenStr = fullData.substring(firstHash + 1, secondHash);
//   DEBUG__________ln("Token string extraído: " + tokenStr);
  
//   // Se espera que el token tenga 46 dígitos hexadecimales (23 bytes)
//   if (tokenStr.length() != 94) {
//     DEBUG__________ln("⚠️ ERROR: Longitud del token inválida. Se esperaba 46 dígitos hexadecimales, se obtuvo: " + String(tokenStr.length()));
//     mensaje = "";
//     return false;
//   }
  
//   // Función lambda para convertir dos caracteres ASCII hex a un byte
//   auto hexToByte = [](const String &str) -> byte {
//     return (byte)strtol(str.c_str(), nullptr, 16);
//   };

//   // Nuevo orden:
//   // Bytes 0-1: CMD
//   // Bytes 2-3: CMD2
//   // Bytes 4-5: Bank
//   // Bytes 6-7: File
//   // Bytes 8-9: Color R
//   // Bytes 10-11: Color G
//   // Bytes 12-13: Color B
//   // Bytes 14-45: 8 parejas para los partners (8×4 dígitos = 32 dígitos)
//   token.currentToken.cmd = hexToByte(tokenStr.substring(0, 2));
//   token.currentToken.cmd2 = hexToByte(tokenStr.substring(2, 4));
//   token.currentToken.addr.bank = hexToByte(tokenStr.substring(4, 6));
//   //updateBankList(token.currentToken.addr.bank);
//   DEBUG__________ln("SPIFFS: Bank actualizado: " + String(token.currentToken.addr.bank, HEX));
//   token.currentToken.addr.file = hexToByte(tokenStr.substring(6, 8));
//   token.currentToken.color.r = hexToByte(tokenStr.substring(8, 10));
//   token.currentToken.color.g = hexToByte(tokenStr.substring(10, 12));
//   token.currentToken.color.b = hexToByte(tokenStr.substring(12, 14));
  
//   for (int i = 0; i < 8; i++) {
//     int startIdx = 14 + i * 4;
//     token.currentToken.partner[i].bank = hexToByte(tokenStr.substring(startIdx, startIdx + 2));
//     token.currentToken.partner[i].file = hexToByte(tokenStr.substring(startIdx + 2, startIdx + 4));
//   }

//   // Leer 24 bytes para el nombre de la familia (posiciones 46 a 93)
// for (int i = 0; i < 24; i++) {
//   token.currentToken.familyName[i] = hexToByte(tokenStr.substring(46 + i * 2, 48 + i * 2));
// }
// token.currentToken.familyName[24] = '\0'; // Asegurar string nulo-terminado

//   updateBankAndFamilyList(token.currentToken.addr.bank, (char*)token.currentToken.familyName);
//   bankList = readBankList();
//   selectedBanks.resize(bankList.size(), false);
//   // Imprimir valores para depuración
//   DEBUG__________ln("Token decodificado:");
//   DEBUG__________("CMD: 0x"); DEBUG__________ln(token.currentToken.cmd, HEX);
//   DEBUG__________("CMD2: 0x"); DEBUG__________ln(token.currentToken.cmd2, HEX);
//   DEBUG__________("Bank: 0x"); DEBUG__________ln(token.currentToken.addr.bank, HEX);
//   DEBUG__________("File: 0x"); DEBUG__________ln(token.currentToken.addr.file, HEX);
//   DEBUG__________("Color R: 0x"); DEBUG__________ln(token.currentToken.color.r, HEX);
//   DEBUG__________("Color G: 0x"); DEBUG__________ln(token.currentToken.color.g, HEX);
//   DEBUG__________("Color B: 0x"); DEBUG__________ln(token.currentToken.color.b, HEX);
//   for (int i = 0; i < 8; i++) {
//     DEBUG__________("Partner "); DEBUG__________(i); DEBUG__________(": Bank=0x");
//     DEBUG__________(token.currentToken.partner[i].bank, HEX);
//     DEBUG__________(", File=0x");
//     DEBUG__________ln(token.currentToken.partner[i].file, HEX);
//   }
//   DEBUG__________("🏷️  Familia: ");
//   DEBUG__________ln(token.currentToken.familyName);

  
//   mensaje = tokenStr;
//   // Actualizar el UID procesado para evitar relecturas mientras la misma tarjeta esté presente
//   lastProcessedUID = currentUID;
//   mensajeLeido = true;

//   return true;
// }

bool TOKEN_::leerMensajeNFC(String &mensaje) {
  DEBUG__________ln("[NFC] leerMensajeNFC() ENTER");
  DEBUG__________ln("DEBUG: Procesando mensaje NDEF para UID: " + currentUID);

    uint8_t rawBuffer[128] = {0};
    int len = 0;

    // Rango real que estás usando (6..30). Ajusta si de verdad solo necesitas 6..18.
    const uint8_t kFirstPage = 6;
    const uint8_t kLastPage  = 30;

    for (uint8_t page = kFirstPage; page <= kLastPage; page++) {
        uint8_t pageBuffer[4] = {0};

        bool ok = false;
        for (int attempt = 0; attempt < 4; ++attempt) {
            if (nfc.ntag2xx_ReadPage(page, pageBuffer)) {
                ok = true;
                break;
            }
            delay(6);
        }

        if (!ok) {
            DEBUG__________("⚠️ ERROR: Falló lectura página ");
            DEBUG__________ln(page);
            mensaje = "";
            return false;
        }

        if (len + 4 <= (int)sizeof(rawBuffer)) {
            memcpy(rawBuffer + len, pageBuffer, 4);
            len += 4;
        } else {
            DEBUG__________ln("⚠️ ERROR: rawBuffer overflow");
            mensaje = "";
            return false;
        }

        // Pequeña pausa para estabilidad I2C/PN532
        delay(2);
    }

    // Buscar delimitadores '#' (0x23) directamente en bytes
    int firstHash = -1, secondHash = -1;
    for (int i = 0; i < len; ++i) {
        if (rawBuffer[i] == '#') {
            if (firstHash < 0) firstHash = i;
            else { secondHash = i; break; }
        }
    }

    if (firstHash < 0 || secondHash < 0 || secondHash <= firstHash + 1) {
        DEBUG__________ln("⚠️ ERROR: delimitadores '#' no encontrados o inválidos");
        mensaje = "";
        return false;
    }

    // Extraer token ASCII entre hashes (ignorando 0x00 no debería estar ahí, pero lo toleramos)
    String tokenStr;
    tokenStr.reserve(120);

    for (int i = firstHash + 1; i < secondHash; ++i) {
        const uint8_t b = rawBuffer[i];
        if (b == 0x00) continue;
        tokenStr += (char)b;
    }

    DEBUG__________ln("Token string extraído: " + tokenStr);

    // Tu formato actual: 94 hex chars = 47 bytes
    if (tokenStr.length() != 94) {
        DEBUG__________ln("⚠️ ERROR: Longitud token inválida, esperada 94, obtenida: " + String(tokenStr.length()));
        mensaje = "";
        return false;
    }

    auto hexToByte = [](const String &str) -> byte {
        return (byte)strtol(str.c_str(), nullptr, 16);
    };

    token.currentToken.cmd       = hexToByte(tokenStr.substring(0, 2));
    token.currentToken.cmd2      = hexToByte(tokenStr.substring(2, 4));
    token.currentToken.addr.bank = hexToByte(tokenStr.substring(4, 6));
    DEBUG__________ln("SPIFFS: Bank actualizado: " + String(token.currentToken.addr.bank, HEX));
    token.currentToken.addr.file = hexToByte(tokenStr.substring(6, 8));
    token.currentToken.color.r   = hexToByte(tokenStr.substring(8, 10));
    token.currentToken.color.g   = hexToByte(tokenStr.substring(10, 12));
    token.currentToken.color.b   = hexToByte(tokenStr.substring(12, 14));

    for (int i = 0; i < 8; i++) {
        int startIdx = 14 + i * 4;
        token.currentToken.partner[i].bank = hexToByte(tokenStr.substring(startIdx, startIdx + 2));
        token.currentToken.partner[i].file = hexToByte(tokenStr.substring(startIdx + 2, startIdx + 4));
    }

    // familyName: 24 bytes = 48 hex chars, empieza en offset 46 (según tu implementación)
    for (int i = 0; i < 24; i++) {
        token.currentToken.familyName[i] = hexToByte(tokenStr.substring(46 + i * 2, 48 + i * 2));
    }
    token.currentToken.familyName[24] = '\0';

    updateBankAndFamilyList(token.currentToken.addr.bank, (char*)token.currentToken.familyName);
    bankList = readBankList();
    selectedBanks.resize(bankList.size(), false);

    DEBUG__________ln("Token decodificado:");
    DEBUG__________("CMD: 0x");  DEBUG__________ln(token.currentToken.cmd, HEX);
    DEBUG__________("CMD2: 0x"); DEBUG__________ln(token.currentToken.cmd2, HEX);
    DEBUG__________("Bank: 0x"); DEBUG__________ln(token.currentToken.addr.bank, HEX);
    DEBUG__________("File: 0x"); DEBUG__________ln(token.currentToken.addr.file, HEX);

    DEBUG__________("🏷️  Familia: ");
    DEBUG__________ln(token.currentToken.familyName);

    mensaje = tokenStr;
    lastProcessedUID = currentUID;
    mensajeLeido = true;
    return true;
}


void TOKEN_::proponer_token(byte guessbank) {
    lang = 0;
    int fileRand = random(1, 8);  // Genera un número aleatorio entre 1 y 7
    if (guessbank > 0x09 && guessbank < 0x63) lang = static_cast<uint8_t>(currentLanguage) * 10;
    doitPlayer.play_file(guessbank + genre, fileRand + lang);
    propossedToken.addr.bank = guessbank;
    propossedToken.addr.file = fileRand;
}

// Nueva firma: pasa la lista de NS de los destinos (dispositivos 0xDD)
// origin debería ser DEFAULT_BOTONERA en la botonera
void TOKEN_::token_handler(const TOKEN_DATA& token,
                           uint8_t          lang_in,
                           bool             genre_in,
                           uint8_t          origin,
                           const std::vector<TARGETNS>& targetsNS)
{
    // Si el banco es de “voz/idioma”, aplicar stride 10 por idioma
    byte lang = 0;
    if (token.addr.bank > 0x09 && token.addr.bank < 0x63) {
        lang = static_cast<byte>(lang_in * 10);
    }

    // Sólo procesamos FX o NOFX
    if (token.cmd == TOKEN_FX || token.cmd == TOKEN_NOFX) {

        // 1) Color de la ficha (RGB directo)
        COLOR_T colorout;
        colorout.red   = token.color.r;
        colorout.green = token.color.g;
        colorout.blue  = token.color.b;

        // Si hay configuración de color (temporal o permanente), “pinta” todo (broadcast)
        if (token.cmd2 != NO_COLOR_CONF) {
            // broadcast → targetType = 0xFF, targetNS = NS_ZERO
            send_frame(frameMaker_SEND_RGB(origin, BROADCAST, NS_ZERO, colorout));
            delay(200);
        }

        delay(200); // estabilidad visual

        // 2) Audio asociado a la ficha
        DEBUG__________ln("Bank: " + String(token.addr.bank + genre_in, HEX) +
                          ", File: " + String(token.addr.file + lang,  HEX));
        doitPlayer.play_file(token.addr.bank + genre_in, token.addr.file + lang);
        delay(50);
        // while (doitPlayer.is_playing()) { delay(10); }
        // delay(500);

        doitPlayer.wait_end(300, 20000, 20);
        delay(200); // separación entre audios

        // 3) Comportamiento según modo del Token
        if (tokenCurrentMode == TOKEN_BASIC_MODE) {

            if (token.cmd2 == TEMP_COLOR_CONF) {
                // Apagar (negro) tras el audio en TODOS los destinos concretos
                for (const TARGETNS& ns : targetsNS) {
                    send_frame(frameMaker_SEND_COLOR(origin, DEFAULT_DEVICE, ns, /*NEGRO*/ 8));
                    delay(10);
                }
            }
        }
        else if (tokenCurrentMode == TOKEN_PARTNER_MODE) {
           DEBUG__________printf("[TOKEN][PARTNER] branch  waitingForPartner=%d\n", waitingForPartner ? 1 : 0);

            static bool firstTokenStored = false;
            static byte firstTokenBank = 0;
            static byte firstTokenFile = 0;

            if (!waitingForPartner) {
        DEBUG__________printf("[TOKEN][PARTNER] -> store first  bank=0x%02X file=0x%02X\n",
                              token.addr.bank, token.addr.file);
                // Guardar la 1ª ficha y esperar la pareja
                firstTokenBank   = token.addr.bank;
                firstTokenFile   = token.addr.file;
                firstTokenStored = true;
                waitingForPartner = true;

                DEBUG__________printf("Primer token registrado: Bank = 0x%02X, File = 0x%02X\n",
                                      firstTokenBank, firstTokenFile);

                if (token.cmd2 == TEMP_COLOR_CONF) {
                    delay(50);
                    for (const TARGETNS& ns : targetsNS) {
                        send_frame(frameMaker_SEND_COLOR(origin, DEFAULT_DEVICE, ns, /*NEGRO*/ 8));
                        delay(10);
                    }
                    DEBUG__________ln("Primer token: Color temporal, apagando después del audio.");
                }
            } else {
                      DEBUG__________printf("[TOKEN][PARTNER] -> compare  second bank=0x%02X file=0x%02X  vs first bank=0x%02X file=0x%02X\n",
                              token.addr.bank, token.addr.file, firstTokenBank, firstTokenFile);
                // Ya había 1ª ficha almacenada: evaluar pareja
                if (!firstTokenStored) {
                    DEBUG__________ln("Error: No se encontró el primer token almacenado.");
                    return;
                }

                bool match = false;
                DEBUG__________printf("Comparando primer token (Bank = 0x%02X, File = 0x%02X) con partners:\n",
                                      firstTokenBank, firstTokenFile);

                for (int i = 0; i < 8; i++) {
                    DEBUG__________printf("Partner %d: Bank = 0x%02X, File = 0x%02X\n",
                                          i, token.partner[i].bank, token.partner[i].file);

                    // Caso 1: match exacto
                    if (token.partner[i].bank == firstTokenBank &&
                        token.partner[i].file == firstTokenFile) {
                        match = true;
                        DEBUG__________printf("✔️ Match exacto en partner %d\n", i);
                        break;
                    }
                    // Caso 2: mismo banco (file comodín 0xFF)
                    if (token.partner[i].file == 0xFF &&
                        token.partner[i].bank == firstTokenBank) {
                        match = true;
                        DEBUG__________printf("🏷️ Match de familia en partner %d (bank=0x%02X)\n",
                                              i, token.partner[i].bank);
                        break;
                    }
                }

                delay(50);
                doitPlayer.wait_end(300, 20000, 20);
delay(200);
                DEBUG__________printf("[TOKEN][PARTNER] RESP=%s\n", match ? "WIN" : "FAIL");
                // Respuesta WIN/FAIL + audio correspondiente
                int fileNum = match ? random(1, 4) : random(1, 3);
                for (const TARGETNS& ns : targetsNS) {
                    send_frame(frameMaker_SEND_RESPONSE(origin, DEFAULT_DEVICE, ns, match ? WIN : FAIL));
                    delay(10);
                }
                doitPlayer.play_file((match ? WIN_RESP_BANK : FAIL_RESP_BANK) + genre_in,
                                     fileNum + lang);

                delay(50);
                doitPlayer.wait_end(300, 20000, 20);
delay(200);

                // Color post-respuesta según configuración
                if (token.cmd2 == TEMP_COLOR_CONF) {
                    for (const TARGETNS& ns : targetsNS) {
                        send_frame(frameMaker_SEND_COLOR(origin, DEFAULT_DEVICE, ns, /*NEGRO*/ 8));
                        delay(10);
                    }
                    DEBUG__________ln("Segundo token: TEMP_COLOR_CONF, apagando color.");
                } else if (token.cmd2 == PERM_COLOR_CONF) {
                    for (const TARGETNS& ns : targetsNS) {
                        send_frame(frameMaker_SEND_RGB(origin, DEFAULT_DEVICE, ns, colorout));
                        delay(10);
                    }
                    DEBUG__________ln("Segundo token: PERM_COLOR_CONF, manteniendo color.");
                }

                waitingForPartner  = false;
                firstTokenStored   = false;
            }
        }
        else if (tokenCurrentMode == TOKEN_GUESS_MODE) {
            // ¿Acierto o fallo?
            bool isCorrect = (token.addr.bank == propossedToken.addr.bank &&
                              token.addr.file == propossedToken.addr.file);
DEBUG__________printf("[TOKEN][GUESS] branch  proposed B=0x%02X F=0x%02X  scanned B=0x%02X F=0x%02X  RESP=%s\n",
                          propossedToken.addr.bank, propossedToken.addr.file,
                          token.addr.bank, token.addr.file,
                          isCorrect ? "WIN" : "FAIL");
            doitPlayer.wait_end(300, 20000, 20);
            delay(200);

            // Respuesta WIN/FAIL y audio
            for (const TARGETNS& ns : targetsNS) {
                send_frame(frameMaker_SEND_RESPONSE(origin, DEFAULT_DEVICE, ns, isCorrect ? WIN : FAIL));
                delay(10);
            }

            doitPlayer.play_file((isCorrect ? WIN_RESP_BANK : FAIL_RESP_BANK) + genre_in,
                                 random(1, isCorrect ? 4 : 3) + lang);

            doitPlayer.wait_end(300, 20000, 20);
            delay(200);

            // Color post-respuesta
            if (token.cmd2 == TEMP_COLOR_CONF) {
                for (const TARGETNS& ns : targetsNS) {
                    send_frame(frameMaker_SEND_COLOR(origin, DEFAULT_DEVICE, ns, /*NEGRO*/ 8));
                    delay(10);
                }
            } else if (token.cmd2 == PERM_COLOR_CONF) {
                for (const TARGETNS& ns : targetsNS) {
                    send_frame(frameMaker_SEND_RGB(origin, DEFAULT_DEVICE, ns, colorout));
                    delay(10);
                }
            }
        }
    }
}

void TOKEN_::printFicha(const TOKEN_DATA &f) {
  DEBUG__________("CMD: 0x"); DEBUG__________ln(f.cmd, HEX);
  DEBUG__________("CMD2: 0x"); DEBUG__________ln(f.cmd2, HEX);
  DEBUG__________("Bank: 0x"); DEBUG__________ln(f.addr.bank, HEX);
  DEBUG__________("File: 0x"); DEBUG__________ln(f.addr.file, HEX);
  DEBUG__________("R: 0x"); DEBUG__________ln(f.color.r, HEX);
  DEBUG__________("G: 0x"); DEBUG__________ln(f.color.g, HEX);
  DEBUG__________("B: 0x"); DEBUG__________ln(f.color.b, HEX);
  for (int i = 0; i < 8; i++) {
    DEBUG__________("P"); DEBUG__________(i);
    DEBUG__________(".Bank: 0x"); DEBUG__________(f.partner[i].bank, HEX);
    DEBUG__________("  P"); DEBUG__________(i);
    DEBUG__________(".File: 0x"); DEBUG__________ln(f.partner[i].file, HEX);
  }
}

byte TOKEN_::asciiHexToByte(char high, char low) {
  byte value = 0;
  if(high >= '0' && high <= '9')
    value = (high - '0') << 4;
  else if(high >= 'A' && high <= 'F')
    value = (high - 'A' + 10) << 4;
  else if(high >= 'a' && high <= 'f')
    value = (high - 'a' + 10) << 4;
  
  if(low >= '0' && low <= '9')
    value |= (low - '0');
  else if(low >= 'A' && low <= 'F')
    value |= (low - 'A' + 10);
  else if(low >= 'a' && low <= 'f')
    value |= (low - 'a' + 10);
  
  return value;
}

