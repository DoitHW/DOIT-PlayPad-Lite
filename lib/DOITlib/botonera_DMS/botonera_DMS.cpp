#include <botonera_DMS/botonera_DMS.h>
#include <Colors_DMS/Color_DMS.h>
#include <defines_DMS/defines_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>
#include <EEPROM.h>
#include <encoder_handler/encoder_handler.h>
#include <display_handler/display_handler.h>
#include <RelayManager_DMS/RelayStateManager.h>
#include <Translations_handler/translations.h>
#include <display_handler/display_handler.h>
#include <Pulsadores_handler/Pulsadores_handler.h>
#include <deque>

#define MIN_VALID_ELEMENT_SIZE (OFFSET_ID + 1) // Se espera que el archivo tenga al menos OFFSET_ID+1 bytes

static std::deque<SectorMsg> rxSectorInbox;
static volatile bool scanInProgress = false;

static inline bool nsEquals5(const uint8_t* a, const uint8_t* b) {
    for (int i = 0; i < 5; ++i) if (a[i] != b[i]) return false;
    return true;
}

// static ELEM_UNIT_CONF deduceConfFromFile(fs::File &f) noexcept {
//     constexpr uint8_t kBasicModeIndex = 1; // Modo básico 1 (0-based)
 
//     // Comprobación rápida de tamaño mínimo para alcanzar el bloque de flags
//     const size_t cfgOff = OFFSET_MODES + (SIZE_MODE * kBasicModeIndex) + (24 + 192); // al final de cada modo
//     if (f.size() < (cfgOff + 2)) return NO_ELEM;

//     uint8_t modeCfg[2] = {0, 0};
//     f.seek(cfgOff, SeekSet);
//     if (f.read(modeCfg, 2) != 2) return NO_ELEM;

//     // Solo interesa COLOR básico y RELÉ según tu petición
//     const bool hasColor  = getModeFlag(modeCfg, HAS_BASIC_COLOR);
//     const bool hasAction = getModeFlag(modeCfg, HAS_RELAY);  // ajusta si usas HAS_RELAY_1/_2

//     if (hasColor && hasAction) return COLOR_ACTION;
//     if (hasColor)               return COLOR;
//     if (hasAction)              return ACTION;
//     return NO_ELEM;
// }

// Devuelve la lista ordenada y filtrada de ficheros de elementos válidos en SPIFFS
// static std::vector<String> listElementFiles() {
//     std::vector<String> files;
//     File root = SPIFFS.open("/");
//     for (File f = root.openNextFile(); f; f = root.openNextFile()) {
//         const String name = f.name();
//         if (name.startsWith("/element_") && name.endsWith(".bin") && name.indexOf("_icon") < 0) {
//             files.push_back(name);
//         }
//     }
//     std::sort(files.begin(), files.end());
//     return files;
// }

static inline uint8_t confFromModeCfg(const uint8_t modeCfg[2]) noexcept {
    const bool hasColor  = getModeFlag(const_cast<uint8_t*>(modeCfg), HAS_BASIC_COLOR);
    const bool hasAction = getModeFlag(const_cast<uint8_t*>(modeCfg), HAS_RELAY);

    if (hasColor && hasAction) return COLOR_ACTION;
    if (hasColor)              return COLOR;
    if (hasAction)             return ACTION;
    return NO_ELEM;
}

// 32 entradas: NS[5] + conf[1] = 192 bytes
static void buildRoomNsPack32(std::array<uint8_t, 32 * 6> &out) {
    out.fill(0x00);                  // padding con NO_ELEM por defecto
    const uint8_t kBasicModeIndex = DEFAULT_BASIC_MODE;  // o 1 si ese es tu básico

    size_t idx = 0;
    for (const String& file : elementFiles) {           // ya está poblado y filtrado al cargar SPIFFS
        if (idx >= 32) break;

        // Asegura que solo procesas ficheros de elementos reales
        if (!file.startsWith("/element_") || !file.endsWith(".bin") || file.indexOf("_icon") >= 0) {
            continue;
        }

        // 1) NS desde el fichero (robusto)
        TARGETNS ns = NS_ZERO;
        if (!tryGetNSFromFile(file, ns)) {
            // si falla, NO incrementamos idx no creamos huecos
            continue;
        }

        // 2) Flags del modo básico
        uint8_t modeCfg[2] = {0,0};
        if (!getModeConfig(file, kBasicModeIndex, modeCfg)) {
            // si falla la config, dejamos conf=NO_ELEM pero Sí enviamos el NS para depurar
            // (opcional) continue; // si prefieres no enviar tampoco el NS en este caso
        }

        const uint8_t conf = confFromModeCfg(modeCfg);

        // 3) Escribir entrada NS[5] + conf[1]
        const size_t base = idx * 6;
        out[base + 0] = ns.mac01;
        out[base + 1] = ns.mac02;
        out[base + 2] = ns.mac03;
        out[base + 3] = ns.mac04;
        out[base + 4] = ns.mac05;
        out[base + 5] = conf;
        ++idx;
    }

    // Resto queda en 0 NS=00..00 y conf=NO_ELEM
}

extern std::vector<uint8_t> printTargetID;
byte currentCognitiveCommand = COG_ACT_OFF;

BOTONERA_::BOTONERA_() : ELEMENT_() {
            set_type(TYPE_BOTONERA);
        }

// void BOTONERA_::botonera_begin(){}

static inline bool inRange(uint8_t x, uint8_t a, uint8_t b) { return x >= a && x <= b; }

static String bin16(uint16_t v) {
    char buf[17];
    for (int i = 15; i >= 0; --i) buf[15 - i] = (v & (1 << i)) ? '1' : '0';
    buf[16] = 0;
    return String(buf);
}

static String asciiFrom(const std::vector<uint8_t>& v, size_t off, size_t len) {
    String s; s.reserve(len);
    for (size_t i = 0; i < len; ++i) {
        uint8_t c = v[off + i];
        if (c == 0) break;                 // fin de cadena C
        if (c >= 32 && c <= 126) s += char(c);  // imprimible
        else if (c == 10 || c == 13) s += '\n';
        else s += '.';                     // marca no imprimible
    }
    return s;
}
static bool isModeNameDesc(uint8_t sector) {
    return inRange(sector, ELEM_MODE_0_NAME_SECTOR, ELEM_MODE_15_FLAG_SECTOR) &&
           ((sector - ELEM_MODE_0_NAME_SECTOR) % 3 != 2); // NAME/DESC (no FLAG)
}
static bool isModeName(uint8_t sector) {
    return inRange(sector, ELEM_MODE_0_NAME_SECTOR, ELEM_MODE_15_FLAG_SECTOR) &&
           ((sector - ELEM_MODE_0_NAME_SECTOR) % 3 == 0);
}
static bool isModeDesc(uint8_t sector) {
    return inRange(sector, ELEM_MODE_0_NAME_SECTOR, ELEM_MODE_15_FLAG_SECTOR) &&
           ((sector - ELEM_MODE_0_NAME_SECTOR) % 3 == 1);
}
static bool isModeFlag(uint8_t sector) {
    return inRange(sector, ELEM_MODE_0_NAME_SECTOR, ELEM_MODE_15_FLAG_SECTOR) &&
           ((sector - ELEM_MODE_0_NAME_SECTOR) % 3 == 2);
}
static uint8_t modeIndexFromSector(uint8_t sector) {
    return (sector - ELEM_MODE_0_NAME_SECTOR) / 3;
}
static bool isIconRow(uint8_t sector) {
    return inRange(sector, ELEM_ICON_ROW_0_SECTOR, ELEM_ICON_ROW_63_SECTOR);
}
static uint8_t iconRowIndex(uint8_t sector) {
    return sector - ELEM_ICON_ROW_0_SECTOR;
}

static String sectorName(uint8_t s) {
    // Sectores "simples" con nombre fijo
    switch (s) {
        case ELEM_NAME_SECTOR:            return "ELEM_NAME_SECTOR";
        case ELEM_DESC_SECTOR:            return "ELEM_DESC_SECTOR";
        case ELEM_LOCATION_SECTOR:        return "ELEM_LOCATION_SECTOR";
        case ELEM_SERIAL_SECTOR:          return "ELEM_SERIAL_SECTOR";
        case ELEM_UNUSED_SECTOR:          return "ELEM_UNUSED_SECTOR";
        case ELEM_CMODE_SECTOR:           return "ELEM_CMODE_SECTOR";
        case ELEM_MOST_USED_MODE_SECTOR:    return "ELEM_MOST_USED_MODE_SECTOR";
        case ELEM_MOST_USED_COLOR_SECTOR:   return "ELEM_MOST_USED_COLOR_SECTOR";
        case ELEM_MOST_USED_PATTERN_SECTOR: return "ELEM_MOST_USED_PATTERN_SECTOR";
        case ELEM_TOTAL_SESSION_TIME_SECTOR: return "ELEM_TOTAL_SESSION_TIME_SECTOR";
        case ELEM_FLAG_STATE_SECTOR:        return "ELEM_FLAG_STATE_SECTOR";
        case ELEM_CURRENT_COLOR_SECTOR:     return "ELEM_CURRENT_COLOR_SECTOR";
        case ELEM_CURRENT_RED_SECTOR:       return "ELEM_CURRENT_RED_SECTOR";
        case ELEM_CURRENT_GREEN_SECTOR:     return "ELEM_CURRENT_GREEN_SECTOR";
        case ELEM_CURRENT_BLUE_SECTOR:      return "ELEM_CURRENT_BLUE_SECTOR";
        case ELEM_CURRENT_BRIGHTNESS_SECTOR: return "ELEM_CURRENT_BRIGHTNESS_SECTOR";
        case ELEM_CURRENT_FLAGS_SECTOR:     return "ELEM_CURRENT_FLAGS_SECTOR";
        case ELEM_CURRENT_PATTERN_SECTOR:   return "ELEM_CURRENT_PATTERN_SECTOR";
        case ELEM_CURRENT_FILE_SECTOR:      return "ELEM_CURRENT_FILE_SECTOR";
        case ELEM_CURRENT_XMANAGER_SECTOR:  return "ELEM_CURRENT_XMANAGER_SECTOR";
        case ACTIVE_ELEM_LIST:              return "ACTIVE_ELEM_LIST";
    }
    // Series de modos (NAME/DESC/FLAG)
    if (inRange(s, ELEM_MODE_0_NAME_SECTOR, ELEM_MODE_15_FLAG_SECTOR)) {
        uint8_t m = modeIndexFromSector(s);
        uint8_t off = (s - ELEM_MODE_0_NAME_SECTOR) % 3;
        if (off == 0) return "ELEM_MODE_" + String(m) + "_NAME_SECTOR";
        if (off == 1) return "ELEM_MODE_" + String(m) + "_DESC_SECTOR";
        return            "ELEM_MODE_" + String(m) + "_FLAG_SECTOR";
    }
    // Series de icono (filas 0..63)
    if (isIconRow(s)) {
        return "ELEM_ICON_ROW_" + String(iconRowIndex(s)) + "_SECTOR";
    }
    return "SECTOR_DESCONOCIDO";
}

// ===== Estado global de habilitación por LED (0..8) =====
// ===== Utilidades de color =====
static inline const char* fnColor(uint8_t fn) {
    if (fn >= 0xA0 && fn <= 0xAF) return COLOR_BRIGHT_YELLOW;   // REQ
    if (fn >= 0xB0 && fn <= 0xBF) return COLOR_BRIGHT_CYAN;     // SET
    if (fn >= 0xC0 && fn <= 0xCF) return COLOR_BRIGHT_GREEN;    // SEND
    if (fn == 0xDE)                 return COLOR_BRIGHT_RED;    // RETURN ERROR
    if (fn >= 0xD0 && fn <= 0xDF) return COLOR_BRIGHT_MAGENTA;  // RETURN
    return COLOR_BRIGHT_RED;                                     // Desconocida
}

static inline const char* sectorColor(uint8_t sector) {
    // Ajusta a tus enums reales
    if (sector == ELEM_NAME_SECTOR || sector == ELEM_DESC_SECTOR || sector == ELEM_LOCATION_SECTOR) return COLOR_BRIGHT_WHITE;
    if (sector == ELEM_SERIAL_SECTOR)        return COLOR_BRIGHT_CYAN;
    if (sector == ELEM_UNUSED_SECTOR)        return COLOR_BRIGHT_YELLOW;
    if (sector == ELEM_CMODE_SECTOR)         return COLOR_BRIGHT_YELLOW;
    if (sector == ELEM_FLAG_STATE_SECTOR || sector == ELEM_CURRENT_FLAGS_SECTOR || isModeFlag(sector)) return COLOR_BRIGHT_MAGENTA;
    if (isIconRow(sector))                   return COLOR_BRIGHT_BLUE;
    if (sector == ELEM_CURRENT_BRIGHTNESS_SECTOR) return COLOR_BRIGHT_GREEN;
    if (sector == ACTIVE_ELEM_LIST)          return COLOR_BRIGHT_WHITE;
    return COLOR_WHITE;
}

static inline const char* originColor(uint8_t o) {
    switch (o) {
        case DEFAULT_BOTONERA:     return COLOR_BRIGHT_YELLOW;
        case DEFAULT_CONSOLE:      return COLOR_BRIGHT_YELLOW;
        case DEFAULT_DEVICE:       return COLOR_BRIGHT_YELLOW;
        case DEFAULT_TECH_TOOL_ID: return COLOR_BRIGHT_YELLOW;
        case BROADCAST:            return COLOR_BRIGHT_YELLOW;
        default:                   return COLOR_BRIGHT_YELLOW;
    }
}

static inline const char* targetTypeColor(uint8_t t) {
    return COLOR_BRIGHT_CYAN;
}

// ===== Helper para printf con color =====
#define DPRINTF_COLOR(_color, _fmt, ...) \
    do { DEBUG__________printf("%s" _fmt "%s", _color, ##__VA_ARGS__, COLOR_RESET); } while (0)

#define DPRINTLN_COLOR(_color, _s) \
    do { DEBUG__________ln(String(_color) + (String)(_s) + COLOR_RESET); } while (0)

void BOTONERA_::printFrameInfo(LAST_ENTRY_FRAME_T LEF) {
    // Cabecera
    DPRINTLN_COLOR(COLOR_BRIGHT_WHITE, "\n==== Trama Recibida ====");

    // RAW len
    DPRINTF_COLOR(COLOR_DIM, "FrameLen: %u bytes\n", (unsigned)uartBuffer.size());

    // Room
    DPRINTF_COLOR(COLOR_BRIGHT_RED, "Room: 0x%02X\n", LEF.room);

    // Origen (tipo de emisor)
    {
        String origenStr;
        switch (LEF.origin) {
            case     DEFAULT_BOTONERA: origenStr = "BOTONERA";    break;
            case      DEFAULT_CONSOLE: origenStr = "CONSOLA";     break;
            case       DEFAULT_DEVICE: origenStr = "DISPOSITIVO"; break;
            case DEFAULT_TECH_TOOL_ID: origenStr = "HACKING BOX"; break;
            case            BROADCAST: origenStr = "BROADCAST";   break;
            default:                   origenStr = String(LEF.origin);
        }
        DPRINTF_COLOR(originColor(LEF.origin), "Origen:  %s (0x%02X)\n", origenStr.c_str(), LEF.origin);
    }

    // Origin NS
    {
        char nsStr[16];
        snprintf(nsStr, sizeof(nsStr), "%02X:%02X:%02X:%02X:%02X",
                 LEF.originNS.mac01, LEF.originNS.mac02, LEF.originNS.mac03,
                 LEF.originNS.mac04, LEF.originNS.mac05);
        DPRINTF_COLOR(COLOR_BRIGHT_MAGENTA, "OriginNS: %s\n", nsStr);
    }

    // TargetType (usar LEF.targetType, no uartBuffer)
    uint8_t tgtType = LEF.targetType;
    const char* tgtStr;
    switch (tgtType) {
        case DEFAULT_BOTONERA:     tgtStr = "BOTONERA";    break;
        case DEFAULT_DEVICE:       tgtStr = "DISPOSITIVO"; break;
        case DEFAULT_CONSOLE:      tgtStr = "CONSOLA";     break;
        case BROADCAST:            tgtStr = "BROADCAST";   break;
        case DEFAULT_TECH_TOOL_ID: tgtStr = "HACKING BOX"; break;
        default:                   tgtStr = "DESCONOCIDO"; break;
    }
    DPRINTF_COLOR(targetTypeColor(tgtType), "TargetType: %s (0x%02X)\n", tgtStr, tgtType);

    // Destino / Targets (usar LEF.targetNS)
    {
        auto nsIsZero = [](const TARGETNS& ns) {
            return ns.mac01 == 0 &&
                ns.mac02 == 0 &&
                ns.mac03 == 0 &&
                ns.mac04 == 0 &&
                ns.mac05 == 0;
        };

        if (LEF.targetType == BROADCAST || nsIsZero(LEF.targetNS)) {
            DPRINTLN_COLOR(COLOR_DIM, "Destino/Targets: Ninguno (posible broadcast)");
        } else {
            char nsStr[16];
            snprintf(nsStr, sizeof(nsStr), "%02X:%02X:%02X:%02X:%02X",
                    LEF.targetNS.mac01, LEF.targetNS.mac02, LEF.targetNS.mac03,
                    LEF.targetNS.mac04, LEF.targetNS.mac05);
            DPRINTF_COLOR(COLOR_BRIGHT_BLUE, "Destino (NS): %s\n", nsStr);
        }
    }

    // Función
    String functionStr;
    switch (LEF.function) {
        case 0xA0: functionStr = "F_REQ_ELEM_SECTOR"; break;
        case 0xA1: functionStr = "F_REQ_ELEM_INFO"; break;
        case 0xA2: functionStr = "F_REQ_ELEM_STATE"; break;
        case 0xA3: functionStr = "F_REQ_ELEM_ICON"; break;
        case 0xB1: functionStr = "F_SET_ELEM_ID"; break;
        case 0xB2: functionStr = "F_SET_ELEM_MODE"; break;
        case 0xB3: functionStr = "F_SET_ELEM_DEAF"; break;
        case 0xC1: functionStr = "F_SEND_COLOR"; break;
        case 0xC2: functionStr = "F_SEND_RGB"; break;
        case 0xC3: functionStr = "F_SEND_BRIGHTNESS"; break;
        case 0xC5: functionStr = "F_SEND_RESPONSE"; break;
        case 0xCA: functionStr = "F_SEND_SENSOR_VALUE"; break;
        case 0xCB: functionStr = "F_SEND_SENSOR_VALUE_2"; break;
        case 0xCC: functionStr = "F_SEND_FILE_NUM"; break;
        case 0xCD: functionStr = "F_SEND_PATTERN_NUM"; break;
        case 0xCE: functionStr = "F_SEND_FLAG_BYTE"; break;
        case 0xCF: functionStr = "F_SEND_COMMAND"; break;
        case 0xD0: functionStr = "F_RETURN_ELEM_SECTOR"; break;
        case 0xD1: functionStr = "F_RETURN_ELEM_INFO"; break;
        case 0xD2: functionStr = "F_RETURN_ELEM_STATE"; break;
        case 0xD3: functionStr = "F_RETURN_ELEM_ICON"; break;
        case 0xDE: functionStr = "F_RETURN_ELEM_ERROR"; break;
        default:   functionStr = "FUNCIÓN DESCONOCIDA"; break;
    }
    DPRINTF_COLOR(fnColor(LEF.function), "Función: %s (0x%02X)\n", functionStr.c_str(), LEF.function);

    // Datos
    DPRINTF_COLOR(COLOR_DIM, "DataLen: %u bytes\n", (unsigned)LEF.data.size());
    DPRINTF_COLOR(COLOR_DIM, "Data: ");

    if (LEF.data.empty()) {
        DPRINTLN_COLOR(COLOR_DIM, "No hay datos para esta función.");
    } else if (LEF.function == 0xD0) {
        // ===== F_RETURN_ELEM_SECTOR =====
        const uint8_t sector = LEF.data[0];
        const size_t  pStart = 1;
        const size_t  pLen   = (LEF.data.size() > 1) ? (LEF.data.size() - 1) : 0;

        String sname = sectorName(sector);
        DPRINTF_COLOR(sectorColor(sector), "Sector: %s (0x%02X)  PayloadLen=%u\n",
                      sname.c_str(), sector, (unsigned)pLen);

        // Texto (nombre/desc/localización o nombre/desc de modo)
        if (sector == ELEM_NAME_SECTOR ||
            sector == ELEM_DESC_SECTOR ||
            sector == ELEM_LOCATION_SECTOR ||
            isModeNameDesc(sector)) {

            DEBUG__________printf("%sHEX: ", COLOR_DIM);
            for (size_t i = 0; i < pLen; ++i) {
                DEBUG__________printf("%02X ", LEF.data[pStart + i]);
            }
            DEBUG__________printf("%s\n", COLOR_RESET);

            String txt; txt.reserve(pLen);
            for (size_t i = 0; i < pLen; ++i) {
                uint8_t c = LEF.data[pStart + i];
                if (c == 0) break;
                if (c >= 32 && c <= 126) txt += char(c);
                else if (c == 10 || c == 13) txt += '\n';
                else txt += '.';
            }

            if (isModeName(sector)) {
                DPRINTF_COLOR(COLOR_BRIGHT_WHITE, "Modo %u (NAME) ASCII: \"%s\"\n",
                              modeIndexFromSector(sector), txt.c_str());
            } else if (isModeDesc(sector)) {
                DPRINTF_COLOR(COLOR_BRIGHT_WHITE, "Modo %u (DESC) ASCII: \"%s\"\n",
                              modeIndexFromSector(sector), txt.c_str());
            } else {
                DPRINTF_COLOR(COLOR_BRIGHT_WHITE, "Texto ASCII: \"%s\"\n", txt.c_str());
            }

        } else if (sector == ELEM_SERIAL_SECTOR) {
            DEBUG__________printf("%sSerial HEX: ", COLOR_DIM);
            for (size_t i = 0; i < pLen; ++i) DEBUG__________printf("%02X ", LEF.data[pStart + i]);
            DEBUG__________printf("%s\n", COLOR_RESET);

            if (pLen == 4) {
                uint32_t ser = (uint32_t(LEF.data[pStart])   << 24) |
                               (uint32_t(LEF.data[pStart+1]) << 16) |
                               (uint32_t(LEF.data[pStart+2]) << 8)  |
                               (uint32_t(LEF.data[pStart+3])      );
                DPRINTF_COLOR(COLOR_BRIGHT_CYAN, "Serial DEC: %lu\n", (unsigned long)ser);
            }

        } else if (sector == ELEM_UNUSED_SECTOR) {
            if (pLen >= 1) DPRINTF_COLOR(COLOR_BRIGHT_YELLOW, "ID: %u (0x%02X)\n", LEF.data[pStart], LEF.data[pStart]);
            else           DPRINTLN_COLOR(COLOR_DIM, "ID sin payload.");

        } else if (sector == ELEM_CMODE_SECTOR) {
            if (pLen >= 1) DPRINTF_COLOR(COLOR_BRIGHT_YELLOW, "CurrentMode: %u\n", LEF.data[pStart]);
            else           DPRINTLN_COLOR(COLOR_DIM, "CMODE sin payload.");

        } else if (isModeFlag(sector)) {
            if (pLen >= 2) {
                uint16_t flags = (uint16_t(LEF.data[pStart]) << 8) | LEF.data[pStart+1];
                uint8_t  m     = modeIndexFromSector(sector);
                DPRINTF_COLOR(COLOR_BRIGHT_MAGENTA, "Modo %u FLAGS: 0x%04X  (b15..b0=%s)\n",
                              m, flags, bin16(flags).c_str());
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "FLAGS de modo incompletos (esperados 2 bytes).");
            }

        } else if (isIconRow(sector)) {
            uint8_t row = iconRowIndex(sector);
            if (pLen == 128) {
                uint16_t px0 = (uint16_t(LEF.data[pStart+0]) << 8) | LEF.data[pStart+1];
                uint16_t px1 = (uint16_t(LEF.data[pStart+2]) << 8) | LEF.data[pStart+3];
                DPRINTF_COLOR(COLOR_BRIGHT_BLUE, "ICON ROW %u OK (128B). px0=0x%04X px1=0x%04X ...\n", row, px0, px1);
            } else {
                DPRINTF_COLOR(COLOR_BRIGHT_BLUE, "ICON ROW %u tamaño inesperado: %u (esperado 128)\n",
                              row, (unsigned)pLen);
            }

        } else if (sector == ELEM_FLAG_STATE_SECTOR || sector == ELEM_CURRENT_FLAGS_SECTOR) {
            if (pLen >= 2) {
                uint16_t flags = (uint16_t(LEF.data[pStart]) << 8) | LEF.data[pStart+1];
                DPRINTF_COLOR(COLOR_BRIGHT_MAGENTA, "FLAGS: 0x%04X  (b15..b0=%s)\n", flags, bin16(flags).c_str());
            } else if (pLen >= 1) {
                uint8_t f = LEF.data[pStart];
                DPRINTF_COLOR(COLOR_BRIGHT_MAGENTA, "FLAGS(8b): 0x%02X  (b7..b0=%s)\n",
                              f, bin16(uint16_t(f)).substring(8).c_str());
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "FLAGS sin payload.");
            }

        } else if (sector == ELEM_CURRENT_COLOR_SECTOR) {
            if (pLen >= 1) DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "CURRENT_COLOR idx: %u\n", LEF.data[pStart]);
            else           DPRINTLN_COLOR(COLOR_DIM, "CURRENT_COLOR sin payload.");

        } else if (sector == ELEM_CURRENT_RED_SECTOR  ||
                   sector == ELEM_CURRENT_GREEN_SECTOR||
                   sector == ELEM_CURRENT_BLUE_SECTOR) {
            if (pLen >= 1) DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "RGB comp: %u\n", LEF.data[pStart]);
            else           DPRINTLN_COLOR(COLOR_DIM, "RGB comp sin payload.");

        } else if (sector == ELEM_CURRENT_BRIGHTNESS_SECTOR) {
            if (pLen >= 1) DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "BRIGHTNESS: %u\n", LEF.data[pStart]);
            else           DPRINTLN_COLOR(COLOR_DIM, "BRIGHTNESS sin payload.");

        } else if (sector == ELEM_CURRENT_PATTERN_SECTOR ||
                   sector == ELEM_CURRENT_FILE_SECTOR    ||
                   sector == ELEM_CURRENT_XMANAGER_SECTOR) {
            if (pLen >= 1) DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "VAL: %u\n", LEF.data[pStart]);
            else           DPRINTLN_COLOR(COLOR_DIM, "Valor sin payload.");

        } else if (sector == ELEM_MOST_USED_MODE_SECTOR ||
                   sector == ELEM_MOST_USED_COLOR_SECTOR||
                   sector == ELEM_MOST_USED_PATTERN_SECTOR) {
            DEBUG__________printf("%sTOP stats payload (HEX): ", COLOR_DIM);
            for (size_t i = 0; i < pLen; ++i) DEBUG__________printf("%02X ", LEF.data[pStart + i]);
            DEBUG__________printf("%s\n", COLOR_RESET);

        } else if (sector == ELEM_TOTAL_SESSION_TIME_SECTOR) {
            if (pLen == 4) {
                uint32_t t = (uint32_t(LEF.data[pStart])   << 24) |
                             (uint32_t(LEF.data[pStart+1]) << 16) |
                             (uint32_t(LEF.data[pStart+2]) << 8)  |
                             (uint32_t(LEF.data[pStart+3])      );
                DPRINTF_COLOR(COLOR_BRIGHT_CYAN, "TOTAL_SESSION_TIME: %lu\n", (unsigned long)t);
            } else {
                DEBUG__________printf("%sTOTAL_SESSION_TIME (HEX): ", COLOR_DIM);
                for (size_t i = 0; i < pLen; ++i) DEBUG__________printf("%02X ", LEF.data[pStart + i]);
                DEBUG__________printf("%s\n", COLOR_RESET);
            }

        } else if (sector == ACTIVE_ELEM_LIST) {
            DEBUG__________printf("%sACTIVE_ELEM_LIST (HEX): ", COLOR_DIM);
            for (size_t i = 0; i < pLen; ++i) DEBUG__________printf("%02X ", LEF.data[pStart + i]);
            DEBUG__________printf("%s\n", COLOR_RESET);

            String txt; txt.reserve(pLen);
            for (size_t i = 0; i < pLen; ++i) {
                uint8_t c = LEF.data[pStart + i];
                if (c == 0) break;
                if (c >= 32 && c <= 126) txt += char(c);
                else if (c == 10 || c == 13) txt += '\n';
                else txt += '.';
            }
            if (txt.length()) {
                DPRINTF_COLOR(COLOR_BRIGHT_WHITE, "ACTIVE_ELEM_LIST (ASCII): \"%s\"\n", txt.c_str());
            }

        } else {
            DEBUG__________printf("%sPayload (HEX): ", COLOR_DIM);
            for (size_t i = 0; i < pLen; ++i) DEBUG__________printf("%02X ", LEF.data[pStart + i]);
            DEBUG__________printf("%s\n", COLOR_RESET);

            String txt; txt.reserve(pLen);
            for (size_t i = 0; i < pLen; ++i) {
                uint8_t c = LEF.data[pStart + i];
                if (c == 0) break;
                if (c >= 32 && c <= 126) txt += char(c);
                else if (c == 10 || c == 13) txt += '\n';
                else txt += '.';
            }
            if (txt.length()) {
                DPRINTF_COLOR(COLOR_BRIGHT_WHITE, "Payload (ASCII): \"%s\"\n", txt.c_str());
            }
        }

    } else {
        // ===== Otras funciones =====
        if (LEF.function == 0xCA) {
            // SENSOR_DOUBLE X/Y
            if (LEF.data.size() >= 12) {
                int minX = (LEF.data[0]  << 8) | LEF.data[1];
                int maxX = (LEF.data[2]  << 8) | LEF.data[3];
                int valX = (LEF.data[4]  << 8) | LEF.data[5];
                int minY = (LEF.data[6]  << 8) | LEF.data[7];
                int maxY = (LEF.data[8]  << 8) | LEF.data[9];
                int valY = (LEF.data[10] << 8) | LEF.data[11];
                DPRINTF_COLOR(COLOR_BRIGHT_GREEN,
                              "Eje X → MIN=%d, MAX=%d, VAL=%d;  Eje Y → MIN=%d, MAX=%d, VAL=%d\n",
                              minX, maxX, valX, minY, maxY, valY);
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "Trama SENSOR CA incompleta (esperados 12 bytes).");
            }

        } else if (LEF.function == 0xCB) {
            // SENSOR_SINGLE
            if (LEF.data.size() >= 6) {
                int minVal    = (LEF.data[0] << 8) | LEF.data[1];
                int maxVal    = (LEF.data[2] << 8) | LEF.data[3];
                int sensedVal = (LEF.data[4] << 8) | LEF.data[5];
                DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "MIN=%d, MAX=%d, VAL=%d\n", minVal, maxVal, sensedVal);
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "Trama SENSOR CB incompleta (esperados 6 bytes).");
            }

        } else if (LEF.function == F_SEND_COLOR) {
            // Color básico
            String colorName;
            switch (LEF.data[0]) {
                case 0:  colorName = "BLANCO";   break;
                case 1:  colorName = "AMARILLO"; break;
                case 2:  colorName = "NARANJA";  break;
                case 3:  colorName = "ROJO";     break;
                case 4:  colorName = "VIOLETA";  break;
                case 5:  colorName = "AZUL";     break;
                case 6:  colorName = "CELESTE";  break;
                case 7:  colorName = "VERDE";    break;
                case 8:  colorName = "NEGRO";    break;
                default: colorName = "COLOR DESCONOCIDO"; break;
            }
            DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "%s (0x%02X)\n", colorName.c_str(), LEF.data[0]);

        } else if (LEF.function == 0xC2) {
            // RGB
            if (LEF.data.size() >= 3) {
                DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "RGB: R=%u G=%u B=%u\n",
                              LEF.data[0], LEF.data[1], LEF.data[2]);
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "Trama RGB incompleta (esperados 3 bytes).");
            }

        } else if (LEF.function == 0xC3) {
            // BRIGHTNESS
            if (LEF.data.size() >= 1) {
                DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "Brightness: %u\n", LEF.data[0]);
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "Trama BRIGHTNESS incompleta (esperado 1 byte).");
            }

        } else if (LEF.function == 0xCE) {
            // FLAG BYTE (ej. relé)
            if (LEF.data.size() >= 1) {
                uint8_t fb = LEF.data[0];
                DPRINTF_COLOR(COLOR_BRIGHT_MAGENTA, "FLAG BYTE: 0x%02X (%u) → %s\n",
                              fb, fb, (fb ? "ON" : "OFF"));
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "No hay datos para FLAG BYTE.");
            }

        } else if (LEF.function == 0xA0) {
            // Petición de sector
            String idioma = (LEF.data.size() >= 1 && LEF.data[0] == 1) ? "ES" : "OTRO";
            uint8_t sec = (LEF.data.size() >= 2) ? LEF.data[1] : 0xFF;
            if (LEF.data.size() >= 2) {
                String sn = sectorName(sec);
                DPRINTF_COLOR(COLOR_BRIGHT_YELLOW, "Idioma: %s, Sector: %u (%s)\n",
                              idioma.c_str(), sec, sn.c_str());
            } else {
                DPRINTF_COLOR(COLOR_BRIGHT_YELLOW, "Idioma: %s, Sector: --\n", idioma.c_str());
            }

        } else if (LEF.function == 0xCC) {
            // F_SEND_FILE_NUM
            if (LEF.data.size() >= 2) {
                DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "Carpeta (Bank): %d, Archivo (File): %d\n",
                              LEF.data[0], LEF.data[1]);
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "Trama FILE_NUM incompleta (esperados 2 bytes).");
            }

        } else if (LEF.function == F_SEND_RESPONSE) {
            if (inCognitiveMenu) DPRINTF_COLOR(COLOR_DIM, "Response recibido: %d\n", LEF.data[0]);
            else{
                DPRINTF_COLOR(COLOR_DIM, "Response recibido fuera de modo COG → interpretado como color ");

                String colorName;
                switch (LEF.data[0]) {
                    case 0: colorName = "BLANCO";   break;
                    case 1: colorName = "AMARILLO"; break;
                    case 2: colorName = "NARANJA";  break;
                    case 3: colorName = "ROJO";     break;
                    case 4: colorName = "VIOLETA";  break;
                    case 5: colorName = "AZUL";     break;
                    case 6: colorName = "CELESTE";  break;
                    case 7: colorName = "VERDE";    break;
                    case 8: colorName = "NEGRO";    break;
                    default: colorName = "DESCONOCIDO"; break;
                }
                DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "→ %s (0x%02X)\n", colorName.c_str(), LEF.data[0]);
            }
        } else if (LEF.function == F_SEND_COMMAND) {
            // Comandos
            if (!LEF.data.empty()) {
                uint8_t cmd = LEF.data[0];
                String commandStr;
                switch (cmd) {
                    case BLACKOUT:                     commandStr = "BLACKOUT"; break;
                    case START_CMD:                    commandStr = "START_CMD"; break;
                    case SLEEP_SERIAL_WAKEUP_CMD:      commandStr = "SLEEP_SERIAL_WAKEUP_CMD"; break;
                    case SLEEP_RANDOM_WAKEUP_CMD:      commandStr = "SLEEP_RANDOM_WAKEUP_CMD"; break;
                    case SEND_REG_RF_CMD:              commandStr = "SEND_REG_RF_CMD"; break;
                    case SEND_STATS_RF_CMD:            commandStr = "SEND_STATS_RF_CMD"; break;
                    case ERR_DBG_ON:                   commandStr = "ERR_DBG_ON"; break;
                    case ERR_DBG_OFF:                  commandStr = "ERR_DBG_OFF"; break;
                    case SET_ELEM_DEAF:                commandStr = "SET_ELEM_DEAF"; break;
                    case SET_ELEM_LONG_DEAF:           commandStr = "SET_ELEM_LONG_DEAF"; break;
                    case MAGIC_TEST_CMD:               commandStr = "MAGIC_TEST_CMD"; break;
                    case MAGIC_TEST_2_CMD:             commandStr = "MAGIC_TEST_2_CMD"; break;
                    case ALTERNATE_MODE_ON:            commandStr = "ALTERNATE_MODE_ON"; break;
                    case ALTERNATE_MODE_OFF:           commandStr = "ALTERNATE_MODE_OFF"; break;
                    case OTA_AP_ON:                    commandStr = "OTA_AP_ON"; break;
                    case OTA_AP_OFF:                   commandStr = "OTA_AP_OFF"; break;
                    case COG_ACT_ON:                   commandStr = "COG_ACT_ON"; break;
                    case COG_ACT_OFF:                  commandStr = "COG_ACT_OFF"; break;
                    case SHOW_ID_CMD:                  commandStr = "SHOW_ID_CMD"; break;
                    case WIN_CMD:                      commandStr = "WIN_CMD"; break;
                    case FAIL_CMD:                     commandStr = "FAIL_CMD"; break;
                    case MOST_USED_MODE_RF_REQ:        commandStr = "MOST_USED_MODE_RF_REQ"; break;
                    case MOST_USED_COLOR_RF_REQ:       commandStr = "MOST_USED_COLOR_RF_REQ"; break;
                    case MOST_USED_AMBIENT_RF_REQ:     commandStr = "MOST_USED_AMBIENT_RF_REQ"; break;
                    case LIFETIME_TOTAL_RF_REQ:        commandStr = "LIFETIME_TOTAL_RF_REQ"; break;
                    case WORKTIME_TOTAL_RF_REQ:        commandStr = "WORKTIME_TOTAL_RF_REQ"; break;
                    case CURRENT_SESSION_TIME_RF_REQ:  commandStr = "CURRENT_SESSION_TIME_RF_REQ"; break;
                    case CURRENT_SESSION_FILENAME_RF_REQ: commandStr = "CURRENT_SESSION_FILENAME_RF_REQ"; break;
                    case EVENTS_LOG_RF_REQ:            commandStr = "EVENTS_LOG_RF_REQ"; break;
                    case LAST_EVENT_LOG_RF_REQ:        commandStr = "LAST_EVENT_LOG_RF_REQ"; break;
                    case LIST_SESSIONS_RF_REQ:         commandStr = "LIST_SESSIONS_RF_REQ"; break;
                    case CLEAR_SESSIONS_CMD:           commandStr = "CLEAR_SESSIONS_CMD"; break;
                    case RESET_ALL_STATS_CMD:          commandStr = "RESET_ALL_STATS_CMD"; break;
                    case SEND_LAST_ORIGIN_CMD:         commandStr = "SEND_LAST_ORIGIN_CMD"; break;
                    case SEND_SESSION_LOG_RF_CMD:      commandStr = "SEND_SESSION_LOG_RF_CMD"; break;
                    case FORMAT_LITTLEFS_CMD:          commandStr = "FORMAT_LITTLEFS_CMD"; break;
                    case AVERAGE_SESSION_TIME_RF_REQ:  commandStr = "AVERAGE_SESSION_TIME_RF_REQ"; break;
                    case MOST_SELECTED_MODE_RF_REQ:    commandStr = "MOST_SELECTED_MODE_RF_REQ"; break;
                    case MOST_SELECTED_COLOR_RF_REQ:   commandStr = "MOST_SELECTED_COLOR_RF_REQ"; break;
                    case MOST_SELECTED_AMBIENT_RF_REQ: commandStr = "MOST_SELECTED_AMBIENT_RF_REQ"; break;
                    case USAGE_GRAPH_RF_REQ:           commandStr = "USAGE_GRAPH_RF_REQ"; break;
                    case LITTLEFS_MEM_STATS:           commandStr = "LITTLEFS_MEM_STATS"; break;
                    case INTER_SESSION_TIMES:          commandStr = "INTER_SESSION_TIMES"; break;
                    default:                           commandStr = "COMANDO DESCONOCIDO"; break;
                }

                DPRINTF_COLOR(COLOR_BRIGHT_GREEN, "Comando: %s (0x%02X)\n", commandStr.c_str(), cmd);
            } else {
                DPRINTLN_COLOR(COLOR_DIM, "No hay datos para el comando.");
            }
        } else {
            // Volcado por defecto (HEX + ASCII)
            DEBUG__________printf("%s", COLOR_DIM);
            for (size_t i = 0; i < LEF.data.size(); i++) {
                DEBUG__________printf("0x%02X ", LEF.data[i]);
            }
            DEBUG__________printf("%s\n", COLOR_RESET);

            String txt; txt.reserve(LEF.data.size());
            for (size_t i = 0; i < LEF.data.size(); ++i) {
                uint8_t c = LEF.data[i];
                if (c == 0) break;
                if (c >= 32 && c <= 126) txt += char(c);
                else if (c == 10 || c == 13) txt += '\n';
                else txt += '.';
            }
            if (txt.length()) {
                DPRINTF_COLOR(COLOR_BRIGHT_WHITE, "ASCII: \"%s\"\n", txt.c_str());
            }
        }
    }

    DPRINTLN_COLOR(COLOR_BRIGHT_WHITE, "==============================");
}


/**
 * @brief Despacha y atiende un frame RX de control para la botonera.
 *
 * Procesa el campo de función de @p LEF y ejecuta la acción correspondiente:
 * sincronización de sector, actualización de estado de relé, reproducción de
 * sonidos y gestión de comandos (p. ej., modo cognitivo y respuestas WIN/FAIL).
 *
 * @param LEF Estructura de frame recibido. Se espera que contenga:
 * - `function`: código de operación (p. ej. F_RETURN_ELEM_SECTOR, F_SEND_COMMAND, ...).
 * - `data[]`: payload; se usa `data[0]` y, según el caso, `data[1]`.
 * - `origin`: identificador de origen, reenviado a `sectorIn_handler`.
 *
 * @return void
 *
 * @pre `element` debe ser un puntero válido e inicializado.
 * @pre El contexto debe ser de tarea (no ISR) si se usa `uxTaskGetStackHighWaterMark`.
 * @pre Cuando `function == F_SEND_FILE_NUM` se requiere `LEF.data[0]` (banco) y `LEF.data[1]` (fichero).
 * @pre Cuando `function == F_SEND_COMMAND` se requiere `LEF.data[0]` (código de comando).
 *
 * @note Las dependencias externas (`RelayStateManager`, `doitPlayer`, `activateCognitiveMode`,
 * `deactivateCognitiveMode`, `printFrameInfo`, `sectorIn_handler`, etc.) deben estar disponibles.
 *
 * @warning Este manejador asume que `LEF.data` ofrece al menos 12 bytes según la función.
 * Validar tamaño aguas arriba para evitar accesos fuera de rango.
 *
 * @see activateCognitiveMode, deactivateCognitiveMode, RelayStateManager::set, doitPlayer.play_file
 */

 void BOTONERA_::RX_main_handler(LAST_ENTRY_FRAME_T LEF) {
    // -------- Helpers NS-only (no IDs) --------
    auto nsIsZero = [](const TARGETNS& ns) -> bool {
        return ns.mac01==0 && ns.mac02==0 && ns.mac03==0 && ns.mac04==0 && ns.mac05==0;
    };

    // Para frames que traen "subcódigo" en data[0] (sectores, etc.): NS en data[1..5]
    auto extractSenderNS_fromSectorPayload = [&](const std::vector<byte>& d, TARGETNS& out) -> bool {
        if (d.size() >= 6) {
            out = { d[1], d[2], d[3], d[4], d[5] };
            return !nsIsZero(out);
        }
        return false;
    };

    // Para F_SEND_FLAG_BYTE (flags en data[0]): NS en data[1..5]
    auto extractSenderNS_fromFlagPayload = [&](const std::vector<byte>& d, TARGETNS& out) -> bool {
        if (d.size() >= 6) {
            out = { d[1], d[2], d[3], d[4], d[5] };
            return !nsIsZero(out);
        }
        return false;
    };

    // -------- Validaciones básicas --------
    if (!element) {
    #ifdef DEBUG
        DEBUG__________ln("Error: 'element' no está inicializado.");
    #endif
        return;
    }

    uint8_t localRoom = getLocalRoom();
    if (LEF.room != ROOM_BROADCAST && LEF.room != localRoom) {
#ifdef DEBUG
        DEBUG__________printf("[ROOM] Frame descartado (0x%02X != local 0x%02X)\n", LEF.room, localRoom);
#endif
        return;
    }
    // Trazas del frame entrante
    printFrameInfo(LEF);

    // (Solo en depuración) agua alta de la pila
    #ifdef DEBUG
    {
        UBaseType_t stackSizeBegin = uxTaskGetStackHighWaterMark(NULL);
        (void)stackSizeBegin;
    }
    #endif

    // Constantes
    constexpr uint8_t kFlagsBit0Mask    = 0x01u; // Bit 0: estado del relé remoto.
    constexpr uint8_t kIdx0             = 0u;
    constexpr uint8_t kIdx1             = 1u;
    constexpr uint8_t kLangStride       = 10u;
    constexpr uint8_t kWinFailVariants  = 4u;

    // Despacho
    switch (LEF.function) {

        case F_RETURN_ELEM_SECTOR: {
            // Mientras escaneamos: encola y sal
            if (scanInProgress) {
                if (!LEF.data.empty()) {
                    SectorMsg m;
                    m.sector = LEF.data[0];
                    if (LEF.data.size() > 1) {
                        m.payload.assign(LEF.data.begin() + 1, LEF.data.end());
                    }
                    m.tsMs = millis();
                    rxSectorInbox.emplace_back(std::move(m));
                }
                return;
            }

            if (LEF.data.empty()) {
                DEBUG__________ln("F_RETURN_ELEM_SECTOR sin datos. Ignorado.");
                break;
            }

            TARGETNS senderNS = LEF.originNS;
            if (nsIsZero(senderNS)) {
                senderNS = TARGETNS{0,0,0,0,0};
            }
            element->sectorIn_handler(LEF.data, senderNS, LEF.origin);

            // Si era el sector CMODE, cerramos la espera para que NO dispare el timeout de 500ms
            if (awaitingResponse && LEF.data[0] == ELEM_CMODE_SECTOR) {
                awaitingResponse = false;
            }
            break;
        }
        
        case F_SET_BUTTONS_EXTMAP: {

            const auto &d = LEF.data;

            // Esperamos 55 bytes exactos: 1 + 9*6
            if (d.size() != (size_t)(1 + 9 * 6)) break;

           
            auto colorFrom = [&](uint8_t numColor, uint8_t r, uint8_t g, uint8_t b) -> CRGB {
                if (numColor == BTN_RGB_DIRECT)
                    return CRGB(r, g, b);

                if (numColor < 36) {
                    unsigned int hexColor = listaColores[numColor];
                    uint8_t red   = (hexColor >> 16) & 0xFF;
                    uint8_t green = (hexColor >> 8)  & 0xFF;
                    uint8_t blue  = hexColor & 0xFF;
                    return CRGB(red, green, blue);
                }

                return CRGB(0, 0, 0); // Color por defecto (negro)
            };

            // --- Leer PADFX ---
            const uint8_t PADFX = d[0];

            // --- Parsear botones ---
            struct DecodedBtn { bool active; CRGB base; uint8_t fx; };
            DecodedBtn btn[9]{};

            size_t off = 1;
            for (int i = 0; i < 9; ++i) {
                const uint8_t active   = d[off + 0];
                const uint8_t numColor = d[off + 1];
                const uint8_t r        = d[off + 2];
                const uint8_t g        = d[off + 3];
                const uint8_t b        = d[off + 4];
                const uint8_t fx       = d[off + 5];
                off += 6;

                btn[i].active = (active != 0);
                btn[i].base   = colorFrom(numColor, r, g, b);
                btn[i].fx     = fx;

                // NUEVO: guardar configuracion para respuestas
                PulsadoresHandler::setButtonMappedNumColor(i, numColor);
                PulsadoresHandler::setButtonMappedFx(i, fx);
            }


            // --- Actualiza la máscara de habilitados segun 'active' recibido ---
            bool mask[9];
            for (int i = 0; i < 9; ++i) mask[i] = btn[i].active;
            PulsadoresHandler::setButtonActiveMask(mask);


            // =======================
            //  A) EFECTO GLOBAL
            // =======================
            if (PADFX != (uint8_t)BTN_FX::NO_FX) {

                bool allOff[9] = {false,false,false,false,false,false,false,false,false};
                PulsadoresHandler::setButtonActiveMask(allOff);
                PulsadoresHandler::setGlobalFxNoInput(true);
                ledManager.clearEffects();

                // Color base: primero activo (RGB directo si está)
                // CRGB base = CRGB::White;
                // bool found = false;
                // for (int i = 0; i < 9 && !found; ++i) {
                //     if (!btn[i].active) continue;
                //     base = btn[i].base;
                //     found = true;
                // }
                // if (!found) base = CRGB::White;

                CRGB base = CRGB::White;
                bool found = false;
                for (int i = 0; i < 9; ++i) {
                    const CRGB c = btn[i].base;
                    if (c.r || c.g || c.b) { // primer color distinto de negro
                        base = c;
                        found = true;
                        break;
                    }
                }

                auto addWaveAll = [&](unsigned ms) {
                    for (int i = 0; i < colorHandler.numLeds; ++i)
                        ledManager.addEffect(new WaveEffect(colorHandler, i, base, ms));
                    fill_solid(colorHandler.leds, colorHandler.numLeds, base);
                };

                // Mapea PADFX -> efectos globales
                switch (static_cast<BTN_FX>(PADFX)) {
                    case BTN_FX::SOLID: {
                        fill_solid(colorHandler.leds, colorHandler.numLeds, base);
                        break;
                    }

                    case BTN_FX::SLOW_WAVE: addWaveAll(80); break;
                    case BTN_FX::FAST_WAVE: addWaveAll(20); break;

                    case BTN_FX::RAINBOWLOOP: {
                        unsigned ms = 20;
                        for (int i = 0; i < colorHandler.numLeds; ++i)
                            ledManager.addEffect(new RainbowLoopEffect(colorHandler, i, uint8_t(i * 21), ms));
                        break;
                    }

                    case BTN_FX::BUBBLES_FX: {
                        // Nuevo efecto global especí­fico de "burbujas" (no Sparkle)
                        unsigned ms = 30; uint8_t dens = 28; uint8_t fade = 40;
                        ledManager.addEffect(new BubblesGlobalEffect(colorHandler, base, ms, dens, fade));
                        // Base como rellenado suave para que se vea ambiente
                        fill_solid(colorHandler.leds, colorHandler.numLeds, base);
                        break;
                    }

                    case BTN_FX::BREATHING: {
                        // Breathing distinto de Wave: fundidos base <-> negro
                        for (int i = 0; i < colorHandler.numLeds; ++i)
                            ledManager.addEffect(new FadeEffect(colorHandler, i, base, CRGB::Black, /*period*/80));
                        break;
                    }

                    case BTN_FX::FIRE: {
                        unsigned ms = 25;
                        ledManager.addEffect(new FireGlobalEffect(colorHandler, ms));
                        break;
                    }

                    case BTN_FX::SPARKLE: {
                        unsigned ms = 16; uint8_t dens = 64; uint8_t fade = 40;
                        ledManager.addEffect(new SparkleGlobalEffect(colorHandler, base, ms, dens, fade));
                        fill_solid(colorHandler.leds, colorHandler.numLeds, base);
                        break;
                    }

                    case BTN_FX::COMET: {
                        static const uint8_t kPath[9] = {8,6,4,2,0,1,3,5,7};
                        unsigned ms = 18;
                        ledManager.addEffect(new CometTrailEffect(colorHandler, kPath, 9, base, ms, /*decay*/100));
                        break;
                    }

                    case BTN_FX::PLASMA: {
                        unsigned ms = 16;
                        ledManager.addEffect(new PlasmaNoiseEffect(colorHandler, base, ms));
                        break;
                    }

                    case BTN_FX::HEARTBEAT: {
                        for (int i = 0; i < colorHandler.numLeds; ++i)
                            ledManager.addEffect(new HeartbeatEffect(colorHandler, i, base, /*ms*/28));
                        fill_solid(colorHandler.leds, colorHandler.numLeds, base);
                        break;
                    }

                    case BTN_FX::AURORA_FX: {
                        unsigned ms = 30;
                        ledManager.addEffect(new AuroraGlobalEffect(colorHandler, base, ms));
                        break;
                    }

                    case BTN_FX::STROBE: {
                        unsigned ms = 8;
                        ledManager.addEffect(new StrobeGlobalEffect(colorHandler, base, ms, /*flashes*/3, /*gapMs*/120));
                        break;
                    }

                    case BTN_FX::COLOR_WIPE: {
                        static const uint8_t kOrder[9] = {8,6,4,2,0,1,3,5,7};
                        unsigned ms = 30;
                        fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);
                        ledManager.addEffect(new ColorWipeGlobalEffect(colorHandler, kOrder, 9, base, ms));
                        break;
                    }

                    case BTN_FX::THEATER_CHASE: {
                        unsigned ms = 20;
                        ledManager.addEffect(new TheaterChaseGlobalEffect(colorHandler, base, ms, /*spacing*/3));
                        break;
                    }

                    case BTN_FX::LOADING: {
                    // Punto que recorre de izq -> dcha cí­clicamente con pequeña cola
                    // Velocidad sugerida: ~40 ms/step (ajustable)
                    const unsigned periodMs = 40;
                    const uint8_t  trailLen = 2;   // nº de LEDs de la cola
                    const uint8_t  baseDim  = 16;  // brillo ambiente del fondo (0=negro)

                    ledManager.addEffect(new LoadingGlobalEffect(colorHandler, base, periodMs, trailLen, baseDim));
                    break;
                    }

                    case BTN_FX::LOADING_CIRCULAR: {
                        constexpr uint8_t PATH_CIRCULAR[9] = { 8, 6, 4, 2, 0, 1, 3, 5, 7 };
                        Serial.println("LOADING_CIRCULAR");
                        unsigned periodMs = 100;
                        LoadingEffect::configurePath(PATH_CIRCULAR, sizeof(PATH_CIRCULAR));
                        ledManager.addEffect(new LoadingEffect(colorHandler, base, periodMs));
                        break;
                    }

                    case BTN_FX::LOADING_LINEAL: {
                        constexpr uint8_t PATH_LINEAL[9]   = { 8, 6, 4, 2, 0, 7, 5, 3, 1 };
                        Serial.println("LOADING_LINEAL");
                        unsigned periodMs = 100;
                        LoadingEffect::configurePath(PATH_LINEAL, sizeof(PATH_LINEAL));
                        ledManager.addEffect(new LoadingEffect(colorHandler, base, periodMs));
                        break;
                    }
                    default:
                        // Desconocido => sólido
                        fill_solid(colorHandler.leds, colorHandler.numLeds, base);
                        break;
                }

                FastLED.show();
                break; // no procesamos por-LED
            }

            // =======================
            //  B) EFECTOS POR LED
            // =======================
            ledManager.clearEffects();
            PulsadoresHandler::setGlobalFxNoInput(false); 
            for (int ledIdx = 0; ledIdx < 9 && ledIdx < colorHandler.numLeds; ++ledIdx) {

                const CRGB     base = btn[ledIdx].base;
                const BTN_FX   fx   = static_cast<BTN_FX>(btn[ledIdx].fx);

                switch (fx) {
                    case BTN_FX::NO_FX:
                    case BTN_FX::SOLID: {
                        colorHandler.leds[ledIdx] = base;
                        break;
                    }

                    case BTN_FX::SLOW_WAVE: {
                        ledManager.addEffect(new WaveEffect(colorHandler, ledIdx, base, /*ms*/60));
                        break;
                    }

                    case BTN_FX::FAST_WAVE: {
                        ledManager.addEffect(new WaveEffect(colorHandler, ledIdx, base, /*ms*/20));
                        break;
                    }

                    case BTN_FX::RAINBOWLOOP: {
                        ledManager.addEffect(new RainbowLoopEffect(colorHandler, ledIdx, /*startHue*/uint8_t(ledIdx*21), /*ms*/20));
                        break;
                    }

                    case BTN_FX::BUBBLES_FX: {
                        // Burbujas por LED (chisporroteo suave local)
                        ledManager.addEffect(new BubblesEffect(colorHandler, ledIdx, base, /*tick*/40, /*dens*/28));
                        break;
                    }

                    case BTN_FX::BREATHING: {
                        // Breathing local con fundido al negro (diferente de Wave)
                        ledManager.addEffect(new FadeEffect(colorHandler, ledIdx, base, CRGB::Black, /*period*/80));
                        break;
                    }

                    case BTN_FX::FIRE: {
                        // Nuevo: vela realista por LED (no Bubbles)
                        CRGB warm = base; 
                        warm.g = qadd8(warm.g, warm.r >> 2); 
                        warm.b >>= 2;
                        ledManager.addEffect(new CandleFlickerEffect(colorHandler, ledIdx, warm, /*tick*/22));
                        break;
                    }

                    case BTN_FX::SPARKLE: {
                        // Nuevo: chispa local sobre el pixel (no Bubbles)
                        ledManager.addEffect(new SparkleLocalEffect(colorHandler, ledIdx, base, /*tick*/16));
                        break;
                    }

                    case BTN_FX::COMET: {
                        // Nuevo: paso de cometa local (pulso con decaimiento)
                        ledManager.addEffect(new CometLocalEffect(colorHandler, ledIdx, base, /*period*/18, /*decay*/100));
                        break;
                    }

                    case BTN_FX::PLASMA: {
                        // Nuevo: ruido local con tinte base
                        ledManager.addEffect(new PlasmaLocalEffect(colorHandler, ledIdx, base, /*tick*/12));
                        break;
                    }

                    case BTN_FX::HEARTBEAT: {
                        ledManager.addEffect(new HeartbeatEffect(colorHandler, ledIdx, base, /*ms*/28));
                        break;
                    }

                    case BTN_FX::AURORA_FX: {
                        ledManager.addEffect(new AuroraEffect(colorHandler, ledIdx, base, CRGB::Cyan, /*ms*/35));
                        break;
                    }

                    case BTN_FX::STROBE: {
                        ledManager.addEffect(new StrobeEffect(colorHandler, ledIdx, base, /*updateMs*/12, /*flashes*/3));
                        break;
                    }

                    case BTN_FX::COLOR_WIPE: {
                        // relleno del pixel con rampa
                        ledManager.addEffect(new ColorWipeEffect(colorHandler, ledIdx, base, /*ms*/40, /*fill*/true));
                        break;
                    }

                    case BTN_FX::THEATER_CHASE: {
                        ledManager.addEffect(new TheaterChaseEffect(colorHandler, ledIdx, base, /*ms*/35, /*spacing*/3));
                        break;
                    }

                    case BTN_FX::RELAY_FX: {
                        if (ledIdx == 0) {
                            ledManager.addEffect(new FadeEffect(colorHandler, 0, CRGB::Blue, CRGB::Cyan, 50));
                            if (colorHandler.numLeds > 8) colorHandler.leds[8] = CRGB::Blue;
                        }
                        break;
                    }
                }
            }

            // 1) Determinar el destino al que contestaremos:
            //    - Si el ORIGEN del frame es DC (consola) o la propia botonera lógica: NS = 00:00:00:00:00
            //    - Si el ORIGEN es DD (dispositivo): usar su NS (el del origen)
            uint8_t originType = LEF.origin;
            TARGETNS respNS;

            if (originType == DEFAULT_CONSOLE || originType == DEFAULT_BOTONERA) {
                // Consola / Botonera lógica -> NS cero
                respNS = TARGETNS{0,0,0,0,0};
            } else {
                // Dispositivo físico -> responder a su NS
                respNS = LEF.originNS;
            }

            // 2) Habilitar el modo respuesta en los pulsadores
            PulsadoresHandler::setResponseRoute(originType, respNS);

            FastLED.show();
            break;

        }

        case F_REQ_ELEM_SECTOR: {
            //DEBUG__________ln("Recibido un F_REQ_ELEM_SECTOR");
            if (LEF.data.empty()) {
                DEBUG__________ln("F_REQ_ELEM_SECTOR sin datos. Ignorado.");
                break;
            }

            // Data[0]=language, Data[1]=sector (si viene en formato largo)
            const uint8_t sectorRequested = (LEF.data.size() >= 2) ? LEF.data[1] : LEF.data[0];

            TARGETNS originNS = LEF.originNS;
            if (nsIsZero(originNS)) {
                extractSenderNS_fromSectorPayload(LEF.data, originNS);
            }

            // Reenví­a SOLO el sector en data[0] sectorIn_handler lo espera así­.
            std::vector<uint8_t> data1{ sectorRequested };
            DEBUG__________ln("sectorRequested: " + String(sectorRequested));
            DEBUG__________ln("originNS: " + String(originNS.mac01) + ":" + String(originNS.mac02) + ":" +
                                        String(originNS.mac03) + ":" + String(originNS.mac04) + ":" + String(originNS.mac05));
            DEBUG__________ln("originType: " + String(LEF.origin));

            element->sectorIn_handler(data1, originNS, LEF.origin);
            break;
        }

        case F_SEND_FLAG_BYTE: {
            if (LEF.data.empty()) {
                DEBUG__________ln("F_SEND_FLAG_BYTE sin datos. Ignorado.");
                break;
            }
            const bool flags_bit0 = (LEF.data[kIdx0] & kFlagsBit0Mask) != 0u;

            TARGETNS senderNS = LEF.originNS;
            if (nsIsZero(senderNS)) {
                if (!extractSenderNS_fromFlagPayload(LEF.data, senderNS)) {
                    DEBUG__________ln("F_SEND_FLAG_BYTE sin senderNS válido. Ignorado.");
                    break;
                }
            }

            // NS estado de relé (sin fallback a ID)
            RelayStateManager::set(senderNS, flags_bit0);
            DEBUG__________printf("Relé [%02X:%02X:%02X:%02X:%02X] => %s\n",
                                  senderNS.mac01, senderNS.mac02, senderNS.mac03, senderNS.mac04, senderNS.mac05,
                                  flags_bit0 ? "ON" : "OFF");
            break;
        }

        case F_SEND_PATTERN_NUM: {
            if (LEF.data.empty()) {
                #ifdef DEBUG
                DEBUG__________ln("F_SEND_PATTERN_NUM sin datos (esperado 1 byte de patrón).");
                #endif
                break;
            }
            const uint8_t pattern = LEF.data[0];
            #ifdef DEBUG
            DEBUG__________printf("F_SEND_PATTERN_NUM bank=0x%02X, file=%u\n", AMBIENTS_BANK, pattern);
            #endif
            doitPlayer.play_file(AMBIENTS_BANK, pattern);
            enfocarElemento("Ambientes");
            std::fill(selectedStates.begin(), selectedStates.end(), false);
            // Marcar el actual como seleccionado
            if (currentIndex >= 0 && currentIndex < (int)selectedStates.size()) {
                selectedStates[currentIndex] = true;
            }
            drawCurrentElement();
            break;
        }

        case F_SEND_RESPONSE: {
            if (LEF.data.empty()) break;
            
            // Filtro de target: solo aceptar si es para nosotros (DB) o Broadcast
            if (LEF.targetType != DEFAULT_BOTONERA && LEF.targetType != BROADCAST) {
                #ifdef DEBUG
                DEBUG__________printf("F_SEND_RESPONSE ignorado: TargetType=0x%02X\n", LEF.targetType);
                #endif
                break;
            }

            uint8_t responseType = LEF.data[0];
            if (responseType == WIN) {
                const uint8_t res  = static_cast<uint8_t>(rand() % kWinFailVariants);
                const uint8_t lang = static_cast<uint8_t>(currentLanguage);
                const uint8_t file = static_cast<uint8_t>(lang * kLangStride + res + 1u);
                doitPlayer.play_file(WIN_RESP_BANK, file);
            } else if (responseType == FAIL) {
                const uint8_t res  = static_cast<uint8_t>(rand() % kWinFailVariants);
                const uint8_t lang = static_cast<uint8_t>(currentLanguage);
                const uint8_t file = static_cast<uint8_t>(lang * kLangStride + res + 1u);
                doitPlayer.play_file(FAIL_RESP_BANK, file);
            }
            break;
        }

        case F_SEND_FILE_NUM: {
            DEBUG__________ln("Recibido un play sound");
            if (LEF.data.size() >= 2) {
                doitPlayer.play_file(LEF.data[kIdx0], LEF.data[kIdx1]);
            } else {
                DEBUG__________ln("F_SEND_FILE_NUM con datos insuficientes.");
            }
            break;
        }

        case F_SEND_COMMAND: {
            if (LEF.data.empty()) {
                DEBUG__________ln("F_SEND_COMMAND sin datos.");
                break;
            }
            const uint8_t receivedCommand = LEF.data[kIdx0];
            currentCognitiveCommand = receivedCommand;

            DEBUG__________ln("Comando recibido: " + String(receivedCommand, HEX));

            if (receivedCommand == COG_ACT_ON) {
                DEBUG__________ln("Activando modo cognitivo...");
                activateCognitiveMode();

            } else if (receivedCommand == COG_ACT_OFF) {
                DEBUG__________ln("Desactivando modo cognitivo...");
                deactivateCognitiveMode();

            } else if (receivedCommand == WIN_CMD) {
                // Solo aceptar si es para nosotros (DB) o Broadcast
                if (LEF.targetType != DEFAULT_BOTONERA && LEF.targetType != BROADCAST) {
                    #ifdef DEBUG
                    DEBUG__________printf("WIN_CMD ignorado: TargetType=0x%02X (no es DB ni Broadcast)\n", LEF.targetType);
                    #endif
                    break;
                }
                const uint8_t res  = static_cast<uint8_t>(rand() % kWinFailVariants);
                const uint8_t lang = static_cast<uint8_t>(currentLanguage);
                const uint8_t file = static_cast<uint8_t>(lang * kLangStride + res + 1u);
                doitPlayer.play_file(WIN_RESP_BANK, file);

            } else if (receivedCommand == FAIL_CMD) {
                // Solo aceptar si es para nosotros (DB) o Broadcast
                if (LEF.targetType != DEFAULT_BOTONERA && LEF.targetType != BROADCAST) {
                    #ifdef DEBUG
                    DEBUG__________printf("FAIL_CMD ignorado: TargetType=0x%02X\n", LEF.targetType);
                    #endif
                    break;
                }
                const uint8_t res  = static_cast<uint8_t>(rand() % kWinFailVariants);
                const uint8_t lang = static_cast<uint8_t>(currentLanguage);
                const uint8_t file = static_cast<uint8_t>(lang * kLangStride + res + 1u);
                doitPlayer.play_file(FAIL_RESP_BANK, file);

            } else if (receivedCommand == OTA_AP_ON) {
                if (LEF.origin != DEFAULT_BOTONERA) return;
                DEBUG__________ln("Comando OTA_AP_ON recibido  Activando AP + OTA...");
                element->activarAP_OTA();

            } else if (receivedCommand == OTA_AP_OFF) {
                DEBUG__________ln("Comando OTA_AP_OFF recibido  Desactivando AP + OTA...");
                element->desactivarAP_OTA();
            } else if (receivedCommand == BLACKOUT) {
                if (LEF.origin >= 0x90 && LEF.origin <= 0x9F) return; 
                DEBUG__________ln("BLACKOUT recibido → stop_file + limpiar selección + foco Comunicador");

                // 1) Parar cualquier audio en curso
                doitPlayer.stop_file();

                // 2) Deseleccionar todos los elementos (vector global)
                for (size_t i = 0; i < selectedStates.size(); ++i) {
                    selectedStates[i] = false;
                }

                // 3) Mover foco a Comunicador
                enfocarElemento("Comunicador");

                // 4) Redibujar el elemento actual para reflejar el nuevo foco
                drawCurrentElement();

                break; // No procesar más comandos para este frame
            }

            break;
        }

        default: {
        #ifdef DEBUG
            //DEBUG__________ln("Se ha recibido una función desconocida.");
        #endif
            break;
        }
    }

    #ifdef DEBUG
    {
        UBaseType_t stackSizeEnd = uxTaskGetStackHighWaterMark(NULL);
        (void)stackSizeEnd;
    }
    #endif
}

extern bool adxl;
extern bool useMic;

static inline bool isSpiffsPath(const String& s) {
    return s.length() > 0 && s[0] == '/';
}

/**
 * @brief Procesa datos de sector recibidos para un elemento (botonera) y sincroniza estado.
 *
 * Despacha por tipo de sector (nombre, descripción, modo actual, icono, flags, etc.),
 * actualizando almacenamiento SPIFFS, variables internas y visualización (redibujado)
 * cuando corresponde.
 *
 * @param data Trama recibida. Formato: data[0] = código de sector; según sector, se
 *             requiere data[1] (p.ej. modo/flags). Longitud mí­nima: 1 byte.
 * @param targetin Identificador del elemento remoto (ID origen del frame).
 *
 * @return void
 *
 * @pre SPIFFS debe estar montado y accesible. Los offsets (OFFSET_*) deben ser válidos
 *      para la estructura de fichero de los elementos.
 * @pre Debe invocarse en contexto de tarea (no ISR) si posteriormente se redibuja UI.
 *
 * @note En ELEM_CMODE_SECTOR: data[1] = modo actual; se persiste en OFFSET_CURRENTMODE,
 *       se actualizan flags de sensores y patrón de color del elemento actual.
 * @note En ELEM_CURRENT_FLAGS_SECTOR: data[1] usa el bit 0 como estado ON/OFF del relé.
 *
 * @warning Este manejador accede a data[1] en algunos sectores. Si la trama no tiene
 *          al menos 2 bytes, se ignora el sector con traza de depuración.
 *
 * @see drawCurrentElement, RelayStateManager::set, getModeFlag, colorHandler.setPatternBotonera
 */
void BOTONERA_::sectorIn_handler(const std::vector<byte>& data,
                                 const TARGETNS& originNS,
                                 uint8_t originType)
{
    // ----------------------------
    // Validación inicial de trama
    // ----------------------------
    if (data.size() < 1) {
        DEBUG__________ln("Error: sectorIn_handler ha recibido una trama vací­a.");
        return;
    }

    // ----------------------------
    // Constantes locales
    // ----------------------------
    constexpr byte  kDataSector      = 0u; // sector en data[0]
    constexpr byte  kCModeIdx        = 1u; // cmode en data[1]
    constexpr byte  kFlagsIdx        = 1u; // flags en data[1] (para CURRENT_FLAGS)
    constexpr byte  kBit0Mask        = 0x01u;

    // Offset interno dentro del modo para la configuración (no usar '216' mágico)
    constexpr size_t kOffsetConfigInMode = 216u;
    constexpr size_t kModeConfigBytes    = 2u;

    // Helpers locales
    auto nsEquals = [](const TARGETNS& a, const byte b[5]) -> bool {
        return a.mac01 == b[0] && a.mac02 == b[1] && a.mac03 == b[2] && a.mac04 == b[3] && a.mac05 == b[4];
    };

    auto nsEqualsNS = [](const TARGETNS& a, const TARGETNS& b) -> bool {
        return a.mac01 == b.mac01 && a.mac02 == b.mac02 && a.mac03 == b.mac03 && a.mac04 == b.mac04 && a.mac05 == b.mac05;
    };

    auto isZeroNS = [](const TARGETNS& ns) -> bool {
        return ns.mac01==0 && ns.mac02==0 && ns.mac03==0 && ns.mac04==0 && ns.mac05==0;
    };

    // Detectores de elementos en RAM por NS (Ambientes, Fichas, Apagar, Comunicador, Dado)
    auto isRamElementNS = [&](const TARGETNS& ns, INFO_PACK_T** outOpt, String* outName) -> bool {
        // NOTA: asumo que estas estructuras existen y tienen serialNum[5] cargado.
        struct RamOpt { const char* name; INFO_PACK_T* opt; };
        RamOpt opts[] = {
            {"Ambientes",   &ambientesOption},
            {"Fichas",      &fichasOption},
            {"Apagar",      &apagarSala},
            {"Comunicador", &comunicadorOption},
            {"Dado",        &dadoOption}
        };
        for (auto& r : opts) {
            if (nsEquals(ns, r.opt->serialNum)) {
                if (outOpt)  *outOpt  = r.opt;
                if (outName) *outName = r.name;
                return true;
            }
        }
        return false;
    };

    // Localiza un fichero en SPIFFS por NS
    auto getFilePathBySerial = [&](const TARGETNS& ns) -> String {
        for (const String& filePath : elementFiles) {
            if (!isSpiffsPath(filePath)) continue;
            fs::File f = SPIFFS.open(filePath, "r");
            if (!f) continue;
            if (f.size() >= (int)(OFFSET_SERIAL + 5)) {
                byte serial[5] = {0};
                if (f.seek(OFFSET_SERIAL, SeekSet) && f.read(serial, 5) == 5) {
                    f.close();
                    if (nsEquals(ns, serial)) return filePath;
                } else {
                    f.close();
                }
            } else {
                f.close();
            }
        }
        return String(); // no encontrado
    };

    const byte sector = data[kDataSector];   

    switch (sector) {

        case ELEM_ROOM_NS_PACK: {
            DEBUG__________ln("RECIBIDO UN ELEM_ROOM_NS_PACK");
            // Construye payload de 192B  32*(NS[5]+conf[1])
            std::array<uint8_t, 32 * 6> payload{};
            buildRoomNsPack32(payload);

            // Respuesta: dirigimos por NS al que ha hecho la petición.
            // targetType debe ser direccionamiento por NS  DEFAULT_DEVICE (0xDD)
            send_frame(frameMaker_RETURN_ELEM_SECTOR(
                DEFAULT_BOTONERA,                 // originin (esta BOTONERA)
                originType,                       // targetType = 0xDD (por NS)
                originNS,                         // targetNS (del requester)
                payload.data(),                   // 192 bytes
                ELEM_ROOM_NS_PACK                 // sector
            ));
            break;
        }

        case ELEM_NAME_SECTOR: {
            // TODO: Copiar data[1..] a INFO_PACK_T (nombre 32B). Necesita contrato exacto.
            // Sugerencia: validar data.size() >= 1+32 y volcar a option->name o a fichero SPIFFS.
            break;
        }

        case ELEM_DESC_SECTOR: {
            // TODO: Copiar data[1..] a INFO_PACK_T (desc 192B). Necesita contrato exacto.
            break;
        }

        case ELEM_CMODE_SECTOR: {
            if (data.size() < 2) {
                DEBUG__________ln("ELEM_CMODE_SECTOR: payload inválido (size<2).");
                break;
            }
            const byte receivedMode = data[kCModeIdx];

            
            // Fallback: respuesta unicast por ID SIN NS (00:00:00:00:00)
            // durante foco/consulta directa. Usamos pendingQueryIndex.
            
            if (isZeroNS(originNS)) {
                if (pendingQueryIndex >= 0 && pendingQueryIndex < (int)elementFiles.size()) {
                    const String currentFile = elementFiles[pendingQueryIndex];

                    // 1) Elementos en RAM (no tocar SPIFFS)
                    if (currentFile == "Ambientes" || currentFile == "Fichas" ||
                        currentFile == "Apagar"    || currentFile == "Comunicador" ||
                        currentFile == "Dado")
                    {
                        INFO_PACK_T* ramOpt = nullptr;
                        if      (currentFile == "Ambientes")   ramOpt = &ambientesOption;
                        else if (currentFile == "Fichas")      ramOpt = &fichasOption;
                        else if (currentFile == "Apagar")      ramOpt = &apagarSala;
                        else if (currentFile == "Comunicador") ramOpt = &comunicadorOption;
                        else if (currentFile == "Dado")        ramOpt = &dadoOption;

                        if (ramOpt) {
                            ramOpt->currentMode = receivedMode;
                        }
                        if ((size_t)pendingQueryIndex < selectedStates.size()) {
                            selectedStates[pendingQueryIndex] = (receivedMode != 0);
                        }
                        if (pendingQueryIndex == currentIndex) {
                            colorHandler.setCurrentFile(currentFile);
                            colorHandler.setPatternBotonera(receivedMode, ledManager);
                            drawCurrentElement();
                        }
                        break; // Respuesta gestionada en fallback RAM
                    }

                    // 2) Elementos en SPIFFS
                    if (isSpiffsPath(currentFile)) {
                        // Persistir modo
                        {
                            fs::File file = SPIFFS.open(currentFile, "r+");
                            if (file) {
                                byte storedMode = 0;
                                if (file.seek(OFFSET_CURRENTMODE, SeekSet)) {
                                    (void)file.read(&storedMode, 1);
                                }
                                if (storedMode != receivedMode) {
                                    if (file.seek(OFFSET_CURRENTMODE, SeekSet)) {
                                        (void)file.write(&receivedMode, 1);
                                    }
                                }
                                file.close();
                            } else {
                                DEBUG__________ln("No se pudo abrir el archivo SPIFFS (fallback sin NS).");
                            }
                        }

                        // Actualizar selección/UI y flags de sensores del modo
                        if ((size_t)pendingQueryIndex < selectedStates.size()) {
                            selectedStates[pendingQueryIndex] = (receivedMode != 0);
                        }
                        if (pendingQueryIndex == currentIndex) {
                            byte modeConfig[kModeConfigBytes] = {0};
                            fs::File f2 = SPIFFS.open(currentFile, "r");
                            if (f2) {
                                const size_t cfgOffset =
                                    (size_t)OFFSET_MODES +
                                    (size_t)receivedMode * (size_t)SIZE_MODE +
                                    (size_t)kOffsetConfigInMode;
                                if (f2.seek(cfgOffset, SeekSet)) {
                                    (void)f2.read(modeConfig, kModeConfigBytes);
                                } else {
                                    DEBUG__________ln("No se pudo posicionar en OFFSET_CONFIG del modo (fallback).");
                                }
                                f2.close();
                            } else {
                                DEBUG__________ln("No se pudo abrir el archivo SPIFFS para leer config (fallback).");
                            }

                            adxl   = getModeFlag(modeConfig, HAS_SENS_VAL_1);
                            useMic = getModeFlag(modeConfig, HAS_SENS_VAL_2);

                            colorHandler.setCurrentFile(currentFile);
                            colorHandler.setPatternBotonera(receivedMode, ledManager);
                            drawCurrentElement();
                        }
                        break; // Respuesta gestionada en fallback SPIFFS
                    }

                    // Si no es RAM ni SPIFFS (caso raro)
                    DEBUG__________ln("Fallback sin NS: currentFile no es RAM ni SPIFFS conocido.");
                } else {
                    DEBUG__________ln("Respuesta sin NS pero pendingQueryIndex inválido; no se puede aplicar.");
                }
                // Si llegamos aquí­, dejamos continuar a la rama por NS real (por si aplica).
            }

            // 0) Elementos en RAM por NS (no tocar SPIFFS)
            {
                INFO_PACK_T* ramOpt = nullptr;
                String       ramName;
                if (isRamElementNS(originNS, &ramOpt, &ramName)) {
                    // Actualiza RAM
                    ramOpt->currentMode = receivedMode;

                    // Marca seleccionado/no seleccionado en la lista visible si ese nombre existe
                    for (size_t i = 0; i < elementFiles.size(); ++i) {
                        if (elementFiles[i] == ramName) {
                            if (i < selectedStates.size()) {
                                selectedStates[i] = (receivedMode != 0);
                            }
                            if ((int)i == currentIndex) {
                                drawCurrentElement();
                            }
                            break;
                        }
                    }

                    // Si el elemento visible actual es RAM, aplica color/patrón
                    if ((size_t)currentIndex < elementFiles.size() &&
                        (elementFiles[currentIndex] == "Ambientes" ||
                        elementFiles[currentIndex] == "Fichas" ||
                        elementFiles[currentIndex] == "Apagar" ||
                        elementFiles[currentIndex] == "Comunicador" ||
                        elementFiles[currentIndex] == "Dado")) {
                        colorHandler.setCurrentFile(elementFiles[currentIndex]);
                        colorHandler.setPatternBotonera(receivedMode, ledManager);
                        drawCurrentElement();
                    }
                    break; // RAM: no seguimos con SPIFFS
                }
            }
    
            // 1) Elementos de SPIFFS (buscar por NS)  robusto con fallback a pendingQueryNS
            auto isZeroNS = [](const TARGETNS& ns) {
                return ns.mac01==0 && ns.mac02==0 && ns.mac03==0 && ns.mac04==0 && ns.mac05==0;
            };
            const TARGETNS effectiveNS = isZeroNS(originNS) ? pendingQueryNS : originNS;

            const String filePath = getFilePathBySerial(effectiveNS);
            if (!isSpiffsPath(filePath)) {
                DEBUG__________ln("Error: NS no localizado en SPIFFS (getFilePathBySerial).");
                break;
            }


            // 2) Leer/actualizar modo en SPIFFS
            {
                fs::File file = SPIFFS.open(filePath, "r+");
                if (!file) {
                    DEBUG__________ln("Error: No se pudo abrir el archivo en SPIFFS.");
                    break;
                }

                byte storedMode = 0;
                if (!file.seek(OFFSET_CURRENTMODE, SeekSet)) {
                    DEBUG__________ln("No se pudo posicionar en OFFSET_CURRENTMODE.");
                } else {
                    (void)file.read(&storedMode, 1);
                }

                if (storedMode != receivedMode) {
                    DEBUG__________printf("Actualizando el modo en SPIFFS: %d -> %d\n", storedMode, receivedMode);
                    if (file.seek(OFFSET_CURRENTMODE, SeekSet)) {
                        (void)file.write(&receivedMode, 1);
                    } else {
                        DEBUG__________ln("No se pudo reposicionar en OFFSET_CURRENTMODE para escribir.");
                    }
                } else {
                    DEBUG__________ln("El modo recibido coincide con el almacenado en SPIFFS.");
                }
                file.close();
            }

            // 3) Redibujar pantalla
            drawCurrentElement();

            // 4) Actualizar estado de selección por NS (Sí“LO SPIFFS)
            for (size_t i = 0; i < elementFiles.size(); ++i) {
                const String &path = elementFiles[i];
                if (!isSpiffsPath(path)) continue;

                fs::File sf = SPIFFS.open(path, "r");
                if (!sf) continue;

                bool match = false;
                if (sf.size() >= (int)(OFFSET_SERIAL + 5)) {
                    byte serial[5] = {0};
                    if (sf.seek(OFFSET_SERIAL, SeekSet) && sf.read(serial, 5) == 5) {
                        match = nsEquals(originNS, serial);
                    }
                }
                sf.close();

                if (match) {
                    if (i < selectedStates.size()) {
                        selectedStates[i] = (receivedMode != 0);
                        DEBUG__________printf("Estado de selección actualizado: %s => %s\n",
                                            path.c_str(),
                                            selectedStates[i] ? "Seleccionado" : "No seleccionado");
                    } else {
                        DEBUG__________ln("selectedStates desincronizado con elementFiles (í­ndice fuera de rango).");
                    }

                    if ((int)i == currentIndex) {
                        drawCurrentElement();
                    }
                    break;
                }
            }

            // 5) Si el elemento visible es SPIFFS y coincide el NS, recargar flags/color del modo actual
            if ((size_t)currentIndex < elementFiles.size()) {
                const String currentFile = elementFiles[currentIndex];
                if (isSpiffsPath(currentFile)) {
                    //¿El visible actual es el mismo NS?
                    bool sameNS = false;
                    {
                        fs::File f = SPIFFS.open(currentFile, "r");
                        if (f && f.size() >= (int)(OFFSET_SERIAL + 5)) {
                            byte serial[5] = {0};
                            if (f.seek(OFFSET_SERIAL, SeekSet) && f.read(serial, 5) == 5) {
                                sameNS = nsEquals(originNS, serial);
                            }
                        }
                        if (f) f.close();
                    }

                    if (sameNS) {
                        byte currentMode = 0;
                        byte modeConfig[kModeConfigBytes] = {0};

                        fs::File f2 = SPIFFS.open(currentFile, "r");
                        if (f2) {
                            if (f2.seek(OFFSET_CURRENTMODE, SeekSet)) {
                                (void)f2.read(&currentMode, 1);
                            }
                            const size_t cfgOffset =
                                (size_t)OFFSET_MODES +
                                (size_t)currentMode * (size_t)SIZE_MODE +
                                (size_t)kOffsetConfigInMode;
                            if (f2.seek(cfgOffset, SeekSet)) {
                                (void)f2.read(modeConfig, kModeConfigBytes);
                            } else {
                                DEBUG__________ln("No se pudo posicionar en OFFSET_CONFIG del modo.");
                            }
                            f2.close();
                        }

                        adxl   = getModeFlag(modeConfig, HAS_SENS_VAL_1);
                        useMic = getModeFlag(modeConfig, HAS_SENS_VAL_2);

                        colorHandler.setCurrentFile(currentFile);
                        colorHandler.setPatternBotonera(currentMode, ledManager);

                        DEBUG__________ln("Configuración de modo actual actualizada tras recibir ELEM_CMODE_SECTOR.");
                    }
                }
            } else {
                DEBUG__________ln("currentIndex fuera de rango respecto a elementFiles.");
            }

            break;
        }

        case ELEM_ICON_ROW_63_SECTOR: {
            // Reservado: gestión de fila de iconos 63.
            break;
        }

        case ELEM_CURRENT_FLAGS_SECTOR: {
            if (data.size() < 2) {
                DEBUG__________ln("ELEM_CURRENT_FLAGS_SECTOR con longitud insuficiente (<2).");
                break;
            }
            const bool flags_bit0 = (data[1] & 0x01u) != 0;
            // NS-only (sin fallback a ID):
            RelayStateManager::set(originNS, flags_bit0);
            break;
        }

        default: {
            // Sectores no manejados: intencionalmente ignorado.
            (void)originType; // por si lo quieres usar en futuros sectores
            break;
        }
    }
}

bool BOTONERA_::guardar_elemento(INFO_PACK_T* infoPack) {
    // Generamos un nombre de archivo único
    String uniqueFileName = generateUniqueFileName((char*)infoPack->name);
 
    fs::File file = SPIFFS.open(uniqueFileName, "w");
    if (!file) return false;

    bool success = true;
    success &= writeBytesChecked(file, infoPack->name,      sizeof(infoPack->name));
    success &= writeBytesChecked(file, infoPack->desc,      sizeof(infoPack->desc));
    success &= writeBytesChecked(file, infoPack->serialNum, sizeof(infoPack->serialNum));
    success &= writeBytesChecked(file, &infoPack->ID,       1);
    success &= writeBytesChecked(file, &infoPack->currentMode, 1);

    // Guardamos los 16 modos
    for (int i = 0; i < 16 && success; ++i) {
        success &= writeBytesChecked(file, infoPack->mode[i].name,   sizeof(infoPack->mode[i].name));
        success &= writeBytesChecked(file, infoPack->mode[i].desc,   sizeof(infoPack->mode[i].desc));
        success &= writeBytesChecked(file, infoPack->mode[i].config, sizeof(infoPack->mode[i].config));
    }

    // Guardamos icono (64 filas, N columnas * 2 bytes)
    for (int y = 0; y < ICON_ROWS && success; ++y) {
        success &= writeBytesChecked(file, (uint8_t*)infoPack->icono[y], ICON_COLUMNS * 2);
    }

    // Después del icono
    success &= writeBytesChecked(file, &infoPack->situacion, 1);


    file.close();

    if (success) {
        // Recargamos la lista en memoria
        loadElementsFromSPIFFS();
        return true;
    }

    // Si falló algo, borramos el archivo incompleto
    SPIFFS.remove(uniqueFileName);
    return false;
}

bool BOTONERA_::esperar_respuesta(uint8_t expectedSector,
                                  const uint8_t* expectedNS,
                                  std::vector<uint8_t>& outPayload,
                                  unsigned long timeoutMs)
{
    const uint32_t deadline = millis() + timeoutMs;
    outPayload.clear();

    while ((int32_t)(deadline - millis()) > 0) {

        // --- PUMP RX: procesar cualquier frame pendiente aquí­ mismo ---
        // (equivalente a lo que haces en el loop principal)
        while (frameReceived) {
            frameReceived = false;
            LAST_ENTRY_FRAME_T lef = extract_info_from_frameIn(uartBuffer);
            // OJO: llamamos al handler aquí­ mismo.
            // Si scanInProgress==true y la función es F_RETURN_ELEM_SECTOR,
            // nuestro RX_main_handler encolará en rxSectorInbox y hará 'return'.
            this->RX_main_handler(lef);
        }

        // --- Buscar coincidencia en la inbox ---
        size_t hit = SIZE_MAX;
        for (size_t i = 0; i < rxSectorInbox.size(); ++i) {
            const auto& m = rxSectorInbox[i];
            if (m.sector != expectedSector) continue;

            if (!expectedNS) { hit = i; break; } // descubrimiento: no filtramos NS

            bool ok = false;
            if (m.payload.size() >= 5 && nsEquals5(m.payload.data(), expectedNS)) {
                ok = true; // trae NS embebido y coincide
            } else {
                ok = true; // unicast al NS: aceptar por contexto si no trae NS
            }
            if (ok) { hit = i; break; }
        }

        if (hit != SIZE_MAX) {
            outPayload = std::move(rxSectorInbox[hit].payload);
            rxSectorInbox.erase(rxSectorInbox.begin() + hit);
            return true;
        }

        delay(7); // fina
    }

    return false;
}


bool BOTONERA_::serialExistsInSPIFFS(byte serialNum[5]) {
    if (!SPIFFS.begin(true)) {
        return false;
    }
  
    fs::File root = SPIFFS.open("/");
    if (!root) {
        return false;
    }
    if (!root.isDirectory()) {
        root.close();
        return false;
    }
    
    root.rewindDirectory();
    
    // Verificar primero cuántos archivos hay en total
    int totalFiles = 0;
    while (true) {
        fs::File tempFile = root.openNextFile();
        if (!tempFile) break;
        totalFiles++;
        tempFile.close();
    }
    
    root.rewindDirectory();
    
    static INFO_PACK_T tempPack; 
    
    bool found = false;
    int filesChecked = 0;
    
    while (true) {
        fs::File file = root.openNextFile();
        if (!file) {
            break;
        }
        
        filesChecked++;
        String fileName = String(file.name());
        
        // Normalizar el nombre del archivo (añadir '/' al inicio si no está presente)
        if (!fileName.startsWith("/")) {
            fileName = "/" + fileName;
        }
        
        // Solo procesar archivos reales de elementos
        if (fileName.startsWith("/element_") && fileName.endsWith(".bin")) {
            if (file.size() >= MIN_VALID_ELEMENT_SIZE) {
                // Leer el ID primero para verificar
                file.seek(OFFSET_ID);
                byte id;
                int bytesRead = file.read(&id, 1);

                if (bytesRead == 1) {
                    // Ahora leer el serial
                    file.seek(OFFSET_SERIAL);
                    bytesRead = file.read(tempPack.serialNum, 5);

                    if (bytesRead == 5) {
                        // Comparar con el serial buscado
                        if (memcmp(tempPack.serialNum, serialNum, 5) == 0) {
                            found = true;
                            file.close();
                            // No hay delete ni free
                            break;
                        }
                    }
                }
            }
        }
        file.close();
    }
    
    root.close();
    
    return found;
}

void BOTONERA_::dibujarMarco(uint16_t color) {
    tft.drawRoundRect(2, 2, 124, 124, 10, color);
    tft.drawRoundRect(4, 4, 120, 120, 8, color);
}


#define MSG_ERROR_NO_RESPUESTA 0
#define MSG_ADVERTENCIA_ID_DESAJUSTE 1
#define MSG_EXITO_GENERICO 2
#define MSG_NUEVO_ELEMENTO_AGREGADO 3
#define MSG_ERROR_DESCONOCIDO 4

void BOTONERA_::mostrarMensajeTemporal(int respuesta, int dTime) {
    // 1) Limpiar el sprite (fondo negro) y preparar estilos
    uiSprite.fillSprite(TFT_BLACK);

    // Determinar color y mensajes
    uint32_t colorTexto;
    const char* mensajePrincipal;
    const char* mensajeSecundario;

    if (respuesta == 0) { // ERROR GENERAL
        colorTexto = TFT_RED;
        mensajePrincipal = "ERROR";
        mensajeSecundario = "Sin respuesta";
    } else if (respuesta == 1) { // ADVERTENCIA
        colorTexto = TFT_YELLOW;
        mensajePrincipal = "ADVERTENCIA";
        mensajeSecundario = "Verifique el estado del elemento";
    } else if (respuesta == 2) { // ÉXITO / NUEVO
        colorTexto = TFT_GREEN;
        mensajePrincipal  = getTranslation("SUCCESS");
        mensajeSecundario = getTranslation("ROOM_UPDATED");
    } else if (respuesta == 3) { // ERROR ESPECíFICO
        colorTexto = TFT_RED;
        mensajePrincipal = "ERROR";
        mensajeSecundario = "Fallo en la operación";
    } else { // ERROR DESCONOCIDO
        colorTexto = TFT_RED;
        mensajePrincipal = "ERROR";
        mensajeSecundario = "Error desconocido";
    }

    // 2) Mensaje principal centrado
    uiSprite.setTextDatum(MC_DATUM);
    uiSprite.setTextFont(respuesta == 1 ? 2 : 4);    // Advertencia usa fuente máss pequeña
    uiSprite.setTextColor(colorTexto, TFT_BLACK);    // Fondo negro para evitar fantasmas
    uiSprite.drawString(mensajePrincipal, 64, 30);

    // 3) Mensaje secundario (envuelto simple por longitud) centrado
    uiSprite.setTextFont(2);
    uiSprite.setTextColor(TFT_WHITE, TFT_BLACK);

    const int maxCharsPerLine = 18;
    const int lineHeight = 20;
    char buffer[maxCharsPerLine + 1];
    int textLength = strlen(mensajeSecundario);
    int yPos = 60;

    for (int i = 0; i < textLength; i += maxCharsPerLine) {
        strncpy(buffer, mensajeSecundario + i, maxCharsPerLine);
        buffer[maxCharsPerLine] = '\0';
        uiSprite.drawString(buffer, 64, yPos);
        yPos += lineHeight;
    }

    // 4) Icono correspondiente (centrado)
    const int iconCenterX = 64;
    const int iconCenterY = 90;
    const int iconSize    = 10;

    if (respuesta == 0 || respuesta == 3 || (respuesta != 1 && respuesta != 2)) {
        // ERROR (0, 3, default)
        uiSprite.fillCircle(iconCenterX, iconCenterY, iconSize, TFT_RED);
        uiSprite.drawLine(iconCenterX - iconSize/2 + 2, iconCenterY - iconSize/2 + 2,
                          iconCenterX + iconSize/2 - 2, iconCenterY + iconSize/2 - 2, TFT_WHITE);
        uiSprite.drawLine(iconCenterX + iconSize/2 - 2, iconCenterY - iconSize/2 + 2,
                          iconCenterX - iconSize/2 + 2, iconCenterY + iconSize/2 - 2, TFT_WHITE);
    }
    else if (respuesta == 2) {
        // ÉXITO
        uiSprite.fillCircle(iconCenterX, iconCenterY, iconSize, TFT_GREEN);
        uiSprite.drawLine(iconCenterX - iconSize/2 + 2, iconCenterY,
                          iconCenterX - iconSize/2 + 4, iconCenterY + 2, TFT_WHITE);
        uiSprite.drawLine(iconCenterX - iconSize/2 + 4, iconCenterY + 2,
                          iconCenterX + iconSize/2 - 1, iconCenterY - 3, TFT_WHITE);
    }
    else if (respuesta == 1) {
        // ADVERTENCIA (triángulo con exclamación)
        const int triangleSize   = iconSize * 2;
        const int triangleHeight = (int)(triangleSize * 0.866f);

        const int x1 = iconCenterX;
        const int y1 = iconCenterY - (triangleHeight / 2);
        const int x2 = iconCenterX - (triangleSize / 2);
        const int y2 = iconCenterY + (triangleHeight / 2);
        const int x3 = iconCenterX + (triangleSize / 2);
        const int y3 = iconCenterY + (triangleHeight / 2);

        uiSprite.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);

        const int exclamationHeight = (int)(triangleHeight * 0.6f);
        const int barY = iconCenterY - (exclamationHeight / 2) + 1;
        uiSprite.fillRect(iconCenterX - 1, barY, 2, exclamationHeight - 3, TFT_BLACK);    // barra
        uiSprite.fillRect(iconCenterX - 1, barY + (exclamationHeight - 3) + 1, 2, 2, TFT_BLACK); // punto
    }

    // 5) Empujar el sprite a pantalla y mantenerlo visible
    uiSprite.pushSprite(0, 0);
    delay(dTime);
}

bool inCognitiveMenu = false;

void BOTONERA_::activateCognitiveMode()
{
    inCognitiveMenu = true;

    PulsadoresHandler::setCognitiveDefaultButtonMap();

    drawCognitiveMenu();
    colorHandler.mapCognitiveLEDs();
}

void BOTONERA_::deactivateCognitiveMode() {
    inCognitiveMenu = false;
    colorHandler.setCurrentFile("Comunicador");
    colorHandler.setPatternBotonera(0, ledManager);
    drawCurrentElement(); // Volver a la pantalla principal
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BOTONERA_::escanearSala()
{
    // ===== Estado inicial UI =====
    formatSubMenuActive = false;
    hiddenMenuActive    = false;

    // ===== Helpers =====
    auto deleteStoredElements = []() {
        std::vector<String> paths;
        fs::File root = SPIFFS.open("/");
        if (!root || !root.isDirectory()) {
            DEBUG__________ln("No se pudo abrir SPIFFS para limpiar elementos antes del escaneo.");
            return 0u;
        }

        for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
            String fileName = f.name();
            f.close();
            if (!fileName.startsWith("/")) {
                fileName = "/" + fileName;
            }
            if (fileName.startsWith("/element_") && fileName.endsWith(".bin")) {
                paths.push_back(fileName);
            }
        }
        root.close();

        unsigned removed = 0;
        for (const String& path : paths) {
            if (SPIFFS.remove(path)) {
                ++removed;
            } else {
                DEBUG__________printf("No se pudo eliminar %s antes del escaneo.\n", path.c_str());
            }
        }
        return removed;
    };

    auto nsToStr = [](const TARGETNS& ns) {
        char b[16];
        snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X", ns.mac01, ns.mac02, ns.mac03, ns.mac04, ns.mac05);
        return String(b);
    };

    auto serialExists = [&](const TARGETNS& ns) {
        byte raw[5] = { ns.mac01, ns.mac02, ns.mac03, ns.mac04, ns.mac05 };
        return serialExistsInSPIFFS(raw);
    };

    // ===== Parámetros del escaneo =====
    constexpr unsigned long kDiscoveryWindowMs = 30000UL; // 30 s para recoger SERIAL/NS
    constexpr unsigned long kSectorTimeoutMs   = 2000UL;  // t/o por sector
    constexpr int           kRetriesPerSector  = 10;      // reintentos por (NS, sector)
    constexpr unsigned long kInterReqDelayMs   = 35UL;    // pausa entre peticiones

    const unsigned removedBeforeScan = deleteStoredElements();
    loadElementsFromSPIFFS();
    currentIndex = 0;
    relayStep = -1;
    communicatorActiveNS = NS_ZERO;
    RelayStateManager::clear();
    DEBUG__________printf("Elementos almacenados eliminados antes del escaneo: %u\n", removedBeforeScan);

    // ===== Activar escaneo (inbox ON en RX_main_handler) =====
    rxSectorInbox.clear();
    scanInProgress = true;

    // ===== Fase 1: Descubrimiento por broadcast (Sleep+Wake) =====
    DEBUG__________ln("===  ESCANEO DE SALA (NS) Descubrimiento ===");
    tft.fillScreen(TFT_BLACK);
    int frameCountAnim = 0;
    drawLoadingModalFrame(getTranslation("SEARCHING"), frameCountAnim);

    const TARGETNS ZERO_NS = {0,0,0,0,0};
    send_frame(frameMaker_SEND_COMMAND(
        DEFAULT_BOTONERA,          // origin (0xDB)
        BROADCAST,                 // targetType (0xFF)
        ZERO_NS,                   // targetNS = 00:00:00:00:00
        SLEEP_SERIAL_WAKEUP_CMD    // comando de descubrimiento
    ));

    std::vector<TARGETNS> discovered;
    const uint32_t tEnd = millis() + kDiscoveryWindowMs;
    uint32_t lastAnim = millis();

    while ((int32_t)(tEnd - millis()) > 0) {
        if (millis() - lastAnim >= 33) {
            uint32_t s = (millis() - (tEnd - kDiscoveryWindowMs)) / 1000U;
            if (s > (kDiscoveryWindowMs/1000U)) s = (kDiscoveryWindowMs/1000U);
            actualizarBarraProgreso2((int)s, (int)(kDiscoveryWindowMs/1000U), getTranslation("SEARCHING"));
            lastAnim = millis();
        }

        // Leemos exclusivamente de la INBOX los sectores 0x03 (Serial/NS), de cualquiera
        std::vector<uint8_t> payload;
        if (esperar_respuesta(ELEM_SERIAL_SECTOR, /*any*/ nullptr, payload, 60)) {
            if (payload.size() >= 5) {
                TARGETNS ns{ payload[0], payload[1], payload[2], payload[3], payload[4] };
                bool dup = false;
                for (auto& d : discovered) {
                    if (d.mac01==ns.mac01 && d.mac02==ns.mac02 && d.mac03==ns.mac03 &&
                        d.mac04==ns.mac04 && d.mac05==ns.mac05) { dup = true; break; }
                }
                if (!dup) {
                    DEBUG__________printf("Descubierto NS=%s\n", nsToStr(ns).c_str());
                    discovered.push_back(ns);
                }
            }
        } else {
            delay(5);
        }
    }

    DEBUG__________printf("Fin descubrimiento: %u dispositivos.\n", (unsigned)discovered.size());

    // ===== Fase 2: Descarga por NS (unicast) =====
    bool huboAltas = false;

    // Lista de sectores a pedir en orden lógico completo (SIN ID)
    std::vector<int> sectores;
    sectores.reserve(3 + 16*3 + ICON_ROWS);

    sectores.push_back(ELEM_NAME_SECTOR);
    sectores.push_back(ELEM_DESC_SECTOR);
    sectores.push_back(ELEM_CMODE_SECTOR);

    // Añadir los 16 modos: NAME, DESC, FLAG (i = 0..15)
    for (int i = 0; i < 16; ++i) {
        sectores.push_back(ELEM_MODE_0_NAME_SECTOR + i * 3);
        sectores.push_back(ELEM_MODE_0_DESC_SECTOR + i * 3);
        sectores.push_back(ELEM_MODE_0_FLAG_SECTOR + i * 3);
    }

    // Por último, las 64 filas del icono
    for (int r = 0; r < ICON_ROWS; ++r) {
        sectores.push_back(ELEM_ICON_ROW_0_SECTOR + r);
    }

    drawLoadingModalFrame(getTranslation("UPDATING"), frameCountAnim);

    // === Buffer grande fuera del stack (evita stack overflow) ===
    static INFO_PACK_T info;  // único y reutilizable

    for (const auto& ns : discovered) {
        if (serialExists(ns)) {
            DEBUG__________printf("Ya existe en SPIFFS: %s (omitido)\n", nsToStr(ns).c_str());
            continue;
        }

        // Reinciamos el buffer
        memset(&info, 0, sizeof(info));
        // SIN ID: no tocar info.ID
        info.serialNum[0]=ns.mac01; info.serialNum[1]=ns.mac02; info.serialNum[2]=ns.mac03;
        info.serialNum[3]=ns.mac04; info.serialNum[4]=ns.mac05;

        bool fallos = false;
        uint32_t lastAnim2 = millis();

        for (size_t i=0; i<sectores.size(); ++i) {
            const int sector = sectores[i];

            if (millis() - lastAnim2 >= 33) {
                actualizarBarraProgreso2((int)(i+1), (int)sectores.size(), getTranslation("UPDATING"));
                lastAnim2 = millis();
            }

            bool ok = false;
            for (int attempt = 0; attempt <= kRetriesPerSector && !ok; ++attempt) {

                // Unicast por NS firma EXACTA que usas:
                send_frame(frameMaker_REQ_ELEM_SECTOR(
                    DEFAULT_BOTONERA,            // origin (0xDB)
                    DEFAULT_DEVICE,              // targetType (0xDD  elemento)
                    ns,                          // TARGETNS (MAC 5 bytes)
                    (byte)currentLanguage,       // idioma
                    (byte)sector                 // sector
                ));

                std::vector<uint8_t> payload;
                const uint8_t* nsPtr = reinterpret_cast<const uint8_t*>(&ns);
                if (esperar_respuesta((uint8_t)sector, nsPtr, payload, kSectorTimeoutMs)) {
                    if (!procesar_sector_NS(sector, &info, ns, payload.data(), payload.size())) {
                        DEBUG__________printf("Sector 0x%02X recibido pero no procesado (NS %s)\n",
                                              sector, nsToStr(ns).c_str());
                    }
                    ok = true;
                } else {
                    DEBUG__________printf("Timeout 0x%02X (NS %s) reint=%d\n",
                                          sector, nsToStr(ns).c_str(), attempt+1);
                }
                delay(kInterReqDelayMs);
            }

            if (!ok) { fallos = true; break; }
        }

        if (fallos) {
            DEBUG__________printf("Error descargando sectores para NS %s (se omite alta)\n", nsToStr(ns).c_str());
            continue;
        }

        // SIN ID: guardar_elemento no debe persistir ningún campo de ID
        if (guardar_elemento(&info)) {
            DEBUG__________printf(" Guardado en SPIFFS (NS %s)\n", nsToStr(ns).c_str());
            huboAltas = true;
        } else {
            DEBUG__________printf("Error guardando (NS %s)\n", nsToStr(ns).c_str());
        }
    }

    scanInProgress = false; // <- MUY importante: volver a flujo normal

    if (huboAltas) {
        loadElementsFromSPIFFS();
        if ((size_t)currentIndex < elementFiles.size()) {
            const String currentFile = elementFiles[currentIndex];
            uint8_t currentMode = 0;

            if (isSpiffsPath(currentFile)) {
                fs::File f = SPIFFS.open(currentFile, "r");
                if (f) {
                    if (f.seek(OFFSET_CURRENTMODE, SeekSet)) {
                        (void)f.read(&currentMode, 1);
                    }
                    f.close();
                }
            } else if (currentFile == "Ambientes") {
                currentMode = ambientesOption.currentMode;
            } else if (currentFile == "Fichas") {
                currentMode = fichasOption.currentMode;
            } else if (currentFile == "Apagar") {
                currentMode = apagarSala.currentMode;
            } else if (currentFile == "Comunicador") {
                currentMode = comunicadorOption.currentMode;
            } else if (currentFile == "Dado") {
                currentMode = dadoOption.currentMode;
            }

            colorHandler.setCurrentFile(currentFile);
            colorHandler.setPatternBotonera(currentMode, ledManager);
            drawCurrentElement();
        }
        DEBUG__________ln("SPIFFS recargado tras altas.");
    }

    mostrarMensajeTemporal(2, 3000);
    DEBUG__________ln("=== FIN ESCANEO DE SALA (NS) ===");
}

// Función para mostrar texto multilí­nea ajustado al ancho máximo sin romper palabras
// void BOTONERA_::iniciarEscaneoElemento(const char* mensajeInicial) {
//     tft.fillScreen(TFT_BLACK);      // limpia TODO (solo aquí­)
//     //dibujarMarco(TFT_WHITE);        // marco fijo

//     // muestra MULTILíNEA (nombre + ID)
//     mostrarTextoAjustado(tft,
//                          mensajeInicial,
//                          64,   // centro X
//                          30,   // Y inicial
//                          120); // ancho máximo de texto
//     delay(100);
// }


void BOTONERA_::actualizarBarraProgreso2(int pasoActual,
                                         int pasosTotales,
                                         const char* etiqueta)
{
    const int W = tft.width();    // 128
    const int H = tft.height();   // 128

    // írea de contenido (antes card); ya SIN marco
    const int cardW = 112;
    const int cardH = 96;
    const int cardX = (W - cardW) / 2; // 8
    const int cardY = (H - cardH) / 2; // 16

    // Cabecera
    const int lineHeight = 20;
    const int textX = cardX + 12;
    const int textY = cardY + 14;

    uiSprite.setTextDatum(TL_DATUM);
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextSize(1);

    // uiSprite.setTextDatum(TL_DATUM);
    // uiSprite.setFreeFont(nullptr);
    // uiSprite.setTextFont(1);  // 6—8 pixeles
    // uiSprite.setTextSize(1);


    
    const int contentX = cardX + 12;
    const int contentW = cardW - 24;

    // Barra centrada verticalmente y MÁS ancha/gruesa
    const int barH = 14;                                // grosor (ajustable)
    const int barX = contentX;                          // ocupa todo el ancho útil
    const int barW = contentW;
    const int barY = cardY + (cardH/2) - (barH/2);      // centrada vertical

    // Porcentaje debajo y centrado respecto a la barra
    const int pctYOffset = 10;                          // separación (ajustable)
    const int pctY = barY + barH + pctYOffset;

    static bool     s_init=false;
    static uint32_t s_lastMs=0;
    static float    s_progSmooth=0.0f;
    static int      s_frameCount=0;

    if (!s_init) { s_init=true; s_lastMs=millis(); s_frameCount=0; }

    const uint32_t now = millis();
    float dt = (now - s_lastMs)/1000.0f; if (dt < 0) dt = 0;
    s_lastMs = now; s_frameCount++;

    float progTarget = 0.0f;
    if (pasosTotales > 0) {
        int pasosClamped = max(0, pasoActual - 1);
        pasosClamped = min(pasosClamped, pasosTotales);
        progTarget = (float)pasosClamped / (float)pasosTotales;
    }
    const float k = 8.0f;
    s_progSmooth += (progTarget - s_progSmooth) * min(1.0f, k*dt);
    s_progSmooth = constrain(s_progSmooth, 0.0f, 1.0f);
    const int progW = (int)(barW * s_progSmooth + 0.5f);

    // ===== DIBUJO =====
    uiSprite.fillSprite(BACKGROUND_COLOR);

    // Cabecera (opcional)
    if (etiqueta) {
    const int headerMaxW = cardW - 16; // margen lateral de 12 px por cada lado

    uiSprite.setTextDatum(TL_DATUM);

    // 1) Fuente normal grande
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextSize(1);

    int anchoTexto = uiSprite.textWidth(etiqueta);

    if (anchoTexto > headerMaxW) {
        // 2) Cambiar a fuente más pequeña si no cabe
        uiSprite.setFreeFont(nullptr); // salir de FreeFont
        uiSprite.setTextFont(2);       // fuente integrada 16 px alto
        uiSprite.setTextSize(1);

        //opcional: volver a medir para comprobar que ahora sí­ cabe
    }

    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.drawString(etiqueta, textX, textY);
    }
    // Separador sutil bajo cabecera (opcional, se puede retirar)
    const int sepY = textY + lineHeight + 2;
    if (sepY < barY - 14) uiSprite.drawFastHLine(cardX + 10, sepY, cardW - 20, TEXT_COLOR);

    // Pista y progreso (barber-pole)
    uiSprite.drawRoundRect(barX - 1, barY - 1, barW + 2, barH + 2, 4, TEXT_COLOR);
    if (progW > 0) {
        uiSprite.fillRoundRect(barX, barY, progW, barH, 4, TFT_CYAN);
        const int stripeStep = 6;
        const int offset = s_frameCount % stripeStep;
        for (int sx = -barH + offset; sx < progW; sx += stripeStep) {
            int x1 = barX + sx,       y1 = barY;
            int x2 = barX + sx + barH, y2 = barY + barH - 1;
            if (x2 > barX + progW) { int dx = x2 - (barX + progW); x2 -= dx; y2 -= dx; }
            if (x1 < barX)         { int dx = barX - x1;          x1 += dx; y1 += dx; }
            if (x1 <= x2) uiSprite.drawLine(x1, y1, x2, y2, BACKGROUND_COLOR);
        }
    }

    // Porcentaje centrado debajo
    char pctStr[8];
    int percent = (int)(s_progSmooth * 100.0f + 0.5f);
    snprintf(pctStr, sizeof(pctStr), "%d%%", percent);
    int pctW = uiSprite.textWidth(pctStr);
    int pctX = barX + (barW/2) - (pctW/2);
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.drawString(pctStr, pctX, pctY);

    uiSprite.pushSprite(0, 0);
}

// Busca el fichero de SPIFFS asociado a un elemento por su NS (5 bytes)

// String BOTONERA_::getFilePathBySerial(const TARGETNS& ns) {
//     // Recorremos la lista ya cargada por loadElementsFromSPIFFS()
//     for (const String& filePath : elementFiles) {
//         fs::File f = SPIFFS.open(filePath, "r");
//         if (!f) continue;

//         // Verificación rápida de tamaño (usa tu MIN_VALID_ELEMENT_SIZE o uno ajustado)
//         if (f.size() < (size_t)(OFFSET_SERIAL + 5)) { f.close(); continue; }

//         byte serial[5];
//         if (f.seek(OFFSET_SERIAL, SeekSet) && f.read(serial, 5) == 5) {
//             if (memcmp(serial, &ns, 5) == 0) {
//                 f.close();
//                 return filePath; //¡Encontrado!
//             }
//         }
//         f.close();
//     }
//     return String(); // no encontrado
// }

bool BOTONERA_::procesar_sector_NS(int sector,
                                   INFO_PACK_T* info,
                                   const TARGETNS& ns,
                                   const uint8_t* data,
                                   size_t len)
{
    if (!info || !data) return false;

    const uint8_t* p = data;
    size_t plen = len;

    // Nunca pelar en SERIAL (0x03). En el resto, si empiezan por el NS objetivo, pelar 5 bytes.
    if (sector != ELEM_SERIAL_SECTOR && plen >= 5) {
        const uint8_t nsArr[5] = { ns.mac01, ns.mac02, ns.mac03, ns.mac04, ns.mac05 };
        if (nsEquals5(p, nsArr)) { p += 5; plen -= 5; }
    }

    auto copyFixed = [](uint8_t* dst, size_t dstLen, const uint8_t* src, size_t srcLen) {
        const size_t n = (srcLen < dstLen) ? srcLen : dstLen;
        if (n) memcpy(dst, src, n);
        if (dstLen > n) memset(dst + n, 0, dstLen - n);
    };

    // Rangos de modos (siguen tus defines)
    const int MODE_NAME_FIRST = ELEM_MODE_0_NAME_SECTOR;
    const int MODE_NAME_LAST  = ELEM_MODE_15_NAME_SECTOR;
    const int MODE_DESC_FIRST = ELEM_MODE_0_DESC_SECTOR;
    const int MODE_DESC_LAST  = ELEM_MODE_15_DESC_SECTOR;
    const int MODE_FLAG_FIRST = ELEM_MODE_0_FLAG_SECTOR;
    const int MODE_FLAG_LAST  = ELEM_MODE_15_FLAG_SECTOR;

    auto isModeName = [&](int s){ return (s >= MODE_NAME_FIRST && s <= MODE_NAME_LAST) && ((s - MODE_NAME_FIRST) % 3 == 0); };
    auto isModeDesc = [&](int s){ return (s >= MODE_DESC_FIRST && s <= MODE_DESC_LAST) && ((s - MODE_DESC_FIRST) % 3 == 0); };
    auto isModeFlag = [&](int s){ return (s >= MODE_FLAG_FIRST && s <= MODE_FLAG_LAST) && ((s - MODE_FLAG_FIRST) % 3 == 0); };

    auto idxName = [&](int s){ return (s - MODE_NAME_FIRST) / 3; };
    auto idxDesc = [&](int s){ return (s - MODE_DESC_FIRST) / 3; };
    auto idxFlag = [&](int s){ return (s - MODE_FLAG_FIRST) / 3; };

    switch (sector) {
        case ELEM_NAME_SECTOR:
            copyFixed((uint8_t*)info->name, sizeof(info->name), p, plen);
            return true;

        case ELEM_DESC_SECTOR:
            copyFixed((uint8_t*)info->desc, sizeof(info->desc), p, plen);
            return true;

        case ELEM_LOCATION_SECTOR:
            if (plen < 1) return false;
            info->situacion = p[0];
            return true;

        case ELEM_CMODE_SECTOR:
            if (plen < 1) return false;
            info->currentMode = DEFAULT_BASIC_MODE;
            return true;

        case ELEM_SERIAL_SECTOR:
            if (plen < 5) return false;
            info->serialNum[0] = p[0];
            info->serialNum[1] = p[1];
            info->serialNum[2] = p[2];
            info->serialNum[3] = p[3];
            info->serialNum[4] = p[4];
            return true;

        case ELEM_UNUSED_SECTOR:
            
            return true;

        default:
            if (isModeName(sector)) {
                int idx = idxName(sector);
                if (idx < 0 || idx >= 16) return false;
                copyFixed((uint8_t*)info->mode[idx].name, sizeof(info->mode[idx].name), p, plen);
                return true;
            }
            if (isModeDesc(sector)) {
                int idx = idxDesc(sector);
                if (idx < 0 || idx >= 16) return false;
                copyFixed((uint8_t*)info->mode[idx].desc, sizeof(info->mode[idx].desc), p, plen);
                return true;
            }
            if (isModeFlag(sector)) {
                int idx = idxFlag(sector);
                if (idx < 0 || idx >= 16 || plen < 2) return false;
                info->mode[idx].config[0] = p[0];
                info->mode[idx].config[1] = p[1];
                return true;
            }
            if (sector >= ELEM_ICON_ROW_0_SECTOR && sector <= ELEM_ICON_ROW_63_SECTOR) {
                const int row = sector - ELEM_ICON_ROW_0_SECTOR;
                if (row < 0 || row >= ICON_ROWS) return false;
                const size_t expected = ICON_COLUMNS * 2; // 128 bytes
                if (plen < expected) return false;
                for (int c = 0; c < ICON_COLUMNS; ++c) {
                    const uint8_t msb = p[2*c + 0];
                    const uint8_t lsb = p[2*c + 1];
                    info->icono[row][c] = (uint16_t(msb) << 8) | lsb;
                }
                return true;
            }
            return true; // no crí­tico
    }
}

