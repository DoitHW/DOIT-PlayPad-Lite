#include <encoder_handler/encoder_handler.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <display_handler/display_handler.h>
#include "icons_64x64_DMS/icons_64x64_DMS.h"
#include <Translations_handler/translations.h>
#include <play_DMS/play_DMS.h>

uint16_t lineBuffer[64];   // definición real
INFO_PACK_T dadoOption;
static const char* EXTRA_CFG_PATH = "/config_extra.bin";


bool writeBytesChecked(fs::File &f, const uint8_t* data, size_t length) {
    size_t written = f.write(data, length);
    if(written != length) {
        #ifdef DEBUG
          DEBUG__________printf("Error: se esperaban escribir %u bytes, se escribieron %u\n",(unsigned)length,(unsigned)written);                                                                        
        #endif
        return false;
    }
    return true;
}

void formatSPIFFS() {
    #ifdef DEBUG
      DEBUG__________("Formateando SPIFFS...");                                                                            
    #endif
    
    SPIFFS.end();
    if (!SPIFFS.format()) {
        #ifdef DEBUG
          DEBUG__________ln("Error al formatear SPIFFS.");                                                                        
        #endif
        
    }
    if (!SPIFFS.begin(true)) {
        #ifdef DEBUG
          DEBUG__________ln("Error al montar SPIFFS después del formateo.");                                                                        
        #endif
        
    }
}


String generateUniqueFileName(const char* baseName) {
    String baseFileName = String("/element_") + baseName + ".bin";
    String fileName = baseFileName;
    int index = 2;
    while (SPIFFS.exists(fileName)) {
        fileName = String("/element_") + baseName + "_" + String(index) + ".bin";
        index++;
    }
    return fileName;
}


// bool readElementData(fs::File& f, char* elementName, char* modeName, int& startX, int& startY) {
//     // Leer nombre del elemento
//     f.seek(OFFSET_NAME, SeekSet);
//     f.read((uint8_t*)elementName, 24);
//     elementName[23] = 0;

//     // Leer nombre del modo actual
//     byte currentMode;
//     f.seek(OFFSET_CURRENTMODE, SeekSet);
//     f.read(&currentMode, 1);

//     f.seek(OFFSET_MODES + (SIZE_MODE * currentMode), SeekSet);
//     f.read((uint8_t*)modeName, 24);
//     modeName[23] = 0;

//     // Calcular posiciones del icono
//     startX = (tft.width() - 64) / 2;
//     startY = (tft.height() - 64) / 2 - 20;

//     // Leer situación
//     f.seek(OFFSET_SITUACION, SeekSet);
//     byte situacionByte = 0;
//     f.read(&situacionByte, 1);
//     // Puedes usarlo si quieres mostrarlo o pasarlo por referencia


//     return true;
// }


bool readElementData(fs::File& f, char* elementName, char* modeName, int& startX, int& startY) {
    // Leer nombre del elemento
    f.seek(OFFSET_NAME, SeekSet);
    f.read((uint8_t*)elementName, 24);
    elementName[23] = 0;

    // Leer nombre del modo actual
    byte currentMode = 0;
    f.seek(OFFSET_CURRENTMODE, SeekSet);
    if (f.read(&currentMode, 1) != 1) {
        currentMode = 0; // fallback defensivo
    }

    constexpr byte MAX_KNOWN_CMODE = 8;

    if (currentMode > MAX_KNOWN_CMODE) {
        // CMODE > 8 → mostrar texto genérico
        strncpy(modeName, "DESCONOCIDO", 24);
        modeName[23] = 0;
    } else {
        // CMODE válido → leer nombre desde la tabla de modos
        f.seek(OFFSET_MODES + (SIZE_MODE * currentMode), SeekSet);
        f.read((uint8_t*)modeName, 24);
        modeName[23] = 0;
    }

    // Calcular posiciones del icono
    startX = (tft.width() - 64) / 2;
    startY = (tft.height() - 64) / 2 - 20;

    // Leer situación
    f.seek(OFFSET_SITUACION, SeekSet);
    byte situacionByte = 0;
    f.read(&situacionByte, 1);
    // Puedes usarlo si quieres mostrarlo o pasarlo por referencia

    return true;
}


void initializeDynamicOptions() {
    // ----- Ambientes -----
    memset(&ambientesOption, 0, sizeof(INFO_PACK_T));
    strncpy((char*)ambientesOption.name, "AMBIENTES", 24);
    strncpy((char*)ambientesOption.desc, "Configura los ambientes de la sala.", 192);
    ambientesOption.currentMode = 0;
    //strncpy((char*)ambientesOption.mode[0].name, "BASICO", 24);
    //strncpy((char*)ambientesOption.mode[0].desc, "Modo basico para ambientes.", 192);
    ambientesOption.mode[0].config[0] = 0x00;
    ambientesOption.mode[0].config[1] = 0x09;
    memcpy(ambientesOption.icono, ambientes_64x64, sizeof(ambientesOption.icono));
    ambientesOption.situacion = 0;

    // ----- Fichas -----
    memset(&fichasOption, 0, sizeof(INFO_PACK_T));
    strncpy((char*)fichasOption.name, getTranslation("FICHAS"), 24);
    strncpy((char*)fichasOption.desc, "Interacción con fichas NFC.", 192);
    fichasOption.currentMode = 0;
    strncpy((char*)fichasOption.mode[0].name, "BASICO", 24);
    strncpy((char*)fichasOption.mode[0].desc, "Modo basico para fichas NFC.", 192);
    fichasOption.mode[0].config[0] = 0x80;
    fichasOption.mode[0].config[1] = 0x08;
    strncpy((char*)fichasOption.mode[1].name, "PAREJAS", 24);
    strncpy((char*)fichasOption.mode[1].desc, "Modo parejas para fichas NFC.", 192);
    fichasOption.mode[1].config[0] = 0x80;
    fichasOption.mode[1].config[1] = 0x08;
    strncpy((char*)fichasOption.mode[2].name, "ADIVINAR", 24);
    strncpy((char*)fichasOption.mode[2].desc, "Modo adivinar para fichas NFC.", 192);
    fichasOption.mode[2].config[0] = 0x80;
    fichasOption.mode[2].config[1] = 0x08;

    memcpy(fichasOption.icono, fichas_64x64, sizeof(fichasOption.icono));
    //DEBUG__________ln("Nombre de fichasOption.name después de crear el icono: " + String((char*)fichasOption.name));
    fichasOption.situacion = 0;

    // ----- ApagarSala -----
    memset(&apagarSala, 0, sizeof(INFO_PACK_T));
    strncpy((char*)apagarSala.name, getTranslation("APAGAR"), 24);
    strncpy((char*)apagarSala.desc, "Apaga la sala por completo.", 192);
    apagarSala.mode[0].config[0] = 0x00;
    apagarSala.mode[0].config[1] = 0x00;
    apagarSala.currentMode = 0;
    // Puedes añadir uno o varios modos, aquí un ejemplo con un solo modo "OFF".
    // strncpy((char*)apagarSala.mode[0].name, "OFF", 24);
    // strncpy((char*)apagarSala.mode[0].desc, "Modo para apagar la sala.", 192);
    // apagarSala.mode[0].config[0] = 0x7F; // El valor que tú desees para este modo
    // apagarSala.mode[0].config[1] = 0xFF;
    apagarSala.situacion = 0;
    // Asumiendo que tienes un icono llamado apagar_64x64:
    memcpy(apagarSala.icono, apagar_sala_64x64, sizeof(apagarSala.icono));


    memset(&comunicadorOption, 0, sizeof(INFO_PACK_T));
    strncpy((char*)comunicadorOption.name, getTranslation("COMUNICADOR"), 24);
    comunicadorOption.currentMode = 0;
    strncpy((char*)comunicadorOption.mode[0].name, "BASICO", 24);
    comunicadorOption.mode[0].config[0] = 0x80;   // ejemplo de flag “visible”
    comunicadorOption.mode[0].config[1] = 0x09;
    comunicadorOption.ID = 0xDB;
    // Copiar aquí el icono de 64×64:
    memcpy(comunicadorOption.icono, comunicador_64x64, sizeof(comunicadorOption.icono));

    // -- DADO -- 
    memset(&dadoOption, 0, sizeof(INFO_PACK_T));
    strncpy((char*)dadoOption.name, "DADO", 24);
    dadoOption.ID = DEFAULT_DICE;

    dadoOption.mode[0].config[0] = 0x00;
    dadoOption.mode[0].config[1] = 0x00;

    // Modo 0: BASICO
    strncpy((char*)dadoOption.mode[1].name, "BASICO", 24);
    dadoOption.mode[1].config[0] = 0x80; // MSB=1 => EXIST/visible
    dadoOption.mode[1].config[1] = 0x00; // sin más flags

    // Modo 1: REBOTES
    strncpy((char*)dadoOption.mode[2].name, "AVANZADO", 24);
    dadoOption.mode[2].config[0] = 0x80;
    dadoOption.mode[2].config[1] = 0x00;

    dadoOption.currentMode = 0;

    // Icono
    memcpy(dadoOption.icono, dado_64x64, sizeof(dadoOption.icono));

}


void loadElementsFromSPIFFS() {
    elementFiles.clear();
    selectedStates.clear();

    // Cargar elementos desde SPIFFS
    fs::File root = SPIFFS.open("/");
    if (!root || !root.isDirectory()) {
                                                                                                                #ifdef DEBUG
                                                                                                                DEBUG__________ln("Error: No se pudo abrir el directorio raíz de SPIFFS.");                                                                           
                                                                                                                #endif
        
        return;
    }

    fs::File file = root.openNextFile();
    while (file) {
        String fileName = file.name();
        if (!fileName.startsWith("/")) {
            fileName = "/" + fileName;
        }
        if (fileName.startsWith("/element_") && fileName.endsWith(".bin") && fileName.indexOf("_icon") < 0) {
            elementFiles.push_back(fileName);
            selectedStates.push_back(false); // Inicialmente no seleccionado
        }
        file.close();
        file = root.openNextFile();
    }
                                                                                    #ifdef DEBUG
                                                                                    DEBUG__________ln("Elementos encontrados:");
                                                                                    #endif
    
     for (size_t i = 0; i < elementFiles.size(); i++) {
         DEBUG__________printf("%d: %s\n", (int)i, elementFiles[i].c_str());
     }
                                                                                    #ifdef DEBUG
                                                                                    DEBUG__________printf("Total de elementos: %d\n", (int)elementFiles.size());                                                                                
                                                                                    #endif
     // Agregar las opciones dinámicas
     elementFiles.push_back("Ambientes");
     selectedStates.push_back(false);
 
     elementFiles.push_back("Fichas");
     selectedStates.push_back(false);
 
     elementFiles.push_back("Apagar");
     selectedStates.push_back(false);

     elementFiles.push_back("Comunicador");
     selectedStates.push_back(false);

     if (isDadoEnabled()) {
     elementFiles.push_back("Dado");
     selectedStates.push_back(false);
     }


}

TARGETNS getCurrentElementNS() {
    // Valor por defecto para “no aplica” o “sin NS”
    TARGETNS ns = NS_ZERO;

    if (elementFiles.empty() || currentIndex < 0 ||
        (size_t)currentIndex >= elementFiles.size()) {
        return ns;
    }

    const String& currentFile = elementFiles[currentIndex];

    // Elementos “en RAM” o UI-puros → no tienen NS: devuelven NS_ZERO
    // - Ambientes / Fichas / Apagar (UI/acciones globales)
    // - Comunicador (targetType 0xDB, NS = 0)
    // - Dado        (targetType 0xDA, NS = 0)
    if (currentFile == "Ambientes"   ||
        currentFile == "Fichas"      ||
        currentFile == "Apagar"      ||
        currentFile == "Comunicador" ||
        currentFile == "Dado") {
        return NS_ZERO;
    }

    // Resto de elementos “reales” en SPIFFS → leer su NS del archivo
    return getNSFromFile(currentFile);
}

bool isCurrentElementSelected() {
    return selectedStates[currentIndex];  // Devuelve true si está seleccionado
}

bool checkMostSignificantBit(byte modeConfig[2]) {
    // Devuelve true si el bit más significativo del primer byte es 1, de lo contrario false
    //DEBUG__________ln("Devolviendo el bit más significativo del modo. ");
    return (modeConfig[0] & 0x80) != 0;
}

bool getModeConfig(const String& fileName, byte mode, byte modeConfig[2]) {
    memset(modeConfig, 0, 2); // Inicializar en 0

    INFO_PACK_T* option = nullptr;

    if      (fileName == "Ambientes")   option = &ambientesOption;
    else if (fileName == "Fichas")      option = &fichasOption;
    else if (fileName == "Apagar")      option = &apagarSala;
    else if (fileName == "Comunicador") option = &comunicadorOption; 
    else if (fileName == "Dado")        option = &dadoOption;

    if (option != nullptr) {
        memcpy(modeConfig, option->mode[mode].config, 2);
        return true;
    }

    // Si no es un elemento fijo, buscar en SPIFFS
    fs::File f = SPIFFS.open(fileName, "r");
    if (!f) {
        #ifdef DEBUG
        DEBUG__________ln("❌ Error abriendo el archivo: " + fileName);
        #endif
        return false;
    }

    f.seek(OFFSET_MODES + (SIZE_MODE * mode) + 24 + 192, SeekSet); // OFFSET_CONFIG
    if (f.read(modeConfig, 2) != 2) {
        #ifdef DEBUG
        DEBUG__________ln("❌ Error leyendo configuración del modo");
        #endif
        f.close();
        return false;
    }

    f.close();
    return true;
}


void setAllElementsToBasicMode() {

    for (size_t i = 0; i < elementFiles.size(); i++) {
        const String& fileName = elementFiles[i];

        // Solo ficheros de elementos reales en SPIFFS:
        if (!fileName.startsWith("/element_")) continue;
        if (!fileName.endsWith(".bin"))        continue;

        fs::File f = SPIFFS.open(fileName, "r+");
        if (!f) {
            #ifdef DEBUG
            DEBUG__________printf("❌ Error abriendo %s para escritura.\n", fileName.c_str());
            #endif
            continue;
        }

        byte basicMode = DEFAULT_BASIC_MODE;
        f.seek(OFFSET_CURRENTMODE, SeekSet);
        f.write(&basicMode, 1);
        f.flush();
        f.close();
    }
}

void updateBankList(byte bank) {
    const char* bankFilePath = "/bank_list.bin";
    std::vector<byte> bankList;

    // Si el archivo existe, lo abrimos en modo lectura para recuperar la lista actual
    if (SPIFFS.exists(bankFilePath)) {
        File f = SPIFFS.open(bankFilePath, "r");
        if (f) {
            while (f.available()) {
                bankList.push_back(f.read());
            }
            f.close();
        } else {
            DEBUG__________ln("Error al abrir el archivo de banks para lectura.");
        }
    }

    // Verificar si el bank ya se encuentra en la lista
    bool exists = false;
    for (byte b : bankList) {
        if (b == bank) {
            exists = true;
            break;
        }
    }

    // Si el bank no está en la lista, agregarlo, ordenar y reescribir el archivo
    if (!exists) {
        bankList.push_back(bank);
        std::sort(bankList.begin(), bankList.end());

        File f = SPIFFS.open(bankFilePath, "w");
        if (f) {
            for (byte b : bankList) {
                f.write(b);
            }
            f.close();

            DEBUG__________("Bank list updated: ");
            for (byte b : bankList) {
                DEBUG__________("0x");
                DEBUG__________(b, HEX);
                DEBUG__________(" ");
            }
            DEBUG__________ln();
        } else {
            DEBUG__________ln("Error al abrir el archivo de banks para escritura.");
        }
    } else {
        DEBUG__________ln("El bank ya existe en la lista.");
    }
}

std::vector<byte> readBankList() {
    std::vector<byte> bankList;
    const char* bankFilePath = "/bank_list.bin";
    
    if (SPIFFS.exists(bankFilePath)) {
        File f = SPIFFS.open(bankFilePath, "r");
        if (f) {
            while (f.available()) {
                bankList.push_back(f.read());
            }
            f.close();
        } else {
            DEBUG__________ln("Error al abrir el archivo de banks para lectura.");
        }
    }
    
    // Ordenar la lista, por si no está ordenada
    std::sort(bankList.begin(), bankList.end());
    return bankList;
}

void updateBankAndFamilyList(byte bank, const char* familyName) {
    updateBankList(bank);  // Mantener funcionalidad original

    const char* filePath = "/families_list.txt";
    String linePrefix = String(bank) + ":";
    bool exists = false;

    // Leer archivo y comprobar si ya existe el bank
    File f = SPIFFS.open(filePath, "r");
    if (f) {
        while (f.available()) {
            String line = f.readStringUntil('\n');
            if (line.startsWith(linePrefix)) {
                exists = true;
                break;
            }
        }
        f.close();
    }

    if (!exists) {
        File f = SPIFFS.open(filePath, "a");
        if (f) {
            f.printf("%d:%s\n", bank, familyName);
            f.close();
            DEBUG__________printf("Familia añadida: %d -> %s\n", bank, familyName);
        }
    }
}

String getFamilyNameFromBank(byte bank) {
    const char* filePath = "/families_list.txt";
    String linePrefix = String(bank) + ":";
    
    File f = SPIFFS.open(filePath, "r");
    if (!f) return String("Desconocido");

    while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.startsWith(linePrefix)) {
            f.close();
            return line.substring(line.indexOf(':') + 1);
        }
    }
    f.close();
    return String("Desconocido");
}



void saveBrightnessToSPIFFS(uint8_t value) {
    File file = SPIFFS.open("/brightness.txt", FILE_WRITE);
    if (file) {
        file.printf("%d", value);  // Guardamos como texto
        file.close();
        DEBUG__________ln("💾 Brillo guardado en SPIFFS: " + String(value));
    } else {
        DEBUG__________ln("❌ Error al guardar brillo en SPIFFS");
    }
}

uint8_t loadBrightnessFromSPIFFS() {
    if (!SPIFFS.exists("/brightness.txt")) return 100;  // Valor por defecto

    File file = SPIFFS.open("/brightness.txt", FILE_READ);
    if (file) {
        String content = file.readStringUntil('\n');
        file.close();
        int value = content.toInt();
        value = constrain(value, 0, 100);
        DEBUG__________ln("🔆 Brillo cargado desde SPIFFS: " + String(value));
        return value;
    } else {
        DEBUG__________ln("❌ Error al leer brillo desde SPIFFS");
        return 100;
    }
}

void loadDeletableElements() {
    deletableElementFiles.clear();

    fs::File root = SPIFFS.open("/");
    if (!root || !root.isDirectory()) {
        DEBUG__________ln("Error al abrir SPIFFS para eliminar elementos.");
        return;
    }

    fs::File file = root.openNextFile();
    while (file) {
        String fileName = file.name();
        if (!fileName.startsWith("/")) {
            fileName = "/" + fileName;
        }

        if (fileName.startsWith("/element_") && fileName.endsWith(".bin") && fileName.indexOf("_icon") < 0) {
            // Extraer nombre sin extensión ni ruta
            String cleanName = fileName;
            cleanName.remove(0, 9); // quitar "/element_"
            cleanName.replace(".bin", "");
            deletableElementFiles.push_back(cleanName);
        }
        
        file.close();
        file = root.openNextFile();
    }
    deletableElementFiles.push_back(getTranslation("VOLVER"));
}

void saveLanguageToSPIFFS(Language lang) {
    File f = SPIFFS.open("/config_lang.txt", FILE_WRITE);
    if (f) {
        f.write((byte)lang);
        f.close();
    }
}

// Language loadLanguageFromSPIFFS() {
//     File f = SPIFFS.open("/config_lang.txt", FILE_READ);
//     if (f && f.available()) {
//         byte val = f.read();
//         f.close();
//         if (val >= 1 && val <= (byte)Language::X1) {
//             return static_cast<Language>(val);
//         }
//     }
//     return Language::ES;  // idioma por defecto si no existe o es inválido
// }

Language loadLanguageFromSPIFFS() {
    File f = SPIFFS.open("/config_lang.txt", FILE_READ);
    if (f && f.available()) {
        byte val = f.read();
        f.close();
        if (val == 0xFF) return static_cast<Language>(-1);  // sin configurar
        if (val >= 1 && val <= (byte)Language::X1) {
            return static_cast<Language>(val);
        }
    }
    return static_cast<Language>(-1); // valor sin configurar
}


void saveSoundSettingsToSPIFFS()
{
    File f = SPIFFS.open("/config_sonido.txt", FILE_WRITE);
    if (!f) return;

    // 0 = mujer, 1 = hombre
    f.write((uint8_t)selectedVoiceGender);

    // 0 = sin negativa, 1 = con negativa
    f.write((uint8_t)(negativeResponse ? 1 : 0));

    // Volumen (según tu orden de menú):
    // 0 = máximo (30), 1 = normal (20), 2 = atenuado (15)
    uint8_t volSel = (uint8_t)selectedVolume;
    if (volSel > 2) volSel = 1; // clamp a NORMAL por seguridad
    f.write(volSel);

    f.close();
}

void loadSoundSettingsFromSPIFFS()
{
    static const char* kPath = "/config_sonido.txt";



    File f = SPIFFS.open(kPath, "r");
    if (!f) {
        rfDebugPrint("[SND][LOAD] file not found");
        return;
    }

    const size_t sz = f.size();
    rfDebugPrintf("[SND][LOAD] size=%u", (unsigned)sz);

    if (sz < 3) {
        f.close();
        rfDebugPrint("[SND][LOAD] ERROR size<3");
        return;
    }

    // Lee últimos 3 bytes si el fichero creció
    if (sz > 3) {
        f.seek((uint32_t)(sz - 3), SeekSet);
        rfDebugPrintf("[SND][LOAD] seek to %u", (unsigned)(sz - 3));
    }

    uint8_t buf[3] = {0, 0, 1}; // defaults: mujer, sin, normal
    const size_t r = f.read(buf, sizeof(buf));
    f.close();

    rfDebugPrintf("[SND][LOAD] read=%u expected=%u", (unsigned)r, (unsigned)sizeof(buf));
    if (r != sizeof(buf)) {
        rfDebugPrint("[SND][LOAD] ERROR short read");
        return;
    }



    uint8_t vg   = buf[0];
    uint8_t neg  = buf[1];
    uint8_t vsel = buf[2];

    if (vg > 1) vg = 0;
    if (vsel > 2) vsel = 1;

    // Aplicar a RAM
    selectedVoiceGender = vg;
    token.genre         = vg;
    negativeResponse    = (neg == 1);
    selectedVolume      = vsel;

    // Aplicar volumen real
    constexpr uint8_t kVolMaximo   = 30;
    constexpr uint8_t kVolNormal   = 20;
    constexpr uint8_t kVolAtenuado = 15;

    uint8_t vol = kVolNormal;
    if (selectedVolume == 0)      vol = kVolMaximo;
    else if (selectedVolume == 2) vol = kVolAtenuado;

    doitPlayer.player.volume(vol);


    // Compactación si el fichero estaba hinchado
    if (sz != 3) {
        rfDebugPrint("[SND][LOAD] compacting (sz!=3)...");
        saveSoundSettingsToSPIFFS();
    }
}



// --- persistencia on/off de DADO ---
void saveExtraElementsConfig(const ExtraElementsConfig& cfg) {
    fs::File f = SPIFFS.open(EXTRA_CFG_PATH, FILE_WRITE);
    if (!f) return;
    uint8_t b = cfg.dadoEnabled ? 1 : 0;
    f.write(&b, 1);
    f.close();
}

ExtraElementsConfig loadExtraElementsConfig() {
    ExtraElementsConfig cfg{ false };
    fs::File f = SPIFFS.open(EXTRA_CFG_PATH, FILE_READ);
    if (!f) return cfg;
    uint8_t b = 0;
    if (f.read(&b, 1) == 1) cfg.dadoEnabled = (b != 0);
    f.close();
    return cfg;
}

bool isDadoEnabled() { return loadExtraElementsConfig().dadoEnabled; }

void setDadoEnabled(bool enabled) {
    auto cfg = loadExtraElementsConfig();
    cfg.dadoEnabled = enabled;
    saveExtraElementsConfig(cfg);
}

static inline String normalizePath_(const String& name) {
    if (name.length() == 0) return String();
    if (name[0] == '/') return name;
    return "/" + name;
}

bool tryGetNSFromFile(const String& fileName, TARGETNS& outNS)
{
    outNS = NS_ZERO;  // valor por defecto en caso de fallo

    // Normaliza ruta (admite "ElementoX" o "/ElementoX")
    const String path = normalizePath_(fileName);

    // Abre en lectura
    fs::File f = SPIFFS.open(path, "r");
    if (!f) {
    #ifdef DEBUG
        DEBUG__________printf("❌ tryGetNSFromFile: no se pudo abrir '%s'\n", path.c_str());
    #endif
        return false;
    }

    // Comprobación defensiva de tamaño (opcional pero recomendado)
    const size_t kNeeded = (size_t)OFFSET_SERIAL + 5u;
    if ((size_t)f.size() < kNeeded) {
    #ifdef DEBUG
        DEBUG__________printf("❌ tryGetNSFromFile: archivo demasiado pequeño (%u < %u) '%s'\n",
                              (unsigned)f.size(), (unsigned)kNeeded, path.c_str());
    #endif
        f.close();
        return false;
    }

    // Posición segura y lectura de los 5 bytes
    if (!f.seek(OFFSET_SERIAL, SeekSet)) {
    #ifdef DEBUG
        DEBUG__________printf("❌ tryGetNSFromFile: seek(OFFSET_SERIAL=%u) falló en '%s'\n",
                              (unsigned)OFFSET_SERIAL, path.c_str());
    #endif
        f.close();
        return false;
    }

    uint8_t buf[5] = {0,0,0,0,0};
    int readBytes = f.read(buf, 5);
    f.close();

    if (readBytes != 5) {
    #ifdef DEBUG
        DEBUG__________printf("❌ tryGetNSFromFile: sólo se leyeron %d/5 bytes en '%s'\n",
                              readBytes, path.c_str());
    #endif
        return false;
    }

    // Copia segura al struct TARGETNS
    outNS.mac01 = buf[0];
    outNS.mac02 = buf[1];
    outNS.mac03 = buf[2];
    outNS.mac04 = buf[3];
    outNS.mac05 = buf[4];

    return true;
}

TARGETNS getNSFromFile(const String& fileName)
{
    TARGETNS ns = NS_ZERO;
    (void)tryGetNSFromFile(fileName, ns);
    return ns;
}

static inline bool isElementFile_(const String& path) {
  return path.startsWith("/element_") && path.endsWith(".bin") && path.indexOf("_icon") < 0;
}

// String getFilePathByNS(const TARGETNS& ns) {
//     if (nsEqualsZero(ns)) return String();  // no tiene sentido buscar NS_ZERO

//     for (const String& file : elementFiles) {
//         if (!isElementFile_(file)) continue;
//         TARGETNS tmp = NS_ZERO;
//         if (tryGetNSFromFile(file, tmp) && nsEquals(tmp, ns)) {
//             return file;
//         }
//     }
//     return String();  // no encontrado
// }

// bool nsExistsInSPIFFS(const TARGETNS& ns) {
//     if (nsEqualsZero(ns)) return false;

//     fs::File root = SPIFFS.open("/");
//     if (!root || !root.isDirectory()) return false;

//     fs::File f = root.openNextFile();
//     while (f) {
//         String name = f.name();
//         if (!name.startsWith("/")) name = "/" + name;
//         if (isElementFile_(name)) {
//             TARGETNS tmp = NS_ZERO;
//             if (tryGetNSFromFile(name, tmp) && nsEquals(tmp, ns)) {
//                 f.close();
//                 root.close();
//                 return true;
//             }
//         }
//         f.close();
//         f = root.openNextFile();
//     }
//     root.close();
//     return false;
// }

// --- NS/MAC propia de la BOTONERA ---
static TARGETNS s_ownNS = {0,0,0,0,0};               

TARGETNS getOwnNS() {                                  
    return s_ownNS;                                  
}                                                      

void setOwnNS(const TARGETNS& ns) {                    
    s_ownNS = ns;                                      
    setLocalNS(ns);
}                                  

void saveMicSilenceThresholdToSPIFFS(uint8_t value)
{
    value = constrain(value, 0, 255);

    File file = SPIFFS.open("/mic_silence.txt", FILE_WRITE);
    if (!file) {
        DEBUG__________ln("❌ Error al guardar micSilenceThreshold en SPIFFS");
        return;
    }

    file.println(value);
    file.close();

    DEBUG__________ln("🎤 Umbral MIC guardado en SPIFFS: " + String(value));
}

uint8_t loadMicSilenceThresholdFromSPIFFS()
{
    if (!SPIFFS.exists("/mic_silence.txt")) {
        // Default seguro
        return 5;
    }

    File file = SPIFFS.open("/mic_silence.txt", FILE_READ);
    if (!file) {
        DEBUG__________ln("❌ Error al leer micSilenceThreshold desde SPIFFS");
        return 5;
    }

    String content = file.readStringUntil('\n');
    file.close();

    int value = content.toInt();
    value = constrain(value, 0, 255);

    DEBUG__________ln("🎤 Umbral MIC cargado desde SPIFFS: " + String(value));
    return (uint8_t)value;
}










