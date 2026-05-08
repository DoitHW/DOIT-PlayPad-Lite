#pragma once
#include <FS.h>
#include <SPIFFS.h>
#include <string>
#include <Frame_DMS/Frame_DMS.h>
#include <Translations_handler/translations.h>
#include <play_DMS/play_DMS.h>
#include <token_DMS/token_DMS.h>
#include <stdint.h>   // en C o C++


// Cálculo de offsets dentro de INFO_PACK_T:
// Longitudes de campos en INFO_PACK_T
constexpr size_t NAME_LEN    = 32;   // <-- antes 24; AHORA 32
constexpr size_t DESC_LEN    = 192;
constexpr size_t SERIAL_LEN  = 5;    // NS de 5 bytes
constexpr size_t ID_LEN      = 1;    // Placeholder legacy (mantener por ahora)
constexpr size_t CURMODE_LEN = 1;

// Cálculo de offsets dentro de INFO_PACK_T (formato en disco)
constexpr size_t OFFSET_NAME        = 0;                             // = 0
constexpr size_t OFFSET_DESC        = OFFSET_NAME + NAME_LEN;        // = 32
constexpr size_t OFFSET_SERIALNUM   = OFFSET_DESC + DESC_LEN;        // = 32 + 192 = 224
constexpr size_t OFFSET_ID          = OFFSET_SERIALNUM + SERIAL_LEN; // = 224 + 5 = 229  (LEGACY)
constexpr size_t OFFSET_CURRENTMODE = OFFSET_ID + ID_LEN;            // = 229 + 1 = 230

// Estructura de modos (NO cambia): name[24] + desc[192] + config[2] = 218
constexpr size_t SIZE_MODE  = 24 + 192 + 2;       // = 218
constexpr size_t SIZE_MODES = SIZE_MODE * 16;     // = 3488
constexpr size_t OFFSET_MODES = OFFSET_CURRENTMODE + CURMODE_LEN; // = 230 + 1 = 231

// Icono (uint16_t), igual que antes
constexpr size_t OFFSET_ICONO     = OFFSET_MODES + SIZE_MODES;      // = 231 + 3488 = 3719
constexpr size_t OFFSET_SITUACION = OFFSET_ICONO + (ICON_ROWS * ICON_COLUMNS * 2);
// Ejemplo si ICON_ROWS*ICON_COLUMNS*2 = 8192  => OFFSET_SITUACION = 3719 + 8192 = 11911

// Alias de compatibilidad (para no romper código existente que usa OFFSET_SERIAL)
#ifndef OFFSET_SERIAL
  #define OFFSET_SERIAL OFFSET_SERIALNUM
#endif


// icono =64*64*2=8192 bytes total ~11900

extern uint16_t lineBuffer[64]; 
extern INFO_PACK_T ambientesOption;
extern INFO_PACK_T fichasOption;
extern INFO_PACK_T apagarSala;
extern INFO_PACK_T comunicadorOption;
extern TOKEN_ token;
extern DOITSOUNDS_ doitPlayer;



bool writeBytesChecked(fs::File &f, const uint8_t* data, size_t length);
void formatSPIFFS();
String generateUniqueFileName(const char* baseName);

bool readElementData(fs::File& f, char* elementName, char* modeName, int& startX, int& startY);
void loadElementsFromSPIFFS();
void initializeDynamicOptions();
//byte getCurrentElementID();
TARGETNS getCurrentElementNS();
bool isCurrentElementSelected();
bool checkMostSignificantBit(byte modeConfig[2]);
bool getModeConfig(const String& fileName, byte mode, byte modeConfig[2]);
void setAllElementsToBasicMode();
void updateBankList(byte bank);
void updateBankAndFamilyList(byte bank, const char* familyName);
String getFamilyNameFromBank(byte bank);
void saveBrightnessToSPIFFS(uint8_t value);
uint8_t loadBrightnessFromSPIFFS();
std::vector<byte> readBankList();
void loadDeletableElements();
void saveLanguageToSPIFFS(Language lang);
Language loadLanguageFromSPIFFS();
void saveSoundSettingsToSPIFFS();
void loadSoundSettingsFromSPIFFS();
void    saveMicSilenceThresholdToSPIFFS(uint8_t value);
uint8_t loadMicSilenceThresholdFromSPIFFS();


// --- NS/MAC propia de la BOTONERA ---
TARGETNS getOwnNS();                    
void     setOwnNS(const TARGETNS& ns); 

// ===== Elementos Adicionales (persistencia) =====
struct ExtraElementsConfig {
    bool dadoEnabled;
};

void saveExtraElementsConfig(const ExtraElementsConfig& cfg);
ExtraElementsConfig loadExtraElementsConfig();
bool isDadoEnabled();
void setDadoEnabled(bool enabled);

inline bool nsEqualsZero(const TARGETNS& ns) {
  return (ns.mac01|ns.mac02|ns.mac03|ns.mac04|ns.mac05) == 0;
}

static inline bool nsEquals(const TARGETNS& a, const TARGETNS& b) {
    return a.mac01==b.mac01 && a.mac02==b.mac02 && a.mac03==b.mac03 && a.mac04==b.mac04 && a.mac05==b.mac05;
}

// ------------------ API centrada en NS ------------------
bool     tryGetNSFromFile(const String& fileName, TARGETNS& outNS);
TARGETNS getNSFromFile(const String& fileName);
//String   getFilePathByNS(const TARGETNS& ns);       // NUEVA: busca el fichero por NS
//bool     nsExistsInSPIFFS(const TARGETNS& ns);      // NUEVA: true si existe el NS
TARGETNS getCurrentElementNS();   



// Elemento fijo en RAM
extern INFO_PACK_T dadoOption;

// ¿Es todo cero?
inline bool NS_is_zero(const TARGETNS& ns) {
    // más rápido y compacto que 5 comparaciones:
    return (ns.mac01 | ns.mac02 | ns.mac03 | ns.mac04 | ns.mac05) == 0;
}

// Igualdad / desigualdad (soluciona el error de "no operator !=")
inline bool operator==(const TARGETNS& a, const TARGETNS& b) {
    return a.mac01 == b.mac01 &&
           a.mac02 == b.mac02 &&
           a.mac03 == b.mac03 &&
           a.mac04 == b.mac04 &&
           a.mac05 == b.mac05;
}

inline bool operator!=(const TARGETNS& a, const TARGETNS& b) {
    return !(a == b);
}

// (Opcional) creador rápido
inline TARGETNS make_ns(byte a, byte b, byte c, byte d, byte e) {
    return TARGETNS{a,b,c,d,e};
}
