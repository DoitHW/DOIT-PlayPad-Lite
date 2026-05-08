#pragma once

#include <FS.h>
#include <TFT_eSPI.h>
#include <SPIFFS.h>
#include <vector>
#include <map>
#include <string>

#define BACKGROUND_COLOR TFT_BLACK
#define TEXT_COLOR TFT_WHITE
#define HIGHLIGHT_COLOR TFT_GREEN
#define CARD_COLOR TFT_NAVY

// Declaración adelantada (forward) para no generar bucle de includes.
class DynamicLEDManager;
void display_init(void) noexcept;
void drawErrorMessage(const char* message);
void drawElementIcon(fs::File& f, int startX, int startY);
void drawHiddenMenu(int selection);
void scrollTextTickerBounce(int selection);
void showMessageWithLoading(const char* message, unsigned long delayTime);
void updateNameScroll();
void updateModeScroll();
String getModeDisplayName(const String &fullModeName, bool alternateActive);
void display_sleep();
void display_wakeup();
void drawLanguageMenu(int selection);
void drawBankSelectionMenu(const std::vector<byte>& bankList, const std::vector<bool>& selectedBanks, int currentSelection, int windowOffset);
void drawBrightnessMenu();
#ifdef MIC
void drawMicCalibMenu(uint8_t editingThreshold, uint8_t liveRaw, bool showSaved);
#endif
void drawSunIcon(int16_t x, int16_t y, uint16_t color);
void drawSoundMenu(int selection);
//void scrollTextTickerBounceSound(int selection);
void drawFormatMenu(int selection);
void drawDeleteElementMenu(int selection);
void drawConfirmDelete(const String& fileName);
void showCriticalBatteryMessage();
void drawBatteryIconMini(float percentage);
void drawCognitiveMenu();
void drawConfirmRestoreMenu(int selection);
void drawConfirmRestoreElementMenu(int selection);
void showElemInfo(unsigned long delayTime, const String& serialNumber, const String& elementID);
void mostrarTextoAjustado(TFT_eSPI& tft,
                                 const char* texto,
                                 uint16_t xCentro,
                                 uint16_t yInicio,
                                 uint16_t maxWidth);
void setFontForCurrentLanguage();
//void scrollFileNameTickerBounce(const String& fileName);

void drawLoadingModalFrame(const char* message, int frameCount);
void showWelcomeAnimation();

// Estructura para la caché de modos (Optimización RAM)
struct ModeCacheEntry {
    int originalIndex;  // Índice real (0-15), o -2 (Volver), -3 (Apagar/Encender)
    String label;       // Texto ya traducido y listo para mostrar
    bool isAlternate;   // Si es un modo alternativo (para lógica visual si fuera necesaria)
};

// Declaración de la función de carga y la caché externa
void loadModesCache(const String& fileName);
extern std::vector<ModeCacheEntry> cachedModes;
                                 
extern TFT_eSPI tft;
extern TFT_eSprite uiSprite;
extern bool nameScrollActive;
extern bool modeScrollActive;
extern bool brightnessMenuActive;
extern std::map<String, std::vector<bool>> elementAlternateStates;
extern std::vector<bool> currentAlternateStates;
extern uint8_t currentBrightness;       // Valor actual en porcentaje
extern uint8_t tempBrightness; 
extern bool forceDrawDeleteElementMenu;
extern int bankMenuCurrentSelection;   // 0: Confirmar, 1..n: banks
extern int bankMenuWindowOffset;
extern bool criticalBatteryLock; // lo usarás en otras partes
extern float batteryPercentage;

uint16_t colorWheel(uint8_t pos);

extern int brightnessMenuIndex;

//void showMessageWithProgress(const char* message, unsigned long delayTime);

// Extra Elements menu
void drawExtraElementsMenu(int selection);
void drawConfirmEnableDadoMenu(int selection);

extern bool extraElementsMenuActive;
extern int  extraElementsMenuSelection;

extern bool confirmEnableDadoActive;
extern int  confirmEnableDadoSelection;


