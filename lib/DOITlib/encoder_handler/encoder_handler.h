#pragma once

#include <SPIFFS_handler/SPIFFS_handler.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <vector>

                                                        //3V3 desconectado
#if defined(BOTONERA_NEW)
// Pines del encoder            //BOTONERA FRANC        BOTONERA MARC
#define ENC_A 34                //26                    34                      (33 para nuevas botoneras sin placa)
#define ENC_B 33                //34                    33                      (34 para nuevas botoneras sin placa)
#define ENC_BUTTON 26           //33                    26                      (26 para nuevas botoneras sin placa)

#elif defined(BOTONERA_OLD)
// Pines del encoder            //BOTONERA FRANC        BOTONERA MARC
#define ENC_A 26                //26                    34                      (33 para nuevas botoneras sin placa)
#define ENC_B 34                //34                    33                      (34 para nuevas botoneras sin placa)
#define ENC_BUTTON 33           //33                    26                      (26 para nuevas botoneras sin placa)
#endif

enum MODE_CONFIGS{
    HAS_BASIC_COLOR= 0,
    HAS_PULSE,
    HAS_ADVANCED_COLOR,
    HAS_RELAY,
    HAS_RELAY_N1,
    HAS_RELAY_N2,
    NOP_1,
    HAS_SENS_VAL_1,
    HAS_SENS_VAL_2,
    NOP_2,
    HAS_PASSIVE,
    HAS_BINARY_SENSORS,
    HAS_BANK_FILE,
    HAS_PATTERNS,
    HAS_ALTERNATIVE_MODE,
    MODE_EXIST
};

// Declaración de funciones relacionadas con el encoder
void encoder_init_func() noexcept;
void handleEncoder() noexcept;
void handleHiddenMenuNavigation(int &hiddenMenuSelection) noexcept;

// Declaración de funciones externas (forward declarations)
//void animateTransition(int direction);
void drawModesScreen();
void drawCurrentElement();

void handleModeSelection(const String& currentFile) noexcept;
//void toggleElementSelection(const String& currentFile) noexcept;
//void requestAndSyncElementMode();

bool getModeFlag(const uint8_t modeConfig[2], MODE_CONFIGS flag) noexcept;
//void debugModeConfig(const uint8_t modeConfig[2]) noexcept;

std::vector<bool> initializeAlternateStates(const String &currentFile) noexcept;

void handleBankSelectionMenu(std::vector<byte>& bankList, std::vector<bool>& selectedBanks) noexcept;

void handleBrightnessMenu();

static void handleMicCalibMenu();

void handleSoundMenu();

void handleFormatMenu();

void handleDeleteElementMenu();

void handleConfirmDelete();

void handleConfirmRestoreMenu() noexcept;

void handleConfirmRestoreElementMenu();

bool isInMainMenu();

void enfocarElemento(const String& name);

void drawMicCalibrationMenu(int sens);


//int getTotalModesForFile(const String &file);

void printElementDetails();
// Variables externas requeridas
extern std::vector<String> elementFiles;
extern std::vector<char> selectedStates;
extern int currentIndex;
extern int totalModes;
extern int currentModeIndex;
extern bool inModesScreen;
//extern bool modeScreenEnteredByLongPress;
extern bool isLongPress;
extern bool hiddenMenuActive;
extern unsigned long buttonPressStart;
extern int globalVisibleModesMap[17];  // Declaración de la variable global
extern bool ignoreInputs;
extern ESP32Encoder encoder;
//extern bool modeAlternateActive;

extern unsigned long lastDisplayInteraction; // Última vez que se interactuó con la pantalla
extern bool displayOn;                    // Estado de la pantalla (encendida por defecto)
extern unsigned long encoderIgnoreUntil; // Tiempo hasta el cual se ignoran las entradas del encoder
extern bool systemLocked;


// Variables para el submenú de selección de idioma
extern bool languageMenuActive;
extern int languageMenuSelection;  // Índice de la opción seleccionada (0 a 5)

extern bool formatSubMenuActive;
extern int formatMenuSelection;


extern bool soundMenuActive;
extern int soundMenuSelection;
extern byte selectedVoiceGender;
extern bool negativeResponse;
extern byte selectedVolume;

extern bool deleteElementMenuActive;
extern int deleteElementSelection;
extern std::vector<String> deletableElementFiles;
extern bool confirmDeleteActive;
extern int confirmSelection;
extern bool confirmDeleteMenuActive;
extern bool ignoreNextEncoderClick;

extern bool confirmRestoreMenuActive;
extern int confirmRestoreSelection;
extern bool confirmRestoreMenuElementActive;
extern int confirmRestoreElementSelection;

extern unsigned long lastFocusChangeTime;
extern int lastQueriedElementIndex;


extern unsigned long lastModeQueryTime;
extern int pendingQueryIndex;
extern TARGETNS pendingQueryNS;
extern bool awaitingResponse;

extern int formatMenuCurrentIndex;
extern int32_t formatMenuLastValue;

enum BrightnessOption {
    BRIGHTNESS_NORMAL = 0,
    BRIGHTNESS_DIM    = 1
};

void handleExtraElementsMenu();
void handleConfirmEnableDadoMenu();

extern bool extraElementsMenuActive;
extern int  extraElementsMenuSelection;
extern bool confirmEnableDadoActive;
extern int  confirmEnableDadoSelection;






 







