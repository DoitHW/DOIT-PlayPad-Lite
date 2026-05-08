#pragma once
#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>

// Definiciones de filas y columnas
#define FILAS 4
#define COLUMNAS 3

// Declarar las variables de pines y colores
extern int filas[FILAS];
extern int columnas[COLUMNAS];
extern byte pulsadorColor[FILAS][COLUMNAS];
extern byte relay_state;

// Externs necesarios
extern bool ambienteActivo;
extern std::vector<uint8_t> idsSPIFFS;
extern int  relayStep;
extern bool passiveModeActive;
extern bool passiveIsMashed;
extern std::vector<TARGETNS> nsSPIFFS;
extern uint8_t  communicatorTargetType;
extern TARGETNS communicatorActiveNS;


enum ButtonEventType { BUTTON_PRESSED, BUTTON_RELEASED };

// ==========================================
// Contexto del Botón
// Agrupa configuración y estado del objetivo actual
// ==========================================
struct ButtonContext {
    String   currentFile;
    uint8_t  targetType;
    TARGETNS targetNS;
    bool     isRespMode;
    
    // Flags de Configuración (Mode Config)
    bool hasPulse;
    bool hasPassive;
    bool hasRelay;
    bool hasRelayN1;
    bool hasRelayN2;
    bool hasAdvanced;
    bool hasBasic;
    bool hasPatterns;
    
    // Flags derivados lógicos
    bool isMultiRelay;
    bool isAromaterapia;
    bool isCommunicatorBroadcast; 
};

class PulsadoresHandler {
public:
    PulsadoresHandler();
    void begin();
    static void limpiarEstados();
    
    // Función Principal
    void procesarPulsadores(); 
    
    // Utilidades
    bool relayButtonIsPressed() const { return relayButtonPressed; }
    bool isButtonPressed(byte color);
    byte lastBasicColor = 0; 
    
    static bool isButtonEnabled(uint8_t ledIdx); 
    static void setButtonActiveMask(const bool mask[9]);

    static void setCognitiveDefaultButtonMap();
    static void resetButtonMappedNumColors();

    //mapeo recibido por EXT MAP
    static void setButtonMappedNumColor(uint8_t ledIdx, uint8_t numColor);
    static uint8_t getButtonMappedNumColor(uint8_t ledIdx);
    static void setButtonMappedFx(uint8_t ledIdx, uint8_t fx);
    static uint8_t getButtonMappedFx(uint8_t ledIdx);
    
    // Ruta de Respuesta
    static void setResponseRoute(uint8_t targetType, const TARGETNS& targetNS);
    static void clearResponseRoute();                                          
    static bool isResponseRouteActive();    
    static uint8_t  getResponseTargetType();  
    static TARGETNS getResponseTargetNS();
    
    // Bloqueo Global
    static void setGlobalFxNoInput(bool enable);  
    static bool isGlobalFxNoInput(); 

private:
    bool relayButtonPressed = false;
    bool blueButtonPressed = false;

    // Estados estáticos globales
    static bool     s_responseModeEnabled;
    static uint8_t  s_respTargetType;
    static TARGETNS s_respTargetNS;
    static bool     s_globalFxNoInput;  
    
    //configuracion recibida por F_SET_BUTTONS_EXTMAP para cada LED
    static uint8_t  s_btnMappedNumColor[9];
    static uint8_t  s_btnMappedFx[9];

    // --- Dispatcher Principal ---
    void processButtonEvent(int i, int j, ButtonEventType event, const ButtonContext& ctx);

    // --- Helpers de Contexto ---
    void resolveContext(ButtonContext& ctx); 
    
    // --- Handlers Específicos ---
    void handleCognitiveMenu(int i, int j, ButtonEventType event);
    void handleResponseRoute(byte buttonColor);
    void handleAmbientes(byte buttonColor, ButtonEventType event);
    void handleComunicadorRelayCycle(); // Lógica de ciclo del comunicador
    
    // Lógica de Negocio (Core)
    void handleRelayLogic(byte buttonColor, ButtonEventType event, const ButtonContext& ctx);
    void handleColorLogic(byte buttonColor, ButtonEventType event, const ButtonContext& ctx);

    // Sub-lógicas de Color
    void applyBasicLatched(byte buttonColor, const ButtonContext& ctx);
    void applyBasicPulse(const ButtonContext& ctx);
    void applyAdvancedPaletteNoPulse(byte buttonColor, const ButtonContext& ctx);
    void applyAdvancedPalettePulse(const ButtonContext& ctx);
};

extern PulsadoresHandler pulsadores;

