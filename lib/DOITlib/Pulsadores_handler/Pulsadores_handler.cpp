#include <Colors_DMS/Color_DMS.h>
#include <Frame_DMS/Frame_DMS.h>
#include <Pulsadores_handler/Pulsadores_handler.h>
#include <RelayManager_DMS/RelayStateManager.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <algorithm>
#include <botonera_DMS/botonera_DMS.h>
#include <defines_DMS/defines_DMS.h>
#include <display_handler/display_handler.h>
#include <encoder_handler/encoder_handler.h>


// ==========================================
// Configuración de Hardware
// ==========================================
#if defined(BOTONERA_NEW)
int filas[FILAS] = {5, 4, 6, 7};
int columnas[COLUMNAS] = {3, 2, 1};
byte pulsadorColor[FILAS][COLUMNAS] = {{ORANGE, GREEN, WHITE},
                                       {BLUE, RELAY, RED},
                                       {VIOLET, YELLOW, LIGHT_BLUE},
                                       {BLACK, BLACK, BLACK}};
#elif defined(BOTONERA_OLD)
int filas[FILAS] = {4, 5, 6, 7};
int columnas[COLUMNAS] = {1, 2, 3};
byte pulsadorColor[FILAS][COLUMNAS] = {{ORANGE, GREEN, WHITE},
                                       {BLUE, RELAY, RED},
                                       {LIGHT_BLUE, YELLOW, VIOLET},
                                       {BLACK, BLACK, BLACK}};
#endif

// Estados globales estáticos
static bool lastState[FILAS][COLUMNAS];
static unsigned long pressTime[FILAS][COLUMNAS] = {{0}}; // Tiempo de pulsación
byte relay_state = 0x00;
std::vector<uint8_t> idsSPIFFS;
int  relayStep = -1;
TARGETNS communicatorActiveNS = NS_ZERO;
bool passiveModeActive = false;
bool passiveIsMashed = false;

// Inicialización de estáticos de clase
bool PulsadoresHandler::s_responseModeEnabled = false;
uint8_t PulsadoresHandler::s_respTargetType = 0;
TARGETNS PulsadoresHandler::s_respTargetNS = {0, 0, 0, 0, 0};
bool PulsadoresHandler::s_globalFxNoInput = false;

// Helpers Locales
static inline int colorToLedIdx(byte color) {
  switch (color) {
  case RELAY:
    return 0;
  case WHITE:
    return 1;
  case RED:
    return 2;
  case LIGHT_BLUE:
    return 3;
  case YELLOW:
    return 4;
  case ORANGE:
    return 5;
  case GREEN:
    return 6;
  case VIOLET:
    return 7;
  case BLUE:
    return 8;
  default:
    return -1;
  }
}

static inline bool _isNSZero(const TARGETNS &ns) {
  return memcmp(&ns, &NS_ZERO, sizeof(TARGETNS)) == 0;
}

// ==========================================
// Implementación Clase Principal
// ==========================================

PulsadoresHandler::PulsadoresHandler() {}

void PulsadoresHandler::begin() {
  for (int i = 0; i < FILAS; i++) {
    pinMode(filas[i], OUTPUT);
    digitalWrite(filas[i], HIGH);
  }
  for (int j = 0; j < COLUMNAS; j++) {
    pinMode(columnas[j], INPUT_PULLUP);
  }
}

void PulsadoresHandler::limpiarEstados() {
  for (int i = 0; i < FILAS; i++) {
    for (int j = 0; j < COLUMNAS; j++) {
      lastState[i][j] = false;
      pressTime[i][j] = 0;
    }
  }
}

static bool g_btnEnabled[9] = {true, true, true, true, true,
                               true, true, true, true};

bool PulsadoresHandler::isButtonEnabled(uint8_t ledIdx) {
  return (ledIdx < 9) ? g_btnEnabled[ledIdx] : true;
}

void PulsadoresHandler::setButtonActiveMask(const bool mask[9]) {
  for (int i = 0; i < 9; ++i)
    g_btnEnabled[i] = mask[i];
}

void PulsadoresHandler::resetButtonMappedNumColors()
{
    for (uint8_t i = 0; i < 9; ++i) {
        s_btnMappedNumColor[i] = 0xFF;
        s_btnMappedFx[i] = (uint8_t)BTN_FX::NO_FX;
    }
}

void PulsadoresHandler::setCognitiveDefaultButtonMap()
{
    bool mask[9] = { false };

    const int relayIdx  = colorToLedIdx(RELAY);
    const int blueIdx   = colorToLedIdx(BLUE);
    const int greenIdx  = colorToLedIdx(GREEN);
    const int yellowIdx = colorToLedIdx(YELLOW);
    const int redIdx    = colorToLedIdx(RED);

    if (relayIdx  >= 0) mask[relayIdx]  = true;
    if (blueIdx   >= 0) mask[blueIdx]   = true;
    if (greenIdx  >= 0) mask[greenIdx]  = true;
    if (yellowIdx >= 0) mask[yellowIdx] = true;
    if (redIdx    >= 0) mask[redIdx]    = true;

    setButtonActiveMask(mask);
    resetButtonMappedNumColors();
    limpiarEstados();
    clearResponseRoute();
}

// Getters/Setters estáticos
void PulsadoresHandler::setResponseRoute(uint8_t targetType,
                                         const TARGETNS &targetNS) {
  s_respTargetType = targetType;
  s_respTargetNS = targetNS;
  s_responseModeEnabled = true;
}
void PulsadoresHandler::clearResponseRoute() {
  s_responseModeEnabled = false;
  s_respTargetType = 0;
  s_respTargetNS = NS_ZERO;
}
bool PulsadoresHandler::isResponseRouteActive() {
  return s_responseModeEnabled;
}
uint8_t PulsadoresHandler::getResponseTargetType() { return s_respTargetType; }
TARGETNS PulsadoresHandler::getResponseTargetNS() { return s_respTargetNS; }
void PulsadoresHandler::setGlobalFxNoInput(bool enable) {
  s_globalFxNoInput = enable;
}
bool PulsadoresHandler::isGlobalFxNoInput() { return s_globalFxNoInput; }

bool PulsadoresHandler::isButtonPressed(byte color) {
  for (int i = 0; i < FILAS; i++) {
    digitalWrite(filas[i], LOW);
    delayMicroseconds(1);
    for (int j = 0; j < COLUMNAS; j++) {
      if (pulsadorColor[i][j] != color)
        continue;
      int ledIdx = colorToLedIdx(color);
      if (ledIdx >= 0 && !isButtonEnabled(ledIdx))
        continue;
      if (digitalRead(columnas[j]) == LOW) {
        digitalWrite(filas[i], HIGH);
        return true;
      }
    }
    digitalWrite(filas[i], HIGH);
  }
  return false;
}

// ==========================================
// 1. RESOLUCIÓN DE CONTEXTO (Flags y Target)
// ==========================================
void PulsadoresHandler::resolveContext(ButtonContext &ctx) {
  ctx.currentFile = elementFiles[currentIndex];
  ctx.isRespMode = isResponseRouteActive();

  // 1.1 Determinar Target
  if (ctx.isRespMode) {
    ctx.targetType = getResponseTargetType();
    ctx.targetNS = getOwnNS();
  } else if (ctx.currentFile == "Ambientes") {
    ctx.targetType = BROADCAST;
    ctx.targetNS = NS_ZERO;
  } else if (ctx.currentFile == "Fichas") {
    ctx.targetType = DEFAULT_CONSOLE;
    ctx.targetNS = getOwnNS();
  } else if (ctx.currentFile == "Comunicador") {
    if (_isNSZero(communicatorActiveNS)) {
      ctx.targetType = BROADCAST;
      ctx.targetNS = NS_ZERO;
    } else {
      ctx.targetType = DEFAULT_DEVICE;
      ctx.targetNS = communicatorActiveNS;
    }
  } else {
    ctx.targetType = DEFAULT_DEVICE;
    ctx.targetNS = getCurrentElementNS();
  }

  ctx.isCommunicatorBroadcast =
      (ctx.currentFile == "Comunicador" && _isNSZero(communicatorActiveNS));

  // 1.2 Obtener Configuración (Flags)
  uint8_t modeConfig[2] = {0, 0};
  bool cfgOK = false;

  // Cache local para evitar lecturas repetidas de SPIFFS cuando
  // ni el NS ni el modo han cambiado.
  struct ModeConfigCache {
    bool valid;
    TARGETNS ns;
    int modeIdx;
    uint8_t cfg[2];
  };

  // Inicialización estática
  static ModeConfigCache s_cache = {false, {0, 0, 0, 0, 0}, -1, {0, 0}};

  // Helper local para comparar TARGETNS (5 bytes)
  auto sameNS = [](const TARGETNS &a, const TARGETNS &b) -> bool {
    return a.mac01 == b.mac01 && a.mac02 == b.mac02 && a.mac03 == b.mac03 &&
           a.mac04 == b.mac04 && a.mac05 == b.mac05;
  };

  // Archivos especiales con manejo propio o sin flags
  bool isSpecial =
      (ctx.currentFile == "Ambientes" || ctx.currentFile == "Fichas" ||
       ctx.currentFile == "Comunicador" || ctx.currentFile == "Apagar");

  if (!isSpecial && !ctx.isRespMode && ctx.targetType == DEFAULT_DEVICE) {
    // 1º: intentar usar la caché local
    if (s_cache.valid && sameNS(s_cache.ns, ctx.targetNS) &&
        s_cache.modeIdx == currentModeIndex) {
      modeConfig[0] = s_cache.cfg[0];
      modeConfig[1] = s_cache.cfg[1];
      cfgOK = true;
    } else {
      // 2º: pedir al RelayStateManager (cache propio o lectura)
      cfgOK = RelayStateManager::getModeConfigForNS(ctx.targetNS, modeConfig);

      // 3º: Fallback robusto a lectura de archivo si falla RelayManager
      if (!cfgOK) {
        fs::File f = SPIFFS.open(ctx.currentFile, "r");
        if (f) {
          int rIdx = 0;
          f.seek(OFFSET_CURRENTMODE, SeekSet);
          if (f.available())
            rIdx = f.read();
          if (rIdx < 0)
            rIdx = 0;
          if (rIdx > 15)
            rIdx = 15;

          // 216 es el offset del config dentro del bloque del modo
          f.seek(OFFSET_MODES + rIdx * SIZE_MODE + 216, SeekSet);
          if (f.available() >= 2) {
            f.read(modeConfig, 2);
            cfgOK = true;
          }
          f.close();
        }
      }

      // 4º: actualizar caché si hemos obtenido una config válida
      if (cfgOK) {
        s_cache.valid = true;
        s_cache.ns = ctx.targetNS;
        s_cache.modeIdx = currentModeIndex;
        s_cache.cfg[0] = modeConfig[0];
        s_cache.cfg[1] = modeConfig[1];
      }
    }
  } else if (ctx.currentFile == "Comunicador" && !ctx.isCommunicatorBroadcast) {
    // Comunicador apuntando a dispositivo
    cfgOK = RelayStateManager::getModeConfigForNS(ctx.targetNS, modeConfig);
    // Para evitar mezclar contextos muy distintos, invalidamos la caché
    s_cache.valid = false;
  } else {
    // Otros casos (especiales, respMode, broadcast, etc.) no usan esta caché
    s_cache.valid = false;
  }

  // 1.3 Asignar Flags
  if (cfgOK) {
    ctx.hasPulse = getModeFlag(modeConfig, HAS_PULSE);
    ctx.hasPassive = getModeFlag(modeConfig, HAS_PASSIVE);
    ctx.hasRelay = getModeFlag(modeConfig, HAS_RELAY);
    ctx.hasRelayN1 = getModeFlag(modeConfig, HAS_RELAY_N1);
    ctx.hasRelayN2 = getModeFlag(modeConfig, HAS_RELAY_N2);
    ctx.hasAdvanced = getModeFlag(modeConfig, HAS_ADVANCED_COLOR);
    ctx.hasBasic = getModeFlag(modeConfig, HAS_BASIC_COLOR);
    ctx.hasPatterns = getModeFlag(modeConfig, HAS_PATTERNS);
  } else {
    // Defaults seguros si no hay config (ej: Broadcast general)
    ctx.hasPulse = false;
    ctx.hasPassive = false;
    ctx.hasRelay = true;
    ctx.hasRelayN1 = false;
    ctx.hasRelayN2 = false;
    ctx.hasAdvanced = false;
    ctx.hasBasic = true;
    ctx.hasPatterns = false;
  }

  // Lógica derivada
  ctx.isMultiRelay = (ctx.hasRelayN1 || ctx.hasRelayN2);
  ctx.isAromaterapia = (!ctx.hasRelay && ctx.isMultiRelay);
}

// ==========================================
// 2. FUNCIÓN PRINCIPAL: procesarPulsadores
// ==========================================
void PulsadoresHandler::procesarPulsadores() {

  String currentFile = elementFiles[currentIndex];

  // Ignorar completamente los pulsadores cuando:
  //  - Estamos en la pantalla de modos de un elemento "normal" (no fijos)
  //  - O está activo cualquier menú / submenú donde se navega con encoder
  if (
      // 1) Pantalla de modos de un elemento SPIFFS
      (inModesScreen && currentFile != "Ambientes" && currentFile != "Fichas" &&
       currentFile != "Comunicador" && currentFile != "Dado")

      // 2) Menú oculto de ajustes y submenús
      || hiddenMenuActive

      // 3) Submenús específicos
      || soundMenuActive || brightnessMenuActive || formatSubMenuActive ||
      deleteElementMenuActive || confirmDeleteMenuActive ||
      confirmRestoreMenuActive ||
      confirmRestoreMenuElementActive

      // 4) Menús extra (Dado, elementos extra…)
      || extraElementsMenuActive ||
      confirmEnableDadoActive

      // 5) Cualquier modal que ya marca que no debe haber entrada
      || ignoreInputs) {
    relayButtonPressed = false;
    blueButtonPressed = false;
    return;
  }

  // A) Filtros Globales mínimos
  if (elementFiles[currentIndex] == "Apagar")
    return;
  const bool noInput = isGlobalFxNoInput();
  const bool respMode = isResponseRouteActive();

  // B) Menú Cognitivo → prioridad absoluta
  //    Si estás en actividades cognitivas, lo primero es esta ruta.
  if (inCognitiveMenu && !respMode && !noInput) {
    for (int i = 0; i < FILAS; i++) {
      digitalWrite(filas[i], LOW);
      delayMicroseconds(10);

      for (int j = 0; j < COLUMNAS; j++) {
        byte color = pulsadorColor[i][j];
        bool pressed = (digitalRead(columnas[j]) == LOW);

        // Sólo botones relevantes: colores + RELAY
        if (color != BLUE && color != GREEN && color != YELLOW &&
            color != RED && color != RELAY) {
          pressed = false;
        }

        // En menú cognitivo IGNORAMOS la máscara de 'active'.
        // La idea es que cognitivas siempre funcione, independientemente
        // de lo que haya hecho setButtonActiveMask() en otros contextos.

        if (pressed != lastState[i][j]) {
          handleCognitiveMenu(i, j, pressed ? BUTTON_PRESSED : BUTTON_RELEASED);
          lastState[i][j] = pressed;
        }
      }

      digitalWrite(filas[i], HIGH);
    }
    return; // Nada más se ejecuta en modo cognitivo
  }

  // C) A partir de aquí, el resto de modos “normales”
  ButtonContext ctx;
  resolveContext(ctx);

  bool isSelected = selectedStates[currentIndex];
  bool isException = (ctx.currentFile == "Ambientes" ||
                      /*ctx.currentFile == "Fichas"    ||*/
                      ctx.currentFile == "Comunicador");

  if (!respMode && !isException && !isSelected)
    return;

  if (ctx.currentFile == "Comunicador" && !isSelected && !respMode) {
    relayButtonPressed = false;
    blueButtonPressed = false;
    return;
  }

  if (noInput && !respMode)
    return;

  // E) Reset de estados si cambia el modo
  static int lastModeIndex = -1;
  if (currentModeIndex != lastModeIndex) {
    limpiarEstados(); // Resetea pressTime y lastState
    lastModeIndex = currentModeIndex;
    // Reiniciar paleta del modo avanzado
    applyAdvancedPaletteNoPulse(BLACK, ctx); // Reset hack
  }

  // F) BUCLE PRINCIPAL
  bool curRelayState = false;
  bool curBlueState = false;
  bool stateChanged = false;

  for (int i = 0; i < FILAS; i++) {
    digitalWrite(filas[i], LOW);
    delayMicroseconds(10);

    for (int j = 0; j < COLUMNAS; j++) {
      byte color = pulsadorColor[i][j];
      bool isPressed = (digitalRead(columnas[j]) == LOW);

      // F.1 Filtro Máscara (Gate)
      // RELAY siempre permitido si hasRelay, RespMode o Broadcast Comunicador
      int ledIdx = colorToLedIdx(color);
      if (ledIdx >= 0 && !isButtonEnabled(ledIdx)) {
        bool allowed =
            ctx.isCommunicatorBroadcast || (color == RELAY && ctx.hasRelay);
        // En modo respuesta, respeta siempre la máscara de active → no añadas
        // respMode aquí
        if (!allowed)
          isPressed = false;
      }

      // F.2 Actualizar Tiempos
      if (color != RELAY) {
        if (!lastState[i][j] && isPressed)
          pressTime[i][j] = millis();
        else if (lastState[i][j] && !isPressed)
          pressTime[i][j] = 0;
      }

      // F.3 Detectar Flancos y Disparar Eventos
      if (isPressed != lastState[i][j]) {
        stateChanged = true;
        ButtonEventType evt = isPressed ? BUTTON_PRESSED : BUTTON_RELEASED;
        processButtonEvent(i, j, evt, ctx);
        lastState[i][j] = isPressed;
      }

      if (color == RELAY)
        curRelayState |= isPressed;
      if (color == BLUE)
        curBlueState |= isPressed;
    }
    digitalWrite(filas[i], HIGH);
  }

  relayButtonPressed = curRelayState;
  blueButtonPressed = curBlueState;

  // G) Lógica Continua (Pulse / Mezcla)
  // Se ejecuta si hay cambios de estado o estamos en modo pulso activo
  if (!respMode && !inModesScreen && ctx.currentFile != "Fichas" &&
      ctx.hasPulse) {
    // En modos PULSE, recalculamos la salida en cada ciclo (o al menos cuando
    // cambia algo) para garantizar consistencia.
    if (ctx.hasAdvanced) {
      applyAdvancedPalettePulse(ctx);
    } else {
      applyBasicPulse(ctx);
    }
  }
}

// ==========================================
// 3. DISPATCHER: processButtonEvent
// ==========================================
void PulsadoresHandler::processButtonEvent(int i, int j, ButtonEventType event,
                                           const ButtonContext &ctx) {
  byte buttonColor = pulsadorColor[i][j];

  // 1. Fichas (Solo actualiza flags globales, ya hecho en el loop)
  if (ctx.currentFile == "Fichas")
    return;

  // 2. Ruta Respuesta (modo mapeo desde consola)
  //    Si estamos en modo respuesta, sólo gestionamos el RESPONSE y salimos.
  if (ctx.isRespMode) {
    if (event == BUTTON_PRESSED) {
      handleResponseRoute(buttonColor);
    }
    return;
  }

  // 3. Ambientes
  if (ctx.currentFile == "Ambientes") {
    handleAmbientes(buttonColor, event);
    return;
  }

  // 4. Comunicador
  if (ctx.currentFile == "Comunicador") {
    // Caso Relé de Ciclo (Botón RELAY)
    if (buttonColor == RELAY) {
      if (event == BUTTON_PRESSED && digitalRead(ENC_BUTTON) != LOW) {
        handleComunicadorRelayCycle();
      }
      return;
    }
    // Caso "Solo Relé" (Sin color, con Relé)
    if (!ctx.isCommunicatorBroadcast && !ctx.hasBasic && !ctx.hasAdvanced &&
        ctx.hasRelay) {
      if (buttonColor == BLUE && event == BUTTON_PRESSED) {
        // Blue actúa como toggle
        bool cur = RelayStateManager::get(ctx.targetNS);
        RelayStateManager::set(ctx.targetNS, !cur);
        send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, DEFAULT_DEVICE,
                                             ctx.targetNS, !cur ? 0x01 : 0x00));
      }
      return;
    }
  }

  // 5. Relés (Lógica Unificada)
  // Si es botón RELAY o estamos en modo MultiRelay (colores actúan como relés)
  bool isRelayAction = (buttonColor == RELAY);
  bool isMultiRelayAction = (ctx.isMultiRelay || ctx.isAromaterapia) &&
                            (buttonColor == BLUE || buttonColor == GREEN ||
                             buttonColor == YELLOW || buttonColor == RED);

  if (isRelayAction || isMultiRelayAction) {
    handleRelayLogic(buttonColor, event, ctx);
    return;
  }

  // 6. Colores (Lógica de Modos)
  // Si no es relé y no es un modo especial ya tratado, es lógica de color
  if (buttonColor != RELAY) {
    handleColorLogic(buttonColor, event, ctx);
  }
}

// ==========================================
// 4. HANDLERS ESPECÍFICOS
// ==========================================

// 4.1 Menú Cognitivo
void PulsadoresHandler::handleCognitiveMenu(int i, int j,
                                            ButtonEventType event) {
  byte color = pulsadorColor[i][j];
  if (color != BLUE && color != GREEN && color != YELLOW && color != RED &&
      color != RELAY)
    return;

  TARGETNS ownNS = getOwnNS();
  if (event == BUTTON_PRESSED && color != RELAY) {
    // send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, DEFAULT_CONSOLE,
    // ownNS, color));
    send_frame(frameMaker_SEND_RESPONSE(DEFAULT_BOTONERA, DEFAULT_CONSOLE,
                                        ownNS, color));
  } else if (event == BUTTON_RELEASED && color == RELAY) {
    send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, DEFAULT_CONSOLE,
                                         ownNS, 0x01));
  }
}

void PulsadoresHandler::handleResponseRoute(byte buttonColor) {
  const int ledIdx = colorToLedIdx(buttonColor);
  if (ledIdx < 0)
    return;

  // 1) Respetar la máscara de botones activos del mapeo
  if (!PulsadoresHandler::isButtonEnabled(static_cast<uint8_t>(ledIdx)))
    return;

  // 2) Consultar el numColor configurado por F_SET_BUTTONS_EXTMAP
  uint8_t mapped =
      PulsadoresHandler::getButtonMappedNumColor(static_cast<uint8_t>(ledIdx));

  uint8_t colorToSend = 0;

  // Caso A: color de lista → enviar numColor tal cual
  if (mapped != 0xFF && mapped != BTN_RGB_DIRECT) {
    colorToSend = mapped;
  } else {
    // Caso B: RGB directo (0x80) o sin mapeo → comportamiento antiguo:
    // enviar el "valor normal" del botón (ID 1..9)
    auto ledIdxToButtonId = [](uint8_t idx) -> uint8_t {
      static const uint8_t kMap[9] = {5, 9, 4, 8, 3, 7, 2, 6, 1};
      return (idx < 9) ? kMap[idx] : 0;
    };
    colorToSend = ledIdxToButtonId(static_cast<uint8_t>(ledIdx));
  }

  const uint8_t targetType = PulsadoresHandler::getResponseTargetType();
  const TARGETNS targetNS = PulsadoresHandler::getResponseTargetNS();

  if (buttonColor == RELAY &&
      PulsadoresHandler::getButtonMappedFx(static_cast<uint8_t>(ledIdx)) ==
          (uint8_t)BTN_FX::RELAY_FX) {
    send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, targetType, targetNS,
                                         0x01));
    return;
  }

  // Enviar una trama de tipo RESPONSE (C5)
  send_frame(frameMaker_SEND_RESPONSE(DEFAULT_BOTONERA, targetType, targetNS,
                                   colorToSend));
}

// 4.3 Ambientes
void PulsadoresHandler::handleAmbientes(byte buttonColor,
                                        ButtonEventType event) {
  if (event != BUTTON_PRESSED)
    return;
  if (!selectedStates[currentIndex])
    return;

  uint8_t sendVal = (buttonColor == RELAY) ? BLACK : buttonColor;
  send_frame(frameMaker_SEND_PATTERN_NUM(DEFAULT_BOTONERA, BROADCAST, NS_ZERO,
                                         sendVal));
  delay(100);

  if (sendVal != BLACK) {
    doitPlayer.play_file(AMBIENTS_BANK, sendVal);
    ambienteActivo = true;
  } else {
    doitPlayer.stop_file();
    ambienteActivo = false;
  }
}

// ==========================================
// 5. LÓGICA DE RELÉS (3.6 y 3.7)
// ==========================================
void PulsadoresHandler::handleRelayLogic(byte buttonColor,
                                         ButtonEventType event,
                                         const ButtonContext &ctx) {

  // A) Multi-Relé / Aromaterapia (Colores)
  if (buttonColor != RELAY) {
    if (!ctx.isMultiRelay && !ctx.isAromaterapia)
      return; // Seguridad
    if (event != BUTTON_PRESSED)
      return;

    int idx = -1;
    if (buttonColor == BLUE)
      idx = 0;
    else if (buttonColor == GREEN)
      idx = 1;
    else if (buttonColor == YELLOW)
      idx = 2;
    else if (buttonColor == RED)
      idx = 3;

    if (idx >= 0) {
      uint8_t mask = (1 << idx);
      if (ctx.isAromaterapia) {
        // Exclusivo: si ya está, apaga; si no, solo ese.
        relay_state = (relay_state == mask) ? 0x00 : mask;
      } else {
        // Toggle independiente
        relay_state ^= mask;
      }
      // Feedback Color + Flag
      send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                       ctx.targetNS, buttonColor));
      delay(20);
      send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, ctx.targetType,
                                           ctx.targetNS, relay_state));
    }
    return;
  }

  // B) Relé Único (Botón RELAY)
  if (buttonColor == RELAY && ctx.hasRelay) {
    if (digitalRead(ENC_BUTTON) == LOW)
      return;
    if (ctx.targetType == DEFAULT_DEVICE && _isNSZero(ctx.targetNS))
      return;

    if (ctx.hasPulse) {
      // Momentáneo
      bool on = (event == BUTTON_PRESSED);
      send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, ctx.targetType,
                                           ctx.targetNS, on ? 0x01 : 0x00));
      RelayStateManager::set(ctx.targetNS, on);
    } else {
      // Latching (Toggle)
      if (event == BUTTON_PRESSED) {
        bool cur = RelayStateManager::get(ctx.targetNS);
        send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, ctx.targetType,
                                             ctx.targetNS, !cur ? 0x01 : 0x00));
        RelayStateManager::set(ctx.targetNS, !cur);
      }
    }
  }
}

// ==========================================
// 6. LÓGICA DE COLOR (3.1 - 3.5, 3.8, 3.9)
// ==========================================
void PulsadoresHandler::handleColorLogic(byte buttonColor,
                                         ButtonEventType event,
                                         const ButtonContext &ctx) {

  // ── COMUNICADOR: Detección de "Pasivo Machacado" ──
  // Si estamos en Comunicador + BroadcastPassive intacto y el usuario
  // pulsa un color, debemos reenviar START broadcast una sola vez
  // ANTES de mandar el color, para que los elementos salgan del Ambiente 9.
  if (ctx.currentFile == "Comunicador" && passiveModeActive && !passiveIsMashed
      && event == BUTTON_PRESSED && buttonColor != RELAY) {
    send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, START_CMD));
    delay(100);
    passiveIsMashed = true;
    // Continúa la ejecución normal para que el color se envíe abajo
  }

  // 3.8 Passive (Solo Blue)
  if (ctx.hasPassive && buttonColor != BLUE)
    return;

  // 3.9 Patterns
  if (ctx.hasPatterns) {
    if (event == BUTTON_PRESSED) {
      send_frame(frameMaker_SEND_PATTERN_NUM(DEFAULT_BOTONERA, ctx.targetType,
                                             ctx.targetNS, buttonColor));
    }
    return;
  }

  // --- CASO ESPECIAL: modo PASIVO sin flags de color ---
  // En modos donde sólo hay HAS_PASSIVE=1 y no hay color básico/avanzado,
  // queremos que el botón BLUE siga enviando su color.
  if (ctx.hasPassive && !ctx.hasBasic && !ctx.hasAdvanced) {
    if (event == BUTTON_PRESSED && buttonColor == BLUE) {
      send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                       ctx.targetNS, BLUE));
    }
    return;
  }
  // ------------------------------------------------------

  // 3.1 Sin capacidades de color
  if (!ctx.hasBasic && !ctx.hasAdvanced)
    return;

  // Selección de algoritmo
  if (ctx.hasBasic && !ctx.hasAdvanced) {
    // Básico
    if (ctx.hasPulse) {
      // Pulse: Gestionado en el Loop (applyBasicPulse)
      // Aquí no hacemos nada en evento, salvo quizás logging
    } else {
      // 3.2 Básico Latched
      if (event == BUTTON_PRESSED)
        applyBasicLatched(buttonColor, ctx);
    }
  } else if (ctx.hasAdvanced) {
    // Avanzado
    if (ctx.hasPulse) {
      // Pulse: Gestionado en el Loop (applyAdvancedPalettePulse)
    } else {
      // 3.4 Avanzado Palette (No Pulse)
      if (event == BUTTON_PRESSED)
        applyAdvancedPaletteNoPulse(buttonColor, ctx);
    }
  }
}

// --- 6.1 Básico Latched (3.2) ---
void PulsadoresHandler::applyBasicLatched(byte buttonColor,
                                          const ButtonContext &ctx) {
  send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                   ctx.targetNS, buttonColor));
}

// --- 6.2 Básico Pulse (3.3) ---
void PulsadoresHandler::applyBasicPulse(const ButtonContext &ctx) {
  static byte currentSent = BLACK;
  unsigned long maxTime = 0;
  byte winner = BLACK;
  bool anyPressed = false;

  // Buscar el botón pulsado más recientemente (max pressTime)
  for (int i = 0; i < FILAS; i++) {
    for (int j = 0; j < COLUMNAS; j++) {
      if (pulsadorColor[i][j] == RELAY)
        continue;
      if (pressTime[i][j] > 0) {
        anyPressed = true;
        if (pressTime[i][j] >= maxTime) {
          maxTime = pressTime[i][j];
          winner = pulsadorColor[i][j];
        }
      }
    }
  }

  if (anyPressed) {
    if (winner != currentSent) {
      send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                       ctx.targetNS, winner));
      currentSent = winner;
    }
  } else {
    if (currentSent != BLACK) {
      send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                       ctx.targetNS, BLACK));
      currentSent = BLACK;
    }
  }
}

// --- 6.3 Avanzado Palette No Pulse (3.4) ---
// Persistente, toggles, max 2.
void PulsadoresHandler::applyAdvancedPaletteNoPulse(byte buttonColor,
                                                    const ButtonContext &ctx) {
  static std::vector<byte> palette;
  static int lastMode = -1;

  // Reset si es llamada de limpieza
  if (buttonColor == BLACK) {
    palette.clear();
    lastMode = currentModeIndex;
    return;
  }
  if (currentModeIndex != lastMode) {
    palette.clear();
    lastMode = currentModeIndex;
  }

  // Lógica Paleta
  auto it = std::find(palette.begin(), palette.end(), buttonColor);

  if (it != palette.end()) {
    // Ya estaba: Quitar (Toggle OFF)
    palette.erase(it);
  } else {
    // No estaba
    if (palette.size() < 2) {
      // Cabe: Añadir
      palette.push_back(buttonColor);
    } else {
      // Lleno: Reemplazo total (3a pulsación distinto color)
      palette.clear();
      palette.push_back(buttonColor);
    }
  }

  // Calcular Salida
  byte output = BLACK;
  if (palette.empty())
    output = BLACK;
  else if (palette.size() == 1)
    output = palette[0];
  else if (palette.size() == 2) {
    colorHandler.color_mix_handler(palette[0], palette[1], &output);
  }

  send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                   ctx.targetNS, output));
}

// --- 6.4 Avanzado Palette Pulse (3.5) ---
// Basado en botones pulsados físicamente (pressTime > 0)
void PulsadoresHandler::applyAdvancedPalettePulse(const ButtonContext &ctx) {
  static byte currentSent = BLACK;
  static std::vector<byte> activeBtns; // Botones físicamente pulsados ahora

  // Recolectar pulsados (preservando orden si fuera necesario, o simplemente
  // set) Para simplificar, recorremos y llenamos.
  std::vector<byte> currentPressed;
  int count = 0;
  // Usamos orden de pressTime para determinar cuáles son los "últimos 2" si
  // hubiera más de 2 dedos (caso raro) Pero la especificación dice: max 2 en
  // paleta. Si pulsas 3, la paleta se renueva al 3º. Como esto corre en Loop
  // continuo, es complejo detectar el "evento" de 3º pulsación aquí.
  // Simplificación Robustas: Si hay >2 pulsados, tomamos los 2 con mayor
  // pressTime.

  struct BtnInfo {
    byte col;
    unsigned long t;
  };
  std::vector<BtnInfo> pressedList;

  for (int i = 0; i < FILAS; i++) {
    for (int j = 0; j < COLUMNAS; j++) {
      if (pulsadorColor[i][j] != RELAY && pressTime[i][j] > 0) {
        pressedList.push_back({pulsadorColor[i][j], pressTime[i][j]});
      }
    }
  }

  // Ordenar por tiempo (más recientes al final)
  std::sort(pressedList.begin(), pressedList.end(),
            [](const BtnInfo &a, const BtnInfo &b) { return a.t < b.t; });

  // Lógica de Paleta Física
  // Si hay 0 -> Black
  // Si hay 1 -> Color
  // Si hay 2 -> Mix
  // Si hay >2 -> Tomamos los 2 últimos (más recientes) -> Mix

  byte output = BLACK;
  if (pressedList.empty()) {
    output = BLACK;
  } else if (pressedList.size() == 1) {
    output = pressedList[0].col;
  } else {
    // Tomar los dos últimos
    byte c1 = pressedList[pressedList.size() - 2].col;
    byte c2 = pressedList[pressedList.size() - 1].col;
    colorHandler.color_mix_handler(c1, c2, &output);
  }

  // Nota: La detección de "Pasivo Machacado" del Comunicador se realiza
  // en handleColorLogic() antes de llegar aquí. No duplicar lógica.

  if (output != currentSent) {
    send_frame(frameMaker_SEND_COLOR(DEFAULT_BOTONERA, ctx.targetType,
                                     ctx.targetNS, output));
    currentSent = output;
  }
}

// ==========================================
// 7. LÓGICA COMPLEJA: Ciclo Comunicador
// ==========================================
// Máquina de estados:
//   BroadcastNormal --relay--> ElementCycle(elem0)
//   ElementCycle(elemK) --relay--> ElementCycle(elemK+1)    [si K < N-1]
//   ElementCycle(elemN) --relay--> BroadcastPassive
//   BroadcastPassive --relay--> BroadcastNormal             [si intacto]
//   BroadcastPassive --color--> BroadcastPassiveOverridden  [primer color]
//   BroadcastPassiveOverridden --relay--> ElementCycle(elem0)
// ==========================================
void PulsadoresHandler::handleComunicadorRelayCycle() {
  const unsigned D = 100; // delay entre tramas consecutivas

  // --- Helpers locales ---
  auto isZero = [](const TARGETNS &n) { return memcmp(&n, &NS_ZERO, sizeof(TARGETNS)) == 0; };
  auto eqNS   = [](const TARGETNS &a, const TARGETNS &b) { return memcmp(&a, &b, sizeof(TARGETNS)) == 0; };
  auto cmpNS  = [](const TARGETNS &a, const TARGETNS &b) {
    if (a.mac01 != b.mac01) return a.mac01 < b.mac01;
    if (a.mac02 != b.mac02) return a.mac02 < b.mac02;
    if (a.mac03 != b.mac03) return a.mac03 < b.mac03;
    if (a.mac04 != b.mac04) return a.mac04 < b.mac04;
    return a.mac05 < b.mac05;
  };

  // --- Construir lista de elementos reales (NS válidos) ---
  std::vector<TARGETNS> nsList;
  nsList.reserve(elementFiles.size());
  for (const String &f : elementFiles) {
    if (f == "Ambientes" || f == "Fichas" || f == "Comunicador" || f == "Apagar") continue;
    TARGETNS ns = getNSFromFile(f);
    if (!isZero(ns)) nsList.push_back(ns);
  }
  std::sort(nsList.begin(), nsList.end(), cmpNS);
  if (nsList.empty()) return;

  // --- Sincronizar relayStep con communicatorActiveNS ---
  if (!_isNSZero(::communicatorActiveNS)) {
    bool found = false;
    for (size_t k = 0; k < nsList.size(); ++k) {
      if (eqNS(nsList[k], ::communicatorActiveNS)) {
        relayStep = (int)k;
        found = true;
        break;
      }
    }
    if (!found) relayStep = -1;
  }
  if (relayStep < -1 || relayStep >= (int)nsList.size()) relayStep = -1;

  // --- Helper: encender un elemento con ON flag si tiene relé ---
  auto startElement = [&](uint8_t tType, const TARGETNS &tNS) {
    send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, tType, tNS, START_CMD));
    delay(D);
    bool sendFlag = (tType == BROADCAST);
    if (!sendFlag) {
      uint8_t wCfg[2];
      if (RelayStateManager::getModeConfigForNS(tNS, wCfg)) {
        if (getModeFlag(wCfg, HAS_RELAY)) sendFlag = true;
      }
    }
    if (sendFlag) {
      send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, tType, tNS, 0x01));
      delay(D);
    }
  };

  // ==========================================================
  // TRANSICIONES
  // ==========================================================

  if (relayStep == -1) {
    // Estamos en algún estado broadcast (Normal, Pasivo o PasivoMachacado)

    if (!passiveModeActive) {
      // ── ESTADO 1: BroadcastNormal → ElementCycle(elem0) ──
      send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, BLACKOUT));
      delay(D);
      startElement(DEFAULT_DEVICE, nsList[0]);
      relayStep = 0;
      ::communicatorActiveNS = nsList[0];

    } else if (!passiveIsMashed) {
      // ── ESTADO 3: BroadcastPassive intacto → BroadcastNormal ──
      // Solo re-enviar START broadcast sin ambiente 9
      send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, START_CMD));
      delay(D);
      send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, 0x01));
      delay(D);
      passiveModeActive = false;
      relayStep = -1;
      ::communicatorActiveNS = NS_ZERO;

    } else {
      // ── ESTADO 4: BroadcastPassiveOverridden → ElementCycle(elem0) ──
      send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, BLACKOUT));
      delay(D);
      startElement(DEFAULT_DEVICE, nsList[0]);
      relayStep = 0;
      passiveModeActive = false;
      passiveIsMashed = false;
      ::communicatorActiveNS = nsList[0];
    }

  } else if (relayStep < (int)nsList.size() - 1) {
    // ── ESTADO 2: ElementCycle intermedio → siguiente elemento ──
    send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, DEFAULT_DEVICE, nsList[relayStep], BLACKOUT));
    delay(D);
    relayStep++;
    startElement(DEFAULT_DEVICE, nsList[relayStep]);
    ::communicatorActiveNS = nsList[relayStep];

  } else {
    // ── ESTADO 2: ElementCycle último → BroadcastPassive ──
    // NO apagar el último elemento; enviar START broadcast + Ambiente 9
    send_frame(frameMaker_SEND_COMMAND(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, START_CMD));
    delay(D);
    send_frame(frameMaker_SEND_FLAG_BYTE(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, 0x01));
    delay(D);
    send_frame(frameMaker_SEND_PATTERN_NUM(DEFAULT_BOTONERA, BROADCAST, NS_ZERO, 0x09));
    delay(D);
    relayStep = -1;
    passiveModeActive = true;
    passiveIsMashed = false;
    ::communicatorActiveNS = NS_ZERO;
  }

  colorHandler.setCurrentFile("Comunicador");
  colorHandler.setPatternBotonera(currentModeIndex, ledManager);
}

// NUEVO: numColor recibido en F_SET_BUTTONS_EXTMAP para cada LED
uint8_t PulsadoresHandler::s_btnMappedNumColor[9] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t PulsadoresHandler::s_btnMappedFx[9] = {
    (uint8_t)BTN_FX::NO_FX, (uint8_t)BTN_FX::NO_FX, (uint8_t)BTN_FX::NO_FX,
    (uint8_t)BTN_FX::NO_FX, (uint8_t)BTN_FX::NO_FX, (uint8_t)BTN_FX::NO_FX,
    (uint8_t)BTN_FX::NO_FX, (uint8_t)BTN_FX::NO_FX, (uint8_t)BTN_FX::NO_FX};

void PulsadoresHandler::setButtonMappedNumColor(uint8_t ledIdx,
                                                uint8_t numColor) {
  if (ledIdx < 9) {
    s_btnMappedNumColor[ledIdx] = numColor;
  }
}

uint8_t PulsadoresHandler::getButtonMappedNumColor(uint8_t ledIdx) {
  return (ledIdx < 9) ? s_btnMappedNumColor[ledIdx] : 0xFF;
}

void PulsadoresHandler::setButtonMappedFx(uint8_t ledIdx, uint8_t fx) {
  if (ledIdx < 9) {
    s_btnMappedFx[ledIdx] = fx;
  }
}

uint8_t PulsadoresHandler::getButtonMappedFx(uint8_t ledIdx) {
  return (ledIdx < 9) ? s_btnMappedFx[ledIdx] : (uint8_t)BTN_FX::NO_FX;
}
