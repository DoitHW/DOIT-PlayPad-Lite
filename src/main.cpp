#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include <ADXL345_handler/ADXL345_handler.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Colors_DMS/Color_DMS.h>
#include <DynamicLEDManager_DMS/DynamicLEDManager_DMS.h>
#include <ESP32Encoder.h>
#include <Element_DMS/Element_DMS.h>
#include <FS.h>
#include <Frame_DMS/Frame_DMS.h>
#include <Pulsadores_handler/Pulsadores_handler.h>
#include <RelayManager_DMS/RelayStateManager.h>
#include <SPIFFS.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <botonera_DMS/botonera_DMS.h>
#include <defines_DMS/defines_DMS.h>
#include <display_handler/display_handler.h>
#include <driver/i2s.h>
#include <encoder_handler/encoder_handler.h>
#include <icons_64x64_DMS/icons_64x64_DMS.h>
#include <microphone_DMS/microphone_DMS.h>
#include <play_DMS/play_DMS.h>
#include <string.h>
#include <token_DMS/token_DMS.h>
#include <vector>

constexpr uint32_t SLEEP_AFTER_MS = 60000 * 5; // 5 minutos
const uint32_t DISPLAY_OFF_MS = 15000;

constexpr uint8_t FIRMWARE_VERSION = 0x01; // ->  VERSIÓN de placa.     max=0xFF
constexpr uint8_t BUILD_MONTH = 01;        // ->  MES de fabricación.   max=255
constexpr uint16_t SERIAL_NUM =
    (static_cast<uint16_t>(BUILD_MONTH) << 8) | FIRMWARE_VERSION;

bool ambienteActivo = false;

void updateBatteryStatus();
void initBatteryADC();
float readVBat();

PulsadoresHandler pulsadores;
CRGB leds[9];
void useMicrophone();
void useAccelerometer();

void showSerial();

static void enterOtaModeBlocking(BOTONERA_ *element);
static bool isBootOtaRequestedByEncoderBtn();

byte xManager = BROADCAST;
BOTONERA_ element_;
BOTONERA_ *element = &element_;
DOITSOUNDS_ doitPlayer;
COLORHANDLER_ colorHandler; // si lo manejas como global, o extern...
DynamicLEDManager ledManager(colorHandler);
TOKEN_ token;
INFO_PACK_T fichasOption;
INFO_PACK_T ambientesOption;
INFO_PACK_T apagarSala;
INFO_PACK_T comunicadorOption;

INFO_PACK_T elemento;
bool adxlActiveBeforeModesScreen = false;
bool wasInModesScreen = false;

#ifdef MIC
MICROPHONE_ doitMic;
uint8_t micSilenceThreshold = 5;
#endif

// // --- Medición de batería / divisor ---
// constexpr float R_TOP_OHMS    = 147000.0f;   // VBAT -> pin
// constexpr float R_BOTTOM_OHMS = 100000.0f;   // pin  -> GND
// constexpr float DIV_RATIO     = (R_BOTTOM_OHMS / (R_TOP_OHMS +
// R_BOTTOM_OHMS)); // ≈ 0.4049 constexpr float VBAT_CAL_K    = 1.091f;       //
// Factor de calibración fino (Vreal/Vlog)

// --- Medición de batería / divisor (nominal, sólo informativo) ---
constexpr float R_TOP_OHMS = 147000.0f;    // VBAT -> pin
constexpr float R_BOTTOM_OHMS = 100000.0f; // pin  -> GND
constexpr float DIV_RATIO =
    (R_BOTTOM_OHMS / (R_TOP_OHMS + R_BOTTOM_OHMS)); // ≈ 0.4049

// Ya NO usamos VBAT_CAL_K dentro de readVBat para escalar.
// Lo dejamos a 1.0f por si en algún momento lo quieres usar en otro sitio.
constexpr float VBAT_CAL_K = 1.0f;

// Calibración directa V_ADC(pin) → VBAT real
// Datos de medida:
//   VBAT_REAL ≈ 3.68 V
//   VBAT_log_actual ≈ 2.475 V
//   Reconstruyendo V_ADC_fw ≈ 1.378 V
//   GAIN = VBAT_REAL / V_ADC_fw ≈ 3.68 / 1.378 ≈ 2.67
constexpr float VBAT_ADC_TO_BAT_GAIN = 2.67f;

// --- ADC / filtro ---
static esp_adc_cal_characteristics_t adc_chars;
const int PIN_BAT_ADC = 10;
constexpr float ALPHA = 0.05f; // IIR batería

// --- Filtro de mediana ---
#define MEDIAN_FILTER_SIZE 5
static float vBatWindow[MEDIAN_FILTER_SIZE];
static uint8_t vBatIdx = 0;
static bool medianInit = false;

// --- Tabla tensión → SoC (ajusta a tu celda si quieres afinar) ---
static const float voltajes[] = {3.00f, 3.40f, 3.70f, 3.85f, 4.00f, 4.20f};
static const int socTable[] = {0, 10, 25, 50, 75, 100};
static const uint8_t N_SOC = sizeof(socTable) / sizeof(socTable[0]);

// --- Estado de carga ---
enum ChargeState { NOT_CHARGING, CHARGING, CHARGED };
ChargeState chargeState = NOT_CHARGING;

// Porcentaje que se usa SOLO para dibujo (200=enchufado/cargando)
constexpr float BAT_VIS_CHARGING = 200.0f;
float batteryVisualPercentage = 100.0f;
constexpr float BATTERY_WARN_THRESHOLD = 15.0f; // Aviso sonoro
constexpr float BATTERY_LOCK_THRESHOLD = 5.0f;  // Bloqueo crítico
constexpr float BATTERY_UNLOCK_THRESHOLD =
    7.0f; // Desbloqueo tras carga o recuperación

// Variables ya existentes en tu sketch (se respetan)
float vBatFiltered;
unsigned long lastBatteryCheck;
unsigned long lastBatteryWarningTime;
bool criticalBatteryLock;
float batteryPercentage;
int lastBatteryIconIndex = -1;
extern bool displayOn;
extern bool isInMainMenu();
extern void drawCurrentElement();
extern void showCriticalBatteryMessage();

struct BatteryIconThreshold {
  float enterThreshold;
  float exitThreshold;
  int iconIndex;
};
extern const BatteryIconThreshold batteryLevels[];

// ==== Helpers exclusivos para setup() (mismo archivo) =======================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

// ---- Constantes ------------------------------------------------------------
constexpr uint32_t kPostInitDelayMs = 100;
constexpr uint32_t kAudioWaitTimeoutMs = 1500;  // límite para no bloquear
constexpr uint8_t kUiMaxMappedBrightness = 165; // 0..99 → 0..165
constexpr uint8_t kFullBrightnessValue = 255;   // cuando UI=100
constexpr uint32_t kMaxWelcomeDurationMs =
    6000; // tope para bienvenida paralela

// ---- Estado para coordinar bienvenida paralela -----------------------------
static volatile bool s_welcomeLedsDone = false;
static volatile bool s_welcomeDisplayDone = false;

// ---- Brillo UI% → FastLED ---------------------------------------------------
inline void applyLedBrightness(uint8_t uiPercent) {
  FastLED.setBrightness((uiPercent >= 100)
                            ? kFullBrightnessValue
                            : map(uiPercent, 0, 99, 0, kUiMaxMappedBrightness));
}

// 1) UART y RF
inline void initUARTyRF() {
  element->begin();
  delay(kPostInitDelayMs);
  WiFi.mode(WIFI_OFF);
  uartBuffer.clear();
  uartBuffer.reserve(MAX_FRAME_LENGTH);
}

// 2) Preferencias de UI (solo brillo)
inline void cargarBrillo() { currentBrightness = loadBrightnessFromSPIFFS(); }

#ifdef MIC
inline void cargarMicCalib() {
  micSilenceThreshold = loadMicSilenceThresholdFromSPIFFS();
}
#endif

// 3) Periféricos del sistema SIN entradas (sensores + LEDs + display + audio)
inline void initPerifericos() {
#ifdef MIC
  doitMic.begin(); // I2S MIC
  delay(100);
#endif

#ifdef ADXL
  adxl345Handler.init();                  // ADXL345 init
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN); // INT1 del ADXL debe estar en reposo
                                          // LOW (la INT es activa en HIGH)
  adxl345Handler.enableActivityInterrupt(
      600 /*mg*/, true, true, true); // Umbral 600 mg y ejes X/Y/Z activos
  adxl345Handler.clearInterrupts();  // Limpia cualquier latch previo de la INT
                                     // antes de empezar
#endif

#ifdef NFC
  token.begin(); // PN532 (solo init; IRQ después)
#endif

  colorHandler.begin(NUM_LEDS);          // FastLED init (core 1)
  applyLedBrightness(currentBrightness); // brillo ANTES de animar

#ifdef PLAYER
  doitPlayer.begin(); // MP3: inicia y reproduce bienvenida
#endif
  display_init(); // TFT listo antes de animar
}

// 4) Batería (ADC + primera lectura)
inline void initBateria() {
  initBatteryADC();
  vBatFiltered = readVBat();
}

// 5) Estado persistente (SPIFFS): elementos, modos, bancos
inline void cargarElementos() {
  initializeDynamicOptions();
  loadElementsFromSPIFFS();
  setAllElementsToBasicMode();

  bankList = readBankList();
  selectedBanks.resize(bankList.size(), false);

#ifdef DEBUG
  DEBUG__________ln("Banks:");
  for (size_t i = 0; i < bankList.size(); ++i) {
    DEBUG__________(bankList[i], HEX);
    DEBUG__________(" ");
  }
  DEBUG__________ln("");
#endif
}

// 6) Ajustes de sonido DESPUÉS de begin() del player
inline void aplicarSonido() { loadSoundSettingsFromSPIFFS(); }

// 7) Idioma (o abre menú y termina setup)
inline bool asegurarIdioma() {
  currentLanguage = loadLanguageFromSPIFFS();
  if (static_cast<byte>(currentLanguage) == 0xFF) {
    languageMenuActive = true;
    languageMenuSelection = 0;
    currentLanguage = Language::EN; // temporal para pintar
    drawLanguageMenu(languageMenuSelection);
    return false;
  }
  return true;
}

// 8) Espera acotada a que el audio quede inactivo (si procede)
inline void esperarAudioInactivoHasta() {
#ifdef PLAYER
  if (doitPlayer.player.readState() == 513) {
    DEBUG__________ln("Esperando a que termine el audio (máx 1.5s)...");
    const uint32_t t0 = millis();
    while (doitPlayer.player.readState() == 513 &&
           (millis() - t0) < kAudioWaitTimeoutMs) {
      delay(10);
    }
  }
#endif
}

// 9) IRQ del PN532 (solo si NFC está presente)
inline void configurarIRQ_NFC() {
#ifdef NFC
  pinMode(PN532_IRQ, INPUT_PULLUP);
#endif
}

// 10) ENTRADAS: pulsadores y encoder (después de bienvenida)
inline void initPulsadoresYEncoder() {
  pulsadores.begin();
  encoder_init_func();
}

// ---- Tareas FreeRTOS para bienvenida en paralelo ---------------------------
// IMPORTANTE: FastLED.show() durante la bienvenida en el MISMO CORE de begin()
// (core 1).
void taskWelcomeLeds(void * /*pv*/) {
  colorHandler.welcomeEffect(); // bloqueante en su propia tarea
  s_welcomeLedsDone = true;
  vTaskDelete(nullptr);
}

void taskWelcomeDisplay(void * /*pv*/) {
  showWelcomeAnimation(); // bloqueante en su propia tarea
  s_welcomeDisplayDone = true;
  vTaskDelete(nullptr);
}

// 11) Lanzar bienvenida paralela (LEDs + Display). El audio ya va en paralelo.
inline void efectoBienvenida() {
  s_welcomeLedsDone = false;
  s_welcomeDisplayDone = false;

#if CONFIG_FREERTOS_UNICORE
  xTaskCreate(taskWelcomeLeds, "WelcomeLEDs", 4096, nullptr, 1, nullptr);
  xTaskCreate(taskWelcomeDisplay, "WelcomeDisplay", 4096, nullptr, 1, nullptr);
#else
  // LED task en CORE 1 (mismo que FastLED/init); TFT en CORE 0
  xTaskCreatePinnedToCore(taskWelcomeLeds, "WelcomeLEDs", 4096, nullptr, 1,
                          nullptr, 1);
  xTaskCreatePinnedToCore(taskWelcomeDisplay, "WelcomeDisplay", 4096, nullptr,
                          1, nullptr, 0);
#endif
}

// 12) Esperar fin bienvenida (o timeout). NO limpiar LEDs aquí.
inline void esperarFinEfectosBienvenida() {
  const uint32_t t0 = millis();
  while (!(s_welcomeLedsDone && s_welcomeDisplayDone) &&
         (millis() - t0) < kMaxWelcomeDurationMs) {
    delay(10);
  }
  // No clear/show aquí: dejar como quedó la bienvenida.
}

// === NUEVO ===
// Lee el “modo real” del elemento (RAM para fijos; SPIFFS para ficheros)
inline int leerModoActualElemento(const String &file) {
  if (file == "Ambientes")
    return ambientesOption.currentMode;
  else if (file == "Fichas")
    return fichasOption.currentMode;
  else if (file == "Comunicador")
    return comunicadorOption.currentMode;
  else if (file == "Apagar")
    return apagarSala.currentMode;
  else if (file == "Dado")
    return dadoOption.currentMode;

  // SPIFFS: byte en OFFSET_CURRENTMODE
  fs::File f = SPIFFS.open(file, "r");
  if (f) {
    f.seek(OFFSET_CURRENTMODE, SeekSet);
    int idx = f.read(); // 0..15
    f.close();
    if (idx >= 0 && idx < 16)
      return idx;
  }
  return 0; // por defecto
}

// === NUEVO ===
// Aplica patrón LED del elemento actualmente enfocado y fuerza primer frame
// dinámico
inline void aplicarPatronInicialFocoActual() {
  if (currentIndex < 0 || currentIndex >= (int)elementFiles.size())
    return;
  const String &path = elementFiles[currentIndex];

  // 1) Indica a COLORHANDLER qué “archivo” está activo
  colorHandler.setCurrentFile(path);

  // 2) Obtiene el modo actual (RAM/SPIFFS) y aplica patrón
  const int realModeIndex = leerModoActualElemento(path);
  colorHandler.setPatternBotonera(realModeIndex, ledManager);

  // 3) Fuerza primer frame de efectos dinámicos (Fade, etc.) ya mismo
  ledManager.update();
}

} // namespace
// ================= NUEVO DUAL CORE =================
struct SensorState {
  bool isCurrentElementSelected;
  bool isInMainMenu;
  bool isStandardMode;
  TARGETNS targetNS;
  bool hasBinarySensors;
  bool hasMic;
  bool hasAdxl;
  bool hasRelay;
  bool hasRelayN1;
  bool hasRelayN2;
};
SensorState g_sensorState;
SemaphoreHandle_t g_stateMutex = NULL;
SemaphoreHandle_t uartTxMutex = NULL;

extern bool awaitingResponse;

void sensorTask(void *pv) {
  static bool localUseMic = false;
  static bool localUseAdxl = false;

  SensorState localState;

  for (;;) {
    if (g_stateMutex) {
      xSemaphoreTake(g_stateMutex, portMAX_DELAY);
      localState = g_sensorState;
      xSemaphoreGive(g_stateMutex);
    }

#if defined(MIC) || defined(ADXL)
    // Comprobación de cambio de estado o falta de inicialización
    if ((localState.hasMic != localUseMic) ||
        (localState.hasAdxl != localUseAdxl) ||
        (localState.hasMic && !doitMic.isActive()) ||
        (localState.hasAdxl && !adxl345Handler.isInitialized())) {

#ifdef MIC
      if (localState.hasMic) {
        if (!doitMic.isActive() || localState.hasMic != localUseMic) {
          DEBUG__________ln("🎤 Activando/Reconfirmando micrófono.");
#ifdef ADXL
          if (localUseAdxl) {
            adxl345Handler.end();
            delay(100);
          }
#endif
          gpio_reset_pin((gpio_num_t)47);
          gpio_reset_pin((gpio_num_t)48);
          delay(50);
          useMicrophone();
          localUseMic = true;
          localUseAdxl = false;
        }
      }
#endif

#ifdef ADXL
      if (localState.hasAdxl) {
        if (!adxl345Handler.isInitialized() || localState.hasAdxl != localUseAdxl) {
          DEBUG__________ln("📏 Activando/Reconfirmando acelerómetro.");
#ifdef MIC
          if (localUseMic) {
            doitMic.end();
            delay(100);
          }
#endif
          gpio_reset_pin((gpio_num_t)47);
          gpio_reset_pin((gpio_num_t)48);
          pinMode(47, INPUT_PULLUP);
          pinMode(48, INPUT_PULLUP);
          delay(50);
          useAccelerometer();
          localUseMic = false;
          // Solo marcamos como activo si realmente se inicializó
          localUseAdxl = adxl345Handler.isInitialized();
          if (!localUseAdxl) {
            DEBUG__________ln("⚠️ ADXL no se inicializó, se reintentará.");
          }
        }
      }
#endif

      if (!localState.hasMic && !localState.hasAdxl) {
        if (localUseMic || localUseAdxl) {
#ifdef MIC
          if (localUseMic) {
            doitMic.end();
            delay(100);
          }
#endif
#ifdef ADXL
          if (localUseAdxl) {
            adxl345Handler.end();
            delay(100);
          }
#endif
          gpio_reset_pin((gpio_num_t)47);
          gpio_reset_pin((gpio_num_t)48);
          localUseMic = false;
          localUseAdxl = false;
        }
      }
    }

#ifdef MIC
    if (localUseMic) {
      if (localState.isCurrentElementSelected) {
        TARGETNS ns = localState.targetNS;
        if (!NS_is_zero(ns)) {
          const unsigned long micInterval = 50;
          static unsigned long lastMicCheck = 0;
          static int falseCount = 0;
          static bool lastSentState = false;
          static unsigned long lastSendTime = 0;
          static byte micValueAnterior = 0;
          const byte DELTA_THRESHOLD = 2;

          SENSOR_VALUE_T value;
          if (localState.hasBinarySensors) {
            value.msb_min = 0x00;
            value.lsb_min = 0x00;
            value.msb_max = 0x00;
            value.lsb_max = 0x01;

            extern int micSensRuntime;
            const uint8_t rawValue = doitMic.get_mic_value_BYTE(micSensRuntime);
            const uint8_t threshold = micSilenceThreshold;
            bool sonido = (rawValue >= threshold);

            // Teleplot Debug
            Serial.printf(">mic_val:%d\n", rawValue);
            Serial.printf(">mic_threshold:%d\n", threshold);
            Serial.printf(">mic_sent:%d\n", sonido ? 255 : 0);

            if (sonido) {
              falseCount = 0;
              if (!awaitingResponse && !lastSentState &&
                  (millis() - lastSendTime >= 300)) {
                value.msb_val = 0x00;
                value.lsb_val = 0x01;
                send_frame(frameMaker_SEND_SENSOR_VALUE_2(
                    DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                lastSendTime = millis();
                lastSentState = true;
              }
            } else {
              falseCount++;
              int relayCount =
                  (localState.hasRelayN1 << 1) | localState.hasRelayN2;
              relayCount = (relayCount == 0 && localState.hasRelay)
                               ? 1
                               : relayCount + (localState.hasRelay ? 1 : 0);
              int maxretry = (relayCount > 0) ? 5 : 3;

              if (falseCount == maxretry && lastSentState) {
                value.msb_val = 0x00;
                value.lsb_val = 0x00;
                send_frame(frameMaker_SEND_SENSOR_VALUE_2(
                    DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                lastSendTime = millis();
                lastSentState = false;
              }
            }
          } else {
            value.msb_min = 0x00;
            value.lsb_min = 0x00;
            value.msb_max = 0x00;
            value.lsb_max = 0xFF;

            if (millis() - lastMicCheck >= micInterval) {
              lastMicCheck = millis();

              extern int micSensRuntime;
              const uint8_t rawValue =
                  doitMic.get_mic_value_BYTE(micSensRuntime);
              Serial.printf(">mic_val:%d\n", rawValue);
              const uint8_t threshold = micSilenceThreshold;

              constexpr uint32_t SILENCE_HOLD_MS = 100;
              static uint32_t belowSinceMs = 0;
              static bool inSilence = true;

              if (rawValue >= threshold) {
                belowSinceMs = 0;
                if (inSilence) {
                  inSilence = false;
                  micValueAnterior = rawValue;
                  value.msb_val = 0x00;
                  value.lsb_val = rawValue;
                  doitMic.updateSoundTime();

                  if (!awaitingResponse) {
                    send_frame(frameMaker_SEND_SENSOR_VALUE_2(
                        DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                    lastSendTime = millis();
                  }
                } else {
                  doitMic.updateSoundTime();
                  if (abs((int)rawValue - (int)micValueAnterior) >=
                      (int)DELTA_THRESHOLD) {
                    micValueAnterior = rawValue;
                    value.msb_val = 0x00;
                    value.lsb_val = rawValue;

                    if (!awaitingResponse) {
                      send_frame(frameMaker_SEND_SENSOR_VALUE_2(
                          DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                      lastSendTime = millis();
                    }
                  } else if (millis() - lastSendTime > 200) {
                    if (!awaitingResponse) {
                      value.msb_val = 0x00;
                      value.lsb_val = rawValue;
                      send_frame(frameMaker_SEND_SENSOR_VALUE_2(
                          DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                      lastSendTime = millis();
                    }
                  }
                }
              } else {
                if (belowSinceMs == 0)
                  belowSinceMs = millis();
                if (!inSilence &&
                    (millis() - belowSinceMs) >= SILENCE_HOLD_MS) {
                  inSilence = true;
                  micValueAnterior = 0;
                  value.msb_val = 0x00;
                  value.lsb_val = 0x00;

                  if (!awaitingResponse) {
                    send_frame(frameMaker_SEND_SENSOR_VALUE_2(
                        DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                    lastSendTime = millis();
                  }
                }
              }
            }
          }
        }
      }
    }
#endif // MIC

#ifdef ADXL
    if (localUseAdxl) {
      if (localState.isCurrentElementSelected && localState.isInMainMenu) {
        if (localState.isStandardMode) {
          adxl345Handler.readInclinations(localState.hasBinarySensors,
                                          localState.targetNS);
        }
      }
    }
#endif // ADXL

#endif // MIC || ADXL

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ===========================================================================

void setup() {
  // if (isBootOtaRequestedByEncoderBtn()) {
  //   enterOtaModeBlocking(element);
  // }
  pinMode(BOTONERA_DATA_PIN, OUTPUT);
  digitalWrite(BOTONERA_DATA_PIN, LOW);
  delay(5); // pequeño margen
  // 1) UART + RF
  initUARTyRF();

  // 2) Preferencias de UI (brillo)
  cargarBrillo();

#ifdef MIC
  cargarMicCalib();
#endif

  // 3) Periféricos sin entradas (MIC/ADXL/NFC/LEDs/Display/Player)
  initPerifericos();

  // 4) Bienvenida en paralelo: LEDs (core 1) + pantalla (core 0)
  efectoBienvenida();

  // 5) Batería
  initBateria();

  // 6) Estado persistente (elementos, modos, bancos)
  cargarElementos();

  // 7) Ajustes de sonido (ya existe el player)
  aplicarSonido();

  // 8) Esperar fin de bienvenida (NO reinit, NO clear)
  esperarFinEfectosBienvenida();

  // 9) **Habilitar entradas ANTES de comprobar idioma (clave para evitar
  // reinicios)**
  initPulsadoresYEncoder();

  // 10) Idioma (si no hay, mostrar menú y salir)
  if (!asegurarIdioma())
    return;

  // 11) Espera acotada por audio (si aún suena)
  esperarAudioInactivoHasta();

  // 12) NFC: configurar IRQ
  configurarIRQ_NFC();

  // 13) Serial / NS de la consola
  showSerial();

  // 14) Enfoque inicial
  enfocarElemento("Comunicador");

  // 15) Aplicar patrón LED del foco inicial + primer frame dinámico
  aplicarPatronInicialFocoActual();

  // 16) Pintado inicial de la UI
  drawCurrentElement();

  // 17) Iniciar Dual Core Sensor Task
  g_stateMutex = xSemaphoreCreateMutex();
  uartTxMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 5, NULL, 0);
}

bool hiddenMenuActive = false;
bool adxl = false;
bool useMic = false;
bool lastUseMic = !useMic;
bool lastUseAdxl = !adxl;
static bool micWasActiveBeforeSleep = false;

int irqCurr = HIGH;
int irqPrev = HIGH;
bool cardIsRead = false;
unsigned long cardRemovalStart =
    0; // Momento en que se detecta la retirada (IRQ HIGH)

extern bool bankSelectionActive;
extern String confirmedFileToDelete;

static bool adxl_sleeping = false;

int micSensRuntime = MIC_SENS;

// Diagnóstico simple de por qué aborta dormir (solo ADXL)
static bool adxlIntActiveNow() {
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN); // reposo en LOW; INT activa = HIGH
  int v = digitalRead(INT1_ADXL_PIN);
  if (v == HIGH) {
    Serial.println("Sleep abort: ADXL INT1 is HIGH (latched/ruido).");
    return true;
  }
  return false;
}

// ─── Brillo guardado y utilidades
// ─────────────────────────────────────────────
static uint8_t s_savedBrightnessPct = 100;

static uint8_t percentToFastLED(uint8_t pct) {
  if (pct >= 100)
    return 255;
  return map(pct, 0, 99, 0, 165); // respeta tu curva 0..99 → 0..165
}

// Baja el brillo actual en un "fade" hasta targetPct (p.ej. 20%)
static void rampBrightnessToPercent(uint8_t targetPct, uint8_t steps = 10,
                                    uint16_t stepDelayMs = 15) {
  extern uint8_t currentBrightness; // ya la usas en tu proyecto
  uint8_t from = currentBrightness;
  if (from <= targetPct) {
    // si ya es <=, fuerza directamente
    currentBrightness = targetPct;
    FastLED.setBrightness(percentToFastLED(targetPct));
    FastLED.show();
    return;
  }
  int delta = (int)from - (int)targetPct;
  for (uint8_t i = 1; i <= steps; ++i) {
    uint8_t pct = from - (delta * i) / steps;
    FastLED.setBrightness(percentToFastLED(pct));
    FastLED.show();
    delay(stepDelayMs);
  }
  currentBrightness = targetPct; // mantenemos coherencia con tu % almacenado
}

// Filas LOW y columnas INPUT_PULLUP (cualquier pulsación tira columna a LOW)
static void prepareMatrixForSleep() {
  for (int i = 0; i < FILAS; ++i) {
    pinMode(filas[i], OUTPUT);
    digitalWrite(filas[i], LOW);
  }
  for (int j = 0; j < COLUMNAS; ++j) {
    pinMode(columnas[j], INPUT_PULLUP);
  }
}

// Arma wakes: ADXL INT1 (HIGH), columnas (LOW), encoder dinámico (nivel
// opuesto)
static void configureGpioWakeSources_ADXL_Enc_Btn() {
  // ADXL INT1: activa en ALTO (ya hemos limpiado latch antes)
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN);
  bool adxlHigh = (digitalRead(INT1_ADXL_PIN) == HIGH);
  if (!adxlHigh) {
    gpio_wakeup_enable((gpio_num_t)INT1_ADXL_PIN, GPIO_INTR_HIGH_LEVEL);
  }

  // Encoder: armar por nivel opuesto al actual (evita wake inmediato)
  pinMode(ENC_BUTTON, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  gpio_wakeup_enable((gpio_num_t)ENC_BUTTON, (digitalRead(ENC_BUTTON) == HIGH)
                                                 ? GPIO_INTR_LOW_LEVEL
                                                 : GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable((gpio_num_t)ENC_A, (digitalRead(ENC_A) == HIGH)
                                            ? GPIO_INTR_LOW_LEVEL
                                            : GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable((gpio_num_t)ENC_B, (digitalRead(ENC_B) == HIGH)
                                            ? GPIO_INTR_LOW_LEVEL
                                            : GPIO_INTR_HIGH_LEVEL);

  // Columnas: siempre LOW_LEVEL (si alguna está LOW ahora, abortaremos antes de
  // dormir)
  for (int j = 0; j < COLUMNAS; ++j) {
    pinMode(columnas[j], INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)columnas[j], GPIO_INTR_LOW_LEVEL);
  }

  esp_sleep_enable_gpio_wakeup();
}

static void clearGpioWakeSources_ADXL_Enc_Btn() {
  gpio_wakeup_disable((gpio_num_t)INT1_ADXL_PIN);
  gpio_wakeup_disable((gpio_num_t)ENC_BUTTON);
  gpio_wakeup_disable((gpio_num_t)ENC_A);
  gpio_wakeup_disable((gpio_num_t)ENC_B);
  for (int j = 0; j < COLUMNAS; ++j) {
    gpio_wakeup_disable((gpio_num_t)columnas[j]);
  }
}

// Diagnóstico para evitar wake inmediato y saber qué lo impediría
static bool anyWakePinActiveNowDetailed() {
  bool active = false;

  // Columnas (activas por LOW)
  for (int j = 0; j < COLUMNAS; ++j) {
    pinMode(columnas[j], INPUT_PULLUP);
    if (digitalRead(columnas[j]) == LOW) {
      Serial.printf("Abort reason: columna[%d] is LOW\n", j);
      active = true;
    }
  }

  // ADXL (activo por HIGH)
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN);
  if (digitalRead(INT1_ADXL_PIN) == HIGH) {
    Serial.println("Abort reason: ADXL INT1 is HIGH (latched/ruido).");
    active = true;
  }

  // Encoder no aborta (lo armamos por nivel opuesto), pero mostramos estado
  pinMode(ENC_BUTTON, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  if (digitalRead(ENC_BUTTON) == LOW)
    Serial.println("Note: ENC_BUTTON LOW (se armará por HIGH).");
  if (digitalRead(ENC_A) == LOW)
    Serial.println("Note: ENC_A LOW (se armará por HIGH).");
  if (digitalRead(ENC_B) == LOW)
    Serial.println("Note: ENC_B LOW (se armará por HIGH).");

  return active;
}

static void enterLightSleep() {
  if (adxl_sleeping)
    return;
  adxl_sleeping = true;

  if (displayOn)
    display_sleep();

  extern uint8_t currentBrightness;
  s_savedBrightnessPct = currentBrightness;
  rampBrightnessToPercent(20, /*steps*/ 10, /*ms*/ 15);

  adxl345Handler.init();
  adxl345Handler.clearInterrupts();
  pinMode(INT1_ADXL_PIN, INPUT_PULLDOWN);
  delay(3);

  prepareMatrixForSleep();

  Serial.println("[Sleep] Pre-check pin states:");
  if (anyWakePinActiveNowDetailed()) {
    Serial.println("Sleep abort: wake source already active");
    rampBrightnessToPercent(s_savedBrightnessPct, /*steps*/ 6, /*ms*/ 10);
    display_wakeup();
    adxl_sleeping = false;
    return;
  }

#ifdef MIC
  // Guardar si el modo actual usaba MIC y apagar I2S antes de dormir
  micWasActiveBeforeSleep = useMic;
  if (micWasActiveBeforeSleep) {
    DEBUG__________ln("[Sleep] Apagando MIC antes de light sleep.");
    doitMic.end(); // cierra el driver I2S
  }
#endif

  configureGpioWakeSources_ADXL_Enc_Btn();

  Serial.println("=== Entrando en light sleep (ADXL+ENC+BTN) ===");
  Serial.flush();
  delay(5);
  esp_light_sleep_start(); // se duerme, vuelve aquí al despertar

  clearGpioWakeSources_ADXL_Enc_Btn();
  adxl345Handler.clearInterrupts();

#ifdef MIC
  // Si el modo sigue marcando useMic, rearmar el MIC tras el wake
  if (micWasActiveBeforeSleep && useMic) {
    DEBUG__________ln("[Sleep] Reactivando MIC tras light sleep.");
    doitMic.begin(); // reconfigura I2S para que tu bloque del loop pueda leer
  }
#endif

  Serial.println(
      "=== Salí de light sleep (wake por gpio/adxl/encoder/botón) ===");
  rampBrightnessToPercent(s_savedBrightnessPct, /*steps*/ 8, /*ms*/ 10);
  display_wakeup();

  adxl_sleeping = false;
}

// Tick que decide si ya toca dormir (se llama desde el loop)
static void sleepManagerTick() {
  extern unsigned long lastDisplayInteraction;
  extern bool displayOn;

  // Si la pantalla está encendida, nunca dormimos
  if (displayOn)
    return;

  // ============================================================
  // LÓGICA DE BLOQUEO DE SUEÑO (MICRÓFONO Y ADXL)
  // ============================================================

  // 1. BLOQUEO POR MICRÓFONO (Solo si hay ruido reciente)
  if (useMic) {
    unsigned long lastSound = doitMic.getLastSoundTime();
    // Si hubo ruido hace menos de SLEEP_AFTER_MS, no dormimos
    if (millis() - lastSound < SLEEP_AFTER_MS) {
      return;
    }
    // Si hay silencio prolongado, dejamos caer al siguiente check
  }

  // 2. BLOQUEO POR MOVIMIENTO ADXL (Solo si hay movimiento reciente)
  if (adxl) {
    unsigned long lastMove = adxl345Handler.getLastMovementTime();
    // Si hubo movimiento hace menos de SLEEP_AFTER_MS, no dormimos
    if (millis() - lastMove < SLEEP_AFTER_MS) {
      return;
    }
  }

  // ============================================================

  // Si llegamos aquí, es que no hay ni ruido ni movimiento reciente (o están
  // desactivados) Comprobamos el timeout global de la pantalla + inactividad
  if (millis() - lastDisplayInteraction >= (DISPLAY_OFF_MS + SLEEP_AFTER_MS)) {
    DEBUG__________ln("💤 Inactividad total (Silencio/Quieto). Durmiendo...");
    enterLightSleep();
  }
}

void loop() {
  if (g_stateMutex) {
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_sensorState.isCurrentElementSelected = isCurrentElementSelected();
    g_sensorState.isInMainMenu = isInMainMenu();
    g_sensorState.isStandardMode = (elementFiles[currentIndex] != "Ambientes" &&
                                    elementFiles[currentIndex] != "Fichas" &&
                                    elementFiles[currentIndex] != "Apagar");
    g_sensorState.targetNS = getCurrentElementNS();

    static String lastCheckedFile = "";
    static int lastCheckedMode = -1;
    if (elementFiles[currentIndex] != lastCheckedFile ||
        currentModeIndex != lastCheckedMode) {
      byte req[2] = {0};
      getModeConfig(elementFiles[currentIndex], currentModeIndex, req);
      g_sensorState.hasBinarySensors = getModeFlag(req, HAS_BINARY_SENSORS);
      g_sensorState.hasRelay = getModeFlag(req, HAS_RELAY);
      g_sensorState.hasRelayN1 = getModeFlag(req, HAS_RELAY_N1);
      g_sensorState.hasRelayN2 = getModeFlag(req, HAS_RELAY_N2);

      // --- Actualizar Flags en Estado Compartido para sensorTask ---
      g_sensorState.hasAdxl = getModeFlag(req, HAS_SENS_VAL_1);
      g_sensorState.hasMic = getModeFlag(req, HAS_SENS_VAL_2);

      // Sincronizar globals (por compatibilidad si se usan en otros sitios)
      adxl = g_sensorState.hasAdxl;
      useMic = g_sensorState.hasMic;

      lastCheckedFile = elementFiles[currentIndex];
      lastCheckedMode = currentModeIndex;
    }
    xSemaphoreGive(g_stateMutex);
  }

  if (ap_ota_activo) {
    ArduinoOTA.handle();
  }

  updateBatteryStatus();
  // ────────────────────────────────────────────────
  // Bloqueo total del loop por batería crítica
  // ────────────────────────────────────────────────
  static bool wasLocked = false;

  // "Cargando" a nivel visual: batteryVisualPercentage == BAT_VIS_CHARGING
  // (200)
  const bool isChargingVisual = (batteryVisualPercentage == BAT_VIS_CHARGING);

  // Solo bloqueamos si:
  //  - tenemos criticalBatteryLock
  //  - y NO se está cargando visualmente
  bool lockedNow = (criticalBatteryLock && !isChargingVisual);

  if (lockedNow) {
    applyLedBrightness(0);

    // #ifdef DEBUG
    //     if (!wasLocked) {
    //         Serial.println("[BAT] LOOP BLOQUEADO por batería crítica (sin
    //         carga)");
    //     }
    // #endif

    wasLocked = true;
    return; // corta TODO el resto del loop (salvo la batería, que ya se ha
            // actualizado antes)
  }

  // Aquí NO estamos bloqueados
  if (wasLocked) {
    // Transición LOCK → NO-LOCK: sólo una vez
    applyLedBrightness(255);
    drawCurrentElement();

    // #ifdef DEBUG
    //     Serial.println("[BAT] SALIDA de modo LOCK: UI restaurada");
    // #endif

    wasLocked = false;
  } else {
    // Estado normal continuo
    FastLED.setBrightness(255);
  }

  static int hiddenMenuSelection = 0;
  bool relayPressed = pulsadores.relayButtonIsPressed();

  bool relayHeld = false;
  for (int i = 0; i < FILAS; i++) {
    digitalWrite(filas[i], LOW);
    delayMicroseconds(2);
    for (int j = 0; j < COLUMNAS; j++) {
      if (pulsadorColor[i][j] == RELAY && digitalRead(columnas[j]) == LOW) {
        relayHeld = true;
      }
    }
    digitalWrite(filas[i], HIGH);
  }

  if (!hiddenMenuActive && !confirmDeleteMenuActive &&
      !deleteElementMenuActive && !formatSubMenuActive && !soundMenuActive &&
      !brightnessMenuActive && !languageMenuActive && !inModesScreen) {
    if (digitalRead(ENC_BUTTON) == LOW && relayHeld) {
      hiddenMenuActive = true;
      isLongPress = false;
      hiddenMenuSelection = 0;
      drawHiddenMenu(hiddenMenuSelection);
      buttonPressStart = 0;
    }
  }

#ifdef NFC
  // NOTA: en Adafruit_PN532.cpp bajar el timeout del ACK (1000 -> 100)

  static bool localCardProcessed = false;
  static unsigned long ndefRetryNextMs = 0;
  static uint8_t ndefFailStreak = 0;
  static unsigned long sessionStartMs = 0;

  String uid;
  String tokenMessage;
  const unsigned long now = millis();

  // Usa las GLOBALES (NO redeclarar):
  // int irqCurr, irqPrev; bool cardIsRead; unsigned long cardRemovalStart;

  irqCurr = digitalRead(PN532_IRQ);

  // 1) Arranque de sesión por IRQ edge (solo si no estamos en menús
  // bloqueantes)
  if (!cardIsRead && !languageMenuActive && !inModesScreen &&
      !hiddenMenuActive) {
    if (irqCurr == LOW && irqPrev == HIGH) {
      lastDisplayInteraction = now;
      DEBUG__________ln("Got NFC IRQ");

      token.handleCardDetected(); // actualiza token.currentUID
      uid = token.currentUID;

      if (!uid.isEmpty()) {
        if (uid != token.lastProcessedUID || !localCardProcessed) {
          cardIsRead = true;
          localCardProcessed = false;
          cardRemovalStart = now;
          sessionStartMs = now;
          ndefRetryNextMs = now; // permitir intento inmediato
          ndefFailStreak = 0;

          DEBUG__________("Tarjeta detectada, UID: ");
          DEBUG__________ln(uid);
        }
      } else {
        DEBUG__________ln("⚠️ Falsa alarma - IRQ pero no se detectó tarjeta");
        token.resetReader();
      }
    }
    irqPrev = irqCurr;
  }
  // 2) Sesión activa: mientras la tarjeta esté presente, reintentar lectura
  // NDEF sin obligar a retirar
  else if (cardIsRead) {

    const bool cardPresent = token.isCardPresent();

    if (cardPresent) {
      cardRemovalStart = now;

      if (!localCardProcessed) {

        // Throttle de reintentos (p.ej. cada 120ms)
        if ((long)(now - ndefRetryNextMs) >= 0) {
          DEBUG__________printf("[NFC] intentando leer NDEF... uid=%s\n",
                                uid.c_str());
          const bool ok = token.leerMensajeNFC(tokenMessage);

          if (ok) {
            DEBUG__________ln("Mensaje NFC: " + tokenMessage);

            const byte bank = token.currentToken.addr.bank;

            // Regla ambiente
            if (ambienteActivo && bank != 0x08) {
              DEBUG__________printf("[NFC] Ficha ignorada: ambienteActivo=1, "
                                    "bank=0x%02X (≠ 0x08)\n",
                                    bank);
              localCardProcessed = true;
            } else {
              // Construir targetsNS
              std::vector<TARGETNS> targetsNS;
              targetsNS.reserve(elementFiles.size());

              for (size_t i = 0; i < elementFiles.size(); i++) {
                if (i >= selectedStates.size() || !selectedStates[i])
                  continue;

                const String &file = elementFiles[i];
                if (file == "Ambientes" || file == "Fichas" ||
                    file == "Apagar" || file == "Comunicador" ||
                    file == "Dado") {
                  continue;
                }

                TARGETNS ns = getNSFromFile(file);
                if (!NS_is_zero(ns)) {
                  targetsNS.push_back(ns);
                  DEBUG__________printf(
                      " - Elemento [%d]: NS = %02X%02X%02X%02X%02X\n", (int)i,
                      ns.mac01, ns.mac02, ns.mac03, ns.mac04, ns.mac05);
                } else {
                  DEBUG__________printf(" - Elemento [%d]: NS vacío, omitido\n",
                                        (int)i);
                }
              }

              token.token_handler(token.currentToken, (uint8_t)currentLanguage,
                                  (bool)token.genre, DEFAULT_BOTONERA,
                                  targetsNS);

              localCardProcessed = true;
            }

            ndefFailStreak = 0;
          } else {
            // Fallo: reintentar SIN retirar tarjeta
            ndefFailStreak++;
            DEBUG__________printf("[NFC] NDEF read fail (streak=%u)\n",
                                  ndefFailStreak);

            // Próximo intento
            ndefRetryNextMs = now + 120;

            // Si encadenas demasiados fallos, reinicia el lector (pero no cada
            // vez)
            if (ndefFailStreak >= 6) {
              DEBUG__________ln(
                  "[NFC] demasiados fallos seguidos -> resetReader()");
              token.resetReader();
              ndefFailStreak = 0;
              // Importante: NO marques localCardProcessed; seguimos intentando.
            }

            // Timeout global de sesión (evita bucles infinitos si el tag está
            // medio fuera)
            if (now - sessionStartMs > 4000) {
              DEBUG__________ln(
                  "[NFC] timeout de sesión (4s) -> resetReader()");
              token.resetReader();
              sessionStartMs = now;
            }
          }
        }
      }
    } else {
      // 3) Retirada real (50ms)
      if (now - cardRemovalStart >= 50) {
        DEBUG__________ln("❌ Ficha retirada.");
        cardIsRead = false;
        localCardProcessed = false;
        token.lastProcessedUID = "";
        token.startListeningToNFC();
      }
    }
  }

  // 4) Watchdog: NO lo uses para “obligar a retirar”.
  // Déjalo como escape si el reader se queda colgado, pero amplía ventana y no
  // lo dispares si reintentamos.
  if (token.uidDetected && !token.mensajeLeido &&
      (millis() - token.uidDetectionTime > 5000)) {
    DEBUG__________ln(
        "⏱️ UID detectado pero NDEF no leído tras 5s. Reiniciando reader.");
    token.uidDetected = false;
    token.mensajeLeido = false;
    cardIsRead = false;
    token.lastProcessedUID = "";
    token.startListeningToNFC();
  }
#endif // NFC

  if (bankSelectionActive) {
    bool wasActiveBefore = bankSelectionActive;
    handleBankSelectionMenu(bankList, selectedBanks);

    if (bankSelectionActive && wasActiveBefore) {
      static unsigned long lastScrollBank = 0;
      if (millis() - lastScrollBank >= 50) {
        drawBankSelectionMenu(bankList, selectedBanks, bankMenuCurrentSelection,
                              bankMenuWindowOffset);
        lastScrollBank = millis();
      }
    }
  }

  // --- Repetición / propuesta en modo “Fichas” ---
  static bool relayPressedLastState = false;
  static bool bluePressedLastState = false;
  static int lastFichasMode = -1;

  String currentFile = elementFiles[currentIndex];
  relayPressed = pulsadores.relayButtonIsPressed();
  bool randomProposal = pulsadores.isButtonPressed(BLUE);

  // ---------- Auto-resync UI ↔ tokenMode (sólo Fichas) ----------
  if (currentFile == "Fichas") {
    TOKEN_MODE_ mustBe = (fichasOption.currentMode == 1)   ? TOKEN_PARTNER_MODE
                         : (fichasOption.currentMode == 2) ? TOKEN_GUESS_MODE
                                                           : TOKEN_BASIC_MODE;
    if (token.tokenCurrentMode != mustBe) {
      token.set_mode(mustBe);
      DEBUG__________printf(
          "[FICHAS][RESYNC] Corrigiendo tokenMode -> %d (UI:%d)\n", mustBe,
          fichasOption.currentMode);
    }
  }

  // Reset de latches/estados cuando cambia el modo de Fichas
  if (currentFile == "Fichas") {
    if (lastFichasMode != fichasOption.currentMode) {
      relayPressedLastState = false;
      bluePressedLastState = false;
      token.waitingForPartner = false;
      memset(&token.propossedToken, 0, sizeof(token.propossedToken));
      lastFichasMode = fichasOption.currentMode;
      DEBUG__________printf(
          "[FICHAS] Mode change -> UI:%d  tokenMode:%d  (latches reset)\n",
          lastFichasMode, token.tokenCurrentMode);
    }
  }

  if (currentFile == "Fichas" && (relayPressed || randomProposal)) {
    DEBUG__________printf("[FICHAS] tick | UI:%d token:%d  relay:%d blue:%d  "
                          "lastRelay:%d lastBlue:%d\n",
                          fichasOption.currentMode, token.tokenCurrentMode,
                          relayPressed ? 1 : 0, randomProposal ? 1 : 0,
                          relayPressedLastState ? 1 : 0,
                          bluePressedLastState ? 1 : 0);

    if (!relayPressedLastState && !bluePressedLastState) {
      DEBUG__________printf("[FICHAS] edge -> enter switch (tokenMode:%d)\n",
                            token.tokenCurrentMode);

      switch (token.tokenCurrentMode) {

      case TOKEN_BASIC_MODE: {
        DEBUG__________ln("[FICHAS][BASIC] case");
        if (relayPressed) {
          const bool isVoiceBank = (token.currentToken.addr.bank > 0x09 &&
                                    token.currentToken.addr.bank < 0x63);
          byte lang =
              isVoiceBank ? static_cast<uint8_t>(currentLanguage) * 10 : 0;

          DEBUG__________printf(
              "[FICHAS][BASIC] relayPressed  cmd=0x%02X bank=0x%02X "
              "file=0x%02X isVoice=%d lang=%d\n",
              token.currentToken.cmd, token.currentToken.addr.bank,
              token.currentToken.addr.file, isVoiceBank ? 1 : 0, lang);

          if (token.currentToken.cmd == TOKEN_FX) {
            DEBUG__________ln("[FICHAS][BASIC] play FX");
            doitPlayer.play_file(token.currentToken.addr.bank,
                                 token.currentToken.addr.file);
          } else {
            DEBUG__________ln("[FICHAS][BASIC] play VOICE");
            doitPlayer.play_file(token.currentToken.addr.bank + token.genre,
                                 token.currentToken.addr.file + lang);
          }
        }
      } break;

      case TOKEN_PARTNER_MODE: {
        DEBUG__________printf("[FICHAS][PARTNER] case  relay:%d  blue:%d\n",
                              relayPressed ? 1 : 0, randomProposal ? 1 : 0);

        if (relayPressed) {
          const bool isVoiceBank = (token.currentToken.addr.bank > 0x09 &&
                                    token.currentToken.addr.bank < 0x63);
          byte lang =
              isVoiceBank ? static_cast<uint8_t>(currentLanguage) * 10 : 0;

          DEBUG__________printf(
              "[FICHAS][PARTNER] repeat cmd=0x%02X bank=0x%02X file=0x%02X "
              "isVoice=%d lang=%d\n",
              token.currentToken.cmd, token.currentToken.addr.bank,
              token.currentToken.addr.file, isVoiceBank ? 1 : 0, lang);

          if (token.currentToken.cmd == TOKEN_FX) {
            doitPlayer.play_file(token.currentToken.addr.bank + token.genre,
                                 token.currentToken.addr.file);
          } else {
            doitPlayer.play_file(token.currentToken.addr.bank + token.genre,
                                 token.currentToken.addr.file + lang);
          }
        }

        if (randomProposal) {
          DEBUG__________ln("[FICHAS][PARTNER] BLUE -> reset pareja");
          token.waitingForPartner = false;
          token.currentToken = TOKEN_::TOKEN_DATA();
        }
      } break;

      case TOKEN_GUESS_MODE: {
        DEBUG__________printf("[FICHAS][GUESS] case  relay:%d  blue:%d\n",
                              relayPressed ? 1 : 0, randomProposal ? 1 : 0);

        if (randomProposal) {
          // Proponer
          std::vector<byte> activeBanks;
          for (size_t i = 0; i < selectedBanks.size(); i++) {
            if (selectedBanks[i])
              activeBanks.push_back(bankList[i]);
          }
          byte guessBank;
          if (!activeBanks.empty()) {
            int idx = random(0, (int)activeBanks.size());
            guessBank = activeBanks[idx];
            DEBUG__________printf("[FICHAS][GUESS] propose rand bank 0x%02X\n",
                                  guessBank);
          } else {
            const uint8_t bankListF[] = {0x0B, 0x0D, 0x0F, 0x11, 0x17,
                                         0x24, 0x26, 0x28, 0x2A};
            const uint8_t numBanks = sizeof(bankListF) / sizeof(bankListF[0]);
            guessBank = bankListF[random(0, numBanks)];
            DEBUG__________printf(
                "[FICHAS][GUESS] propose fallback bank 0x%02X\n", guessBank);
          }
          token.proponer_token(guessBank);
          DEBUG__________printf("[FICHAS][GUESS] proposed B=0x%02X F=0x%02X\n",
                                token.propossedToken.addr.bank,
                                token.propossedToken.addr.file);
        } else if (relayPressed) {
          if (token.propossedToken.addr.bank == 0 &&
              token.propossedToken.addr.file == 0) {
            DEBUG__________ln("[FICHAS][GUESS] relay but no proposal");
          } else {
            const bool proposedIsVoice =
                (token.propossedToken.addr.bank > 0x09 &&
                 token.propossedToken.addr.bank < 0x63);
            byte lang = proposedIsVoice
                            ? static_cast<uint8_t>(currentLanguage) * 10
                            : 0;

            DEBUG__________printf("[FICHAS][GUESS] repeat proposal B=0x%02X "
                                  "F=0x%02X isVoice=%d lang=%d\n",
                                  token.propossedToken.addr.bank,
                                  token.propossedToken.addr.file,
                                  proposedIsVoice ? 1 : 0, lang);

            doitPlayer.play_file(token.propossedToken.addr.bank + token.genre,
                                 token.propossedToken.addr.file + lang);
          }
        }
      } break;

      default:
        DEBUG__________printf("[FICHAS] unknown tokenMode=%d\n",
                              token.tokenCurrentMode);
        break;
      }
    }

    // Latches
    relayPressedLastState = relayPressed;
    bluePressedLastState = randomProposal;
  } else {
    relayPressedLastState = false;
    bluePressedLastState = false;
  }

  ledManager.update();

  // Pulsadores ya adaptado a NS internamente
  pulsadores.procesarPulsadores();

  if (frameReceived) {
    frameReceived = false;
    LAST_ENTRY_FRAME_T LEF = extract_info_from_frameIn(uartBuffer);
    element->RX_main_handler(LEF);
  }

  if (inModesScreen && !bankSelectionActive) {
    static unsigned long lastCallModes = 0;
    unsigned long now = millis();
    if (now - lastCallModes >= 50) {
      drawModesScreen();
      lastCallModes = now;
    }
  }

  if (!confirmDeleteMenuActive && !deleteElementMenuActive &&
      !formatSubMenuActive && !soundMenuActive && !brightnessMenuActive &&
      !languageMenuActive && !inModesScreen && !hiddenMenuActive && displayOn &&
      (millis() - lastDisplayInteraction > DISPLAY_OFF_MS)) {
    display_sleep();
  }

  if (formatSubMenuActive) {
    handleFormatMenu();

    static int lastDrawnIndex = -1;
    static uint32_t lastDrawMs = 0;
    const uint32_t now = millis();

    // Redraw si cambia selección o para animar ticker (50 ms)
    if (formatMenuCurrentIndex != lastDrawnIndex || (now - lastDrawMs) >= 50) {
      drawFormatMenu(formatMenuCurrentIndex);
      lastDrawnIndex = formatMenuCurrentIndex;
      lastDrawMs = now;
    }
    return;
  }

  if (deleteElementMenuActive) {
    handleDeleteElementMenu();
    if (confirmDeleteMenuActive) {
      drawConfirmDelete(confirmedFileToDelete);
      return;
    }
    if (deleteElementMenuActive) {
      drawDeleteElementMenu(deleteElementSelection);
    }
    return;
  }

  if (soundMenuActive) {
    handleSoundMenu();
    return;
  }

  if (confirmDeleteMenuActive) {
    handleConfirmDelete();
    drawConfirmDelete(confirmedFileToDelete);
    return;
  }

  if (confirmRestoreMenuElementActive)
    handleConfirmRestoreElementMenu();
  else if (confirmRestoreMenuActive)
    handleConfirmRestoreMenu();
  else if (deleteElementMenuActive)
    handleDeleteElementMenu();
  else if (brightnessMenuActive)
    handleBrightnessMenu();
  else if (confirmEnableDadoActive) {
    handleConfirmEnableDadoMenu();
    drawConfirmEnableDadoMenu(confirmEnableDadoSelection);
    return;
  } else if (extraElementsMenuActive) {
    handleExtraElementsMenu();
    drawExtraElementsMenu(extraElementsMenuSelection);
    return;
  } else if (hiddenMenuActive) {
    handleHiddenMenuNavigation(hiddenMenuSelection);
    static unsigned long lastCall = 0;
    unsigned long now = millis();
    if (now - lastCall >= 50) {
      scrollTextTickerBounce(hiddenMenuSelection);
      lastCall = now;
    }
  } else {
    handleEncoder();
  }

#if false // defined(MIC) || defined(ADXL) MOVED TO SENSORTASK
    if (useMic != lastUseMic || adxl != lastUseAdxl)
    {
#ifdef MIC
        if (useMic)
        {
            DEBUG__________ln("🎤 Se activó el micrófono.");
#ifdef ADXL
            // 1. Apagar ADXL si estaba activo para liberar el bus I2C
            if (lastUseAdxl)
            {
                adxl345Handler.end();
                delay(100);
            }
#endif
            // 2. IMPORTANTE: Resetear pines compartidos (47/48) para eliminar configuración I2C residual
            gpio_reset_pin((gpio_num_t)47);
            gpio_reset_pin((gpio_num_t)48);
            delay(50);

            // 3. Iniciar Micrófono (configurará I2S)
            useMicrophone();
            lastUseMic = true;
            lastUseAdxl = false;
        }
#endif

#ifdef ADXL
        if (adxl)
        {
            DEBUG__________ln("📏 Se activó el acelerómetro.");
#ifdef MIC
            // 1. Apagar Micrófono si estaba activo para liberar driver I2S
            if (lastUseMic)
            {
                doitMic.end();
                delay(100);
            }
#endif
            // 2. IMPORTANTE: Resetear pines compartidos (47/48) para eliminar configuración I2S residual
            gpio_reset_pin((gpio_num_t)47);
            gpio_reset_pin((gpio_num_t)48);
            
            // Ayuda extra: Pre-configurar como INPUT_PULLUP para que Wire.begin encuentre el bus limpio
            pinMode(47, INPUT_PULLUP);
            pinMode(48, INPUT_PULLUP);
            delay(50);

            // 3. Iniciar Acelerómetro (configurará I2C)
            useAccelerometer();
            lastUseMic = false;
            lastUseAdxl = true;
        }
#endif

        // CASO: Apagar ambos
        if (!useMic && !adxl)
        {
#ifdef MIC
            if (lastUseMic)
            {
                doitMic.end();
                delay(100);
            }
#endif
#ifdef ADXL
            if (lastUseAdxl)
            {
                adxl345Handler.end();
                delay(100);
            }
#endif
            // Limpieza final de pines
            gpio_reset_pin((gpio_num_t)47);
            gpio_reset_pin((gpio_num_t)48);

            lastUseMic = false;
            lastUseAdxl = false;
        }
    }

#ifdef MIC
    // ====== MIC por NS (antes por ID) ======
    if (useMic)
    {
        // Asegura que el elemento actual es “dispositivo” (no RAM)
        currentFile = elementFiles[currentIndex];

        byte modeConfig[2] = {0};
        if (!getModeConfig(currentFile, currentModeIndex, modeConfig))
        {
            DEBUG__________ln("⚠️ No se pudo obtener la configuración del modo actual (MIC).");
        }
        else if (isCurrentElementSelected())
        {

            // Resolver NS del elemento enfocado
            TARGETNS ns = getCurrentElementNS(); // devuelve NS_ZERO para RAM
            if (!NS_is_zero(ns))
            {
                const unsigned long micInterval = 50; // orig 50
                static unsigned long lastMicCheck = 0;
                static int falseCount = 0;
                static bool lastSentState = false;
                static unsigned long lastSendTime = 0;
                static byte micValueAnterior = 0;
                const byte DELTA_THRESHOLD = 10;

                bool isBinary = getModeFlag(modeConfig, HAS_BINARY_SENSORS);

                SENSOR_VALUE_T value;
                if (isBinary)
                {
                    // rango 0..1
                    value.msb_min = 0x00;
                    value.lsb_min = 0x00;
                    value.msb_max = 0x00;
                    value.lsb_max = 0x01;

                    bool sonido = doitMic.detect_sound_threshold();
                    if (sonido)
                    {
                        falseCount = 0;
                        if (!awaitingResponse && !lastSentState && (millis() - lastSendTime >= 300))
                        {
                            value.msb_val = 0x00;
                            value.lsb_val = 0x01;
                            send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                            lastSendTime = millis();
                            lastSentState = true;
                        }
                    }
                    else
                    {
                        falseCount++;
                        // Afinado: menos reintentos si no hay relé
                        bool hasRelay = getModeFlag(modeConfig, HAS_RELAY);
                        bool hasRelayN1 = getModeFlag(modeConfig, HAS_RELAY_N1);
                        bool hasRelayN2 = getModeFlag(modeConfig, HAS_RELAY_N2);
                        int relayCount = (hasRelayN1 << 1) | hasRelayN2;
                        relayCount = (relayCount == 0 && hasRelay) ? 1 : relayCount + (hasRelay ? 1 : 0);
                        int maxretry = (relayCount > 0) ? 5 : 3;

                        if (falseCount == maxretry && lastSentState)
                        {
                            value.msb_val = 0x00;
                            value.lsb_val = 0x00;
                            send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                            lastSendTime = millis();
                            lastSentState = false;
                        }
                    }
                }
                else
                {
                    // // analógico 0..255
                    // value.msb_min = 0x00;
                    // value.lsb_min = 0x00;
                    // value.msb_max = 0x00;
                    // value.lsb_max = 0xFF;

                    // const byte SILENCE_THRESHOLD = 5; // Ajusta según ruido de fondo real

                    // if (millis() - lastMicCheck >= micInterval)
                    // {
                    //     lastMicCheck = millis();

                    //     const uint8_t rawValue  = doitMic.get_mic_value_BYTE(micSensRuntime);
                    //     const uint8_t threshold = micSilenceThreshold;

                    //     // Silencio REAL: X ms seguidos por debajo del umbral
                    //     constexpr uint32_t SILENCE_HOLD_MS = 10;//200; // AJUSTA: 150..250 según gusto
                    //     static uint32_t belowSinceMs = 0;
                    //     static bool     inSilence    = true;

                    //     // 1) Detectar "sonido" de forma INMEDIATA al superar umbral
                    //     if (rawValue >= threshold)
                    //     {
                    //         belowSinceMs = 0;

                    //         // Acabas de salir de silencio -> manda ya el primer valor (sin esperar a DELTA_THRESHOLD)
                    //         if (inSilence)
                    //         {
                    //             inSilence = false;

                    //             micValueAnterior = rawValue;
                    //             value.msb_val = 0x00;
                    //             value.lsb_val = rawValue;

                    //             doitMic.updateSoundTime();

                    //             if (!awaitingResponse)
                    //             {
                    //                 send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                    //             }
                    //         }
                    //         else
                    //         {
                    //             // Ya estabas en sonido: manda solo si cambia suficiente (reduce spam)
                    //             doitMic.updateSoundTime();

                    //             if (abs((int)rawValue - (int)micValueAnterior) >= (int)DELTA_THRESHOLD)
                    //             {
                    //                 micValueAnterior = rawValue;
                    //                 value.msb_val = 0x00;
                    //                 value.lsb_val = rawValue;

                    //                 if (!awaitingResponse)
                    //                 {
                    //                     send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                    //                 }
                    //             }
                    //         }
                    //     }
                    //     else
                    //     {
                    //         // 2) Candidato a silencio: empieza/continúa el temporizador
                    //         if (belowSinceMs == 0) belowSinceMs = millis();

                    //         // Si llevas X ms por debajo y NO estabas en silencio, manda 0 una sola vez
                    //         if (!inSilence && (millis() - belowSinceMs) >= SILENCE_HOLD_MS)
                    //         {
                    //             inSilence = true;
                    //             micValueAnterior = 0;

                    //             value.msb_val = 0x00;
                    //             value.lsb_val = 0x00;

                    //             if (!awaitingResponse)
                    //             {
                    //                 send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                    //             }
                    //         }
                    //     }
                    // }
                    // --- MODO ANALÓGICO (0..255) ---
                    value.msb_min = 0x00;
                    value.lsb_min = 0x00;
                    value.msb_max = 0x00;
                    value.lsb_max = 0xFF;

                    // CAMBIO CRÍTICO 1: Umbral de variación muy bajo para detectar cambios sutiles
                    const byte DELTA_THRESHOLD = 2; 

                    if (millis() - lastMicCheck >= micInterval)
                    {
                        lastMicCheck = millis();

                        const uint8_t rawValue  = doitMic.get_mic_value_BYTE(micSensRuntime);
                        const uint8_t threshold = micSilenceThreshold;

                        // Lógica de "Silence Hold" para estabilizar la caída a cero
                        constexpr uint32_t SILENCE_HOLD_MS = 100; 
                        static uint32_t belowSinceMs = 0;
                        static bool     inSilence    = true;

                        // --- 1) Detección de SONIDO ---
                        if (rawValue >= threshold)
                        {
                            belowSinceMs = 0; // Reseteamos temporizador de silencio

                            // Caso A: Salimos del silencio -> ENVIAR SIEMPRE
                            if (inSilence)
                            {
                                inSilence = false;
                                micValueAnterior = rawValue;
                                value.msb_val = 0x00;
                                value.lsb_val = rawValue;

                                doitMic.updateSoundTime();

                                if (!awaitingResponse)
                                {
                                    send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                                    lastSendTime = millis(); // Actualizamos reloj envío
                                }
                            }
                            // Caso B: Ya había sonido -> ENVIAR SI CAMBIA O SI PASA TIEMPO
                            else
                            {
                                doitMic.updateSoundTime();

                                // Condición 1: El valor ha cambiado (Delta)
                                if (abs((int)rawValue - (int)micValueAnterior) >= (int)DELTA_THRESHOLD)
                                {
                                    micValueAnterior = rawValue;
                                    value.msb_val = 0x00;
                                    value.lsb_val = rawValue;

                                    if (!awaitingResponse)
                                    {
                                        send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                                        lastSendTime = millis();
                                    }
                                }
                                // Condición 2 (Keep-Alive): El valor es estable pero queremos confirmar que sigue sonando
                                // Esto evita las "pausas" cuando el generador de tonos es muy estable
                                else if (millis() - lastSendTime > 200) 
                                {
                                    if (!awaitingResponse) 
                                    {
                                        value.msb_val = 0x00;
                                        value.lsb_val = rawValue;
                                        // Nota: No actualizamos micValueAnterior aquí para no perder la referencia del delta
                                        send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                                        lastSendTime = millis();
                                    }
                                }
                            }
                        }
                        // --- 2) Detección de SILENCIO ---
                        else
                        {
                            // Empezamos a contar tiempo bajo el umbral
                            if (belowSinceMs == 0) belowSinceMs = millis();

                            // Si pasa el tiempo de hold y NO estábamos ya en silencio, enviamos el 0
                            if (!inSilence && (millis() - belowSinceMs) >= SILENCE_HOLD_MS)
                            {
                                inSilence = true;
                                micValueAnterior = 0;

                                value.msb_val = 0x00;
                                value.lsb_val = 0x00;

                                if (!awaitingResponse)
                                {
                                    send_frame(frameMaker_SEND_SENSOR_VALUE_2(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, value));
                                    lastSendTime = millis();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
#endif // MIC

// #ifdef ADXL
//     // ADXL ya llama internamente a readInclinations() que tú adaptaste a NS
//     if (adxl)
//     {
//         if (isCurrentElementSelected() && isInMainMenu())
//         {
//             if (currentFile != "Ambientes" && currentFile != "Fichas" && currentFile != "Apagar")
//             {
//                 byte situacion = GROUND;
//                 fs::File f = SPIFFS.open(currentFile, "r");
//                 if (f)
//                 {
//                     f.seek(OFFSET_SITUACION, SeekSet);
//                     f.read(&situacion, 1);
//                     f.close();
//                     adxl345Handler.readInclinations(); // ← ya usa NS internamente
//                 }
//                 else
//                 {
//                     DEBUG__________printf("⚠️ No se pudo abrir %s para leer situación.\n", currentFile.c_str());
//                 }
//             }
//         }
//     }
// #endif // ADXL

#if false // MOVED TO SENSORTASK (Core 0) - Don't read I2C from loop/Core 1
#ifdef ADXL
    // ADXL ya llama internamente a readInclinations() que tú adaptaste a NS
    if (adxl)
    {
        if (isCurrentElementSelected() && isInMainMenu())
        {
            if (currentFile != "Ambientes" && currentFile != "Fichas" && currentFile != "Apagar")
            {
                // No abras SPIFFS para leer "situacion" si no lo usas: mete jitter gratuito.
                adxl345Handler.readInclinations(); // ← ya usa NS internamente
            }
        }
    }
#endif // ADXL
#endif // false - MOVED TO SENSORTASK
#endif // #if false (line 1530) - entire legacy sensor block disabled


  if (displayOn && isInMainMenu()) {
    if (nameScrollActive)
      updateNameScroll();
    if (modeScrollActive)
      updateModeScroll();
  }

  static unsigned long lastBatteryMiniUpdate = 0;
  if (!inModesScreen && !hiddenMenuActive && !formatSubMenuActive &&
      !soundMenuActive && !brightnessMenuActive && !languageMenuActive &&
      !bankSelectionActive && !deleteElementMenuActive &&
      !confirmDeleteMenuActive && displayOn && batteryPercentage < 25.0f) {
    if (millis() - lastBatteryMiniUpdate >= 100) {
      drawBatteryIconMini(batteryPercentage);
      uiSprite.pushSprite(0, 0);
      lastBatteryMiniUpdate = millis();
    }
  }
  sleepManagerTick();
}

// == END LOOP == //

#ifdef MIC
void useMicrophone() {
// 1. Desactivar ADXL si estaba activo
#ifdef ADXL
  if (adxl345Handler.isInitialized()) {
    adxl345Handler.end(); // Esto llama a Wire.end()
    DEBUG__________ln("Acelerómetro desactivado para dar paso al MIC.");
  }
#endif

  delay(100);

  // 2. IMPORTANTE: Resetear pines compartidos para quitar configuración I2C
  // Esto libera la matriz de pines del ESP32
  gpio_reset_pin((gpio_num_t)47);
  gpio_reset_pin((gpio_num_t)48);

  // (Opcional) Asegurar estado neutral
  pinMode(47, INPUT);
  pinMode(48, INPUT);

  delay(100);

  // 3. Iniciar Micrófono (I2S)
  doitMic.begin();
  DEBUG__________ln("Micrófono activado (I2S en pines 47/48).");
}
#endif

#ifdef ADXL
void useAccelerometer() {
// 1. Desactivar Micrófono si estaba activo
#ifdef MIC
  // Usamos isActive() para comprobar si el driver I2S realmente está instalado
  if (doitMic.isActive()) {
    doitMic.end();
    delay(50); // Pequeña espera para que el driver I2S libere recursos
  }
#endif

  DEBUG__________ln(
      "Pines 47/48 liberados de I2S. Iniciando recuperación I2C...");

  // 2. RESET PROFUNDO Y RECUPERACIÓN DE BUS (Bus Recovery)
  // Esto es crucial cuando se comparten pines con I2S.

  // A) Soltar la matriz de pines del ESP32
  gpio_reset_pin((gpio_num_t)47);
  gpio_reset_pin((gpio_num_t)48);

  // B) Secuencia manual para desatascar el bus I2C (Bit-banging)
  // SCL (48) se configura como salida para generar pulsos
  // SDA (47) se deja como entrada (PullUp)
  pinMode(47, INPUT_PULLUP);
  pinMode(48, OUTPUT);

  digitalWrite(48, HIGH); // Empezamos en alto

  // Generamos 9 pulsos de reloj manuales.
  // Esto obliga a cualquier dispositivo I2C atascado a soltar la línea SDA.
  for (int i = 0; i < 9; i++) {
    digitalWrite(48, LOW);
    delayMicroseconds(10);
    digitalWrite(48, HIGH);
    delayMicroseconds(10);
  }

  // C) Generar una condición de STOP manual
  pinMode(47, OUTPUT);
  digitalWrite(47, LOW); // SDA Low
  delayMicroseconds(10);
  digitalWrite(48, HIGH); // SCL High
  delayMicroseconds(10);
  digitalWrite(47, HIGH); // SDA High mientras SCL es High (STOP condition)
  delayMicroseconds(10);

  // 3. Preparar pines para la librería Wire
  pinMode(47, INPUT_PULLUP);
  pinMode(48, INPUT_PULLUP);
  delay(50);

  // 4. Iniciar Acelerómetro (Hardware I2C)
  // Forzamos un reinicio de la librería Wire dentro del handler
  adxl345Handler.end(); // Aseguramos que Wire se cierre antes de reabrir
  adxl345Handler.init();

  if (adxl345Handler.isInitialized()) {
    DEBUG__________ln("✅ Acelerómetro activado y detectado (Pines 47/48).");
  } else {
    DEBUG__________ln(
        "❌ Error crítico: ADXL no responde ni tras Bus Recovery.");
  }
}
#endif


const BatteryIconThreshold batteryLevels[] = {
    // { enterThreshold, exitThreshold, iconIndex }
    {0.0, 27.0, 2},  // bajo (toggle)
    {23.0, 68.0, 3}, // medio
    {64.0, 87.0, 1}, // alto
    {88.0, 100.0,
     4}, // **nuevo bin >98%** (sólo para detección de cambios/redraw)
};

void showSerial() {
  byte sector_data[5];
  String macNumbers = WiFi.macAddress();
  macNumbers = macNumbers.substring(9);
  macNumbers.replace(":", "");

  String serialPrefix = "";

  // Convertir el valor hexadecimal a una cadena de 4 caracteres
  uint16_t serialHex = SERIAL_NUM;
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "%04X", serialHex);
  serialPrefix = String(buffer);

  while (serialPrefix.length() < 4)
    serialPrefix = "0" + serialPrefix;

  sector_data[0] = strtoul(serialPrefix.substring(0, 2).c_str(), nullptr, 16);
  sector_data[1] = strtoul(serialPrefix.substring(2, 4).c_str(), nullptr, 16);
  sector_data[2] = strtoul(macNumbers.substring(0, 2).c_str(), nullptr, 16);
  sector_data[3] = strtoul(macNumbers.substring(2, 4).c_str(), nullptr, 16);
  sector_data[4] = strtoul(macNumbers.substring(4, 6).c_str(), nullptr, 16);

  comunicadorOption.serialNum[0] = sector_data[0];
  comunicadorOption.serialNum[1] = sector_data[1];
  comunicadorOption.serialNum[2] = sector_data[2];
  comunicadorOption.serialNum[3] = sector_data[3];
  comunicadorOption.serialNum[4] = sector_data[4];

  comunicadorOption.ID = DEFAULT_BOTONERA;

  // --- guardar el NS global de la BOTONERA ---
  TARGETNS myNS;
  memcpy(&myNS.mac01, sector_data, 5);
  setOwnNS(myNS);

  Serial.println();
  Serial.println("*******************");
  Serial.print("Serial: ");
  for (int i = 0; i < 5; i++) {
    if (sector_data[i] < 0x10)
      Serial.print("0");
    Serial.print(sector_data[i], HEX);
  }
  Serial.println();
  Serial.println("*******************");
  Serial.println();
}

// =====================================================
// 🧰 UTILIDADES: Filtro de mediana y mapeo tensión→%
// =====================================================
float medianFilter(float sample) {
  if (!medianInit) {
    for (int i = 0; i < MEDIAN_FILTER_SIZE; ++i)
      vBatWindow[i] = sample;
    medianInit = true;
    return sample;
  }
  vBatWindow[vBatIdx++] = sample;
  if (vBatIdx >= MEDIAN_FILTER_SIZE)
    vBatIdx = 0;

  float buf[MEDIAN_FILTER_SIZE];
  memcpy(buf, vBatWindow, sizeof(buf));
  for (int i = 0; i < MEDIAN_FILTER_SIZE - 1; ++i) {
    for (int j = i + 1; j < MEDIAN_FILTER_SIZE; ++j) {
      if (buf[j] < buf[i]) {
        float t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }
  return buf[MEDIAN_FILTER_SIZE / 2];
}

float mapVoltageToPercent(float v) {
  if (v <= voltajes[0])
    return socTable[0];
  if (v >= voltajes[N_SOC - 1])
    return socTable[N_SOC - 1];
  for (uint8_t i = 0; i < N_SOC - 1; ++i) {
    if (v >= voltajes[i] && v < voltajes[i + 1]) {
      float t = (v - voltajes[i]) / (voltajes[i + 1] - voltajes[i]);
      return socTable[i] + t * (socTable[i + 1] - socTable[i]);
    }
  }
  return 0.0f;
}

// =======================================
// ⚙️ INICIALIZACIÓN ADC (corregida)
// =======================================
void initBatteryADC() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db); // 11 dB en pin

  const uint32_t DEFAULT_VREF = 1100;
  esp_adc_cal_characterize(ADC_UNIT_1,
                           ADC_ATTEN_DB_12, // 💡 coincide con el pin
                           ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
}

// ======================================================
// 🔎 LECTURA DE VBAT REAL (con calibración y divisor)
// ======================================================
// float readVBat() {
//     // Pre-carga del Csample para fuente de alta impedancia
//     analogRead(PIN_BAT_ADC);
//     delayMicroseconds(40);

//     // Sobremuestreo y media
//     constexpr uint8_t N = 32;
//     uint32_t sum_raw = 0;
//     for (uint8_t i = 0; i < N; ++i) {
//         sum_raw += analogRead(PIN_BAT_ADC);   // valor RAW
//         delayMicroseconds(5);
//     }
//     uint32_t raw = sum_raw / N;

//     // RAW → mV (pin) usando la caracterización
//     uint32_t mv_at_pin = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

//     // Desescalar por divisor y factor de calibración
//     const float v_adc = mv_at_pin / 1000.0f;
//     const float vbat  = (v_adc / DIV_RATIO) * VBAT_CAL_K;
//     return vbat;
// }

float readVBat() {
  // Pre-carga del Csample para fuente de alta impedancia
  analogRead(PIN_BAT_ADC);
  delayMicroseconds(40);

  // Sobremuestreo y media
  constexpr uint8_t N = 32;
  uint32_t sum_raw = 0;
  for (uint8_t i = 0; i < N; ++i) {
    sum_raw += analogRead(PIN_BAT_ADC); // valor RAW
    delayMicroseconds(5);
  }
  const uint32_t raw = sum_raw / N;

  // RAW → mV en el pin ADC usando la caracterización oficial de Espressif
  const uint32_t mv_at_pin = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  const float v_adc = mv_at_pin / 1000.0f; // tensión en el pin ADC (firmware)

  // Calibración directa V_ADC → VBAT usando el factor obtenido de tus
  // mediciones. Ignoramos DIV_RATIO aquí porque la cadena física+ADC no se
  // comporta como el ideal.
  const float vbat = v_adc * VBAT_ADC_TO_BAT_GAIN;

  return vbat;
}

float batteryVoltagePercent = 0.0f;

void updateBatteryStatus() {
  // ================= Timing =================
  constexpr uint32_t READ_PERIOD_MS = 1000;
  static uint32_t lastCheck = 0;
  const uint32_t now = millis();

  const uint32_t dtMs = (uint32_t)(now - lastCheck);
  if (dtMs < READ_PERIOD_MS)
    return;
  lastCheck = now;

  const bool timeGap = (dtMs > (READ_PERIOD_MS * 3U));

  // ================= Parámetros batería =================
  constexpr float VBAT_MAX_VISUAL = 4.20f;
  constexpr float VBAT_EMPTY = 2.90f;

  // ================= Fases de arranque (sin pines STAT/PG) =================
  constexpr uint32_t STARTUP_MIN_MS = 20000;
  constexpr uint32_t WELCOME_GUARD_MS = 5000;

  // ================= Filtros =================
  constexpr float ALPHA_LOCK = 0.20f;

  // ====== ADD (V1): canal de detección más estable para evitar “flip” de
  // cuantización ====== Ojo: NO lo uso para el “edge” (rayo inmediato), solo
  // para buffers largos/slope/score.
  constexpr float ALPHA_DET = 0.35f; // ~2–3 s de asentamiento, suficientemente
                                     // estable sin matar tendencia

  // ================= Heurística solo VBAT (ADC) =================
  // Evento: caída rápida => bloquea inferencia temporalmente (pico de carga)
  constexpr float EVENT_DROP_5S_mV = 12.0f;
  constexpr uint32_t EVENT_BLOCK_MS = 120000UL;

  // Ventanas (1 muestra/s)
  constexpr int WIN_5S = 5;
  constexpr int WIN_15S = 15;
  constexpr int WIN_60S = 60;
  constexpr int WIN_180S = 180;

  // Entrada por score (confirmación lenta)
  constexpr float RISE_60S_mV_ENTER = 3.0f;
  constexpr float RISE_180S_mV_ENTER = 7.0f;
  constexpr float RISE_180S_mV_STRONG = 12.0f;

  // Salidas
  constexpr float FALL_5S_EXIT_mV = -20.0f;
  constexpr float FALL_60S_EXIT_mV = -8.0f;

  // “Likely charging” (rápido)
  constexpr float LIKELY_RISE_15S_mV = 10.0f;
  constexpr float LIKELY_RISE_60S_mV = 4.0f;
  constexpr float LIKELY_SLOPE60_mVmin = 1.0f;
  constexpr float LIKELY_NO_FALL_5S_mV = -2.0f;
  constexpr uint32_t LIKELY_HOLD_MS = 30000UL;

  // “Charging edge” (inmediato al enchufar USB)
  // Importante: detección sobre MEDIANA (rápida), no sobre canales IIR.
  constexpr float EDGE_RISE_1S_mV = 25.0f;
  constexpr float EDGE_RISE_5S_mV = 50.0f;
  constexpr uint32_t EDGE_HOLD_MS = 90000UL;

  // Veto tras bajada de carga (rebote parece carga)
  constexpr uint32_t LOAD_DROP_VETO_MS = 20000UL;

  // Evaluación del score
  constexpr uint32_t EVAL_PERIOD_MS = 30000UL;
  constexpr int SCORE_ENTER = 3;
  constexpr int SCORE_EXIT = 1;

  // Estabilidad (solo calidad de muestra)
  constexpr int16_t STABLE_RANGE_5S_mV = 6;

  // “Charged” aproximado
  constexpr float VBAT_CV_ENTER = 4.18f;
  constexpr float VBAT_CV_EXIT = 4.10f;
  constexpr uint32_t CHARGED_STABLE_MS = 90000UL;

  // ================= Estado interno =================
  static bool primed = false;
  static float vBatLock = 0.0f;

  // ====== ADD (V1): canal de detección estable (IIR sobre vBatMedian) ======
  static float vBatDet = 0.0f;

  // Buffers en mV de la SEÑAL DE DETECCIÓN (ahora: vBatDet, no vBatMedian)
  static int16_t buf5[WIN_5S];
  static int16_t buf15[WIN_15S];
  static int16_t buf60[WIN_60S];
  static int16_t buf180[WIN_180S];

  static int idx5 = 0, idx15 = 0, idx60 = 0, idx180 = 0;
  static int cnt5 = 0, cnt15 = 0, cnt60 = 0, cnt180 = 0;

  // ====== ADD: buffer 5s “edge/estabilidad/eventos” sobre MEDIANA rápida
  // ======
  static int16_t buf5_edge[WIN_5S];
  static int idx5e = 0;
  static int cnt5e = 0;

  // dv1s del EDGE (mediana rápida)
  static int16_t prevMed_mV = 0;
  static bool prevMedInit = false;

  static uint32_t eventBlockUntil = 0;
  static int chargeScore = 0;
  static uint32_t lastEval = 0;

  static uint32_t firstSampleMs = 0;
  static uint32_t welcomeDoneSince = 0;

  // Detección rápida
  static bool likelyCharging = false;
  static uint32_t likelyUntil = 0;

  static bool chargingEdgeVisual = false;
  static uint32_t edgeUntil = 0;

  // Veto por caída absoluta mientras UI dice “carga”
  static bool vetoByAbsoluteDrop = false;
  static int16_t chargeEntryLockRef_mV = 0;

  // Veto por transición de carga (rebote)
  static uint8_t prevLoadBits = 0;
  static uint32_t loadDropVetoUntil = 0;

  static uint32_t chargedStableSince = 0;

  auto ring_oldest_full = [](const int16_t *b, int head) -> int16_t {
    return b[head];
  };

  auto ring_minmax_full = [](const int16_t *b, int size, int head,
                             int16_t &outMin, int16_t &outMax) {
    outMin = 32767;
    outMax = -32768;
    for (int i = 0; i < size; ++i) {
      const int idx = (head + i) % size;
      const int16_t v = b[idx];
      if (v < outMin)
        outMin = v;
      if (v > outMax)
        outMax = v;
    }
  };

  auto linear_slope_mV_per_min_full = [](const int16_t *b, int size,
                                         int head) -> float {
    const int n = size;
    const double meanX = (n - 1) * 0.5;
    double sumXX = 0.0;
    double sumXY = 0.0;

    for (int i = 0; i < n; ++i) {
      const int idx = (head + i) % n; // oldest..newest
      const double x = (double)i - meanX;
      const double y = (double)b[idx];
      sumXX += x * x;
      sumXY += x * y;
    }
    if (sumXX <= 0.0)
      return 0.0f;
    const double slope_mV_per_s = sumXY / sumXX; // 1 sample = 1s
    return (float)(slope_mV_per_s * 60.0);       // mV/min
  };

  auto getLoadBits = []() -> uint8_t {
    uint8_t bits = 0;

    // bit0: display
    if (displayOn)
      bits |= 0x01;

    // bit1: LEDs altos (umbral suficiente)
    extern uint8_t currentBrightness;
    if (currentBrightness > 70)
      bits |= 0x02;

    // bit2: mic
    if (useMic)
      bits |= 0x04;

    // bit3: ADXL activo
    if (adxl)
      bits |= 0x08;

    return bits;
  };

  // ================= 1) Lectura + filtros =================
  const float vBatRaw = readVBat();
  const float vBatMedian = medianFilter(vBatRaw);

  if (!primed) {
    if (firstSampleMs == 0)
      firstSampleMs = now;

    vBatFiltered = vBatMedian;
    vBatLock = vBatMedian;

    // ADD: inicializa canal estable
    vBatDet = vBatMedian;

    const int16_t det_mV = (int16_t)(vBatDet * 1000.0f + 0.5f);
    const int16_t med_mV = (int16_t)(vBatMedian * 1000.0f + 0.5f);

    for (int i = 0; i < WIN_5S; ++i)
      buf5[i] = det_mV;
    for (int i = 0; i < WIN_15S; ++i)
      buf15[i] = det_mV;
    for (int i = 0; i < WIN_60S; ++i)
      buf60[i] = det_mV;
    for (int i = 0; i < WIN_180S; ++i)
      buf180[i] = det_mV;

    for (int i = 0; i < WIN_5S; ++i)
      buf5_edge[i] = med_mV;

    idx5 = idx15 = idx60 = idx180 = 0;
    cnt5 = cnt15 = cnt60 = cnt180 = 0;

    idx5e = 0;
    cnt5e = 0;

    prevMed_mV = med_mV;
    prevMedInit = true;

    chargeScore = 0;
    eventBlockUntil = 0;
    lastEval = now;

    likelyCharging = false;
    likelyUntil = 0;

    chargingEdgeVisual = false;
    edgeUntil = 0;

    vetoByAbsoluteDrop = false;
    chargeEntryLockRef_mV = 0;

    prevLoadBits = getLoadBits();
    loadDropVetoUntil = 0;

    chargedStableSince = 0;

    primed = true;
  } else {
    if (timeGap) {
      vBatFiltered = vBatMedian;
      vBatLock = vBatMedian;

      // ADD: reset canal estable
      vBatDet = vBatMedian;

      const int16_t det_mV = (int16_t)(vBatDet * 1000.0f + 0.5f);
      const int16_t med_mV = (int16_t)(vBatMedian * 1000.0f + 0.5f);

      for (int i = 0; i < WIN_5S; ++i)
        buf5[i] = det_mV;
      for (int i = 0; i < WIN_15S; ++i)
        buf15[i] = det_mV;
      for (int i = 0; i < WIN_60S; ++i)
        buf60[i] = det_mV;
      for (int i = 0; i < WIN_180S; ++i)
        buf180[i] = det_mV;

      for (int i = 0; i < WIN_5S; ++i)
        buf5_edge[i] = med_mV;

      idx5 = idx15 = idx60 = idx180 = 0;
      cnt5 = cnt15 = cnt60 = cnt180 = 0;

      idx5e = 0;
      cnt5e = 0;

      prevMed_mV = med_mV;
      prevMedInit = true;

      chargeScore = 0;
      eventBlockUntil = 0;
      lastEval = now;

      likelyCharging = false;
      likelyUntil = 0;

      chargingEdgeVisual = false;
      edgeUntil = 0;

      vetoByAbsoluteDrop = false;
      chargeEntryLockRef_mV = 0;

      prevLoadBits = getLoadBits();
      loadDropVetoUntil = 0;

      chargedStableSince = 0;
    } else {
      // Canal visual (lento)
      vBatFiltered = ALPHA * vBatMedian + (1.0f - ALPHA) * vBatFiltered;

      // Canal lock (más rápido)
      vBatLock = ALPHA_LOCK * vBatMedian + (1.0f - ALPHA_LOCK) * vBatLock;

      // ADD (V1): canal detección estable (más rápido que vBatFiltered)
      vBatDet = ALPHA_DET * vBatMedian + (1.0f - ALPHA_DET) * vBatDet;
    }
  }

  if (vBatFiltered > 4.25f)
    vBatFiltered = 4.25f;

  // ================= 1.1) Señales en mV =================
  const int16_t medNow_mV = (int16_t)(vBatMedian * 1000.0f + 0.5f);
  int16_t dv1s_mV = 0; // (EDGE) basado en mediana rápida
  if (prevMedInit)
    dv1s_mV = (int16_t)(medNow_mV - prevMed_mV);
  prevMed_mV = medNow_mV;
  prevMedInit = true;

  // Señal de detección estable (la que alimenta buffers largos)
  const int16_t detNow_mV = (int16_t)(vBatDet * 1000.0f + 0.5f);

  // ================= 1.5) Veto tras bajada de carga =================
  const uint8_t loadBits = getLoadBits();
  const bool loadDropped = ((prevLoadBits & (uint8_t)~loadBits) != 0);
  if (loadDropped)
    loadDropVetoUntil = now + LOAD_DROP_VETO_MS;
  prevLoadBits = loadBits;

  const bool vetoRiseByLoadDrop = ((int32_t)(now - loadDropVetoUntil) < 0);

  // ================= 2) Buffers =================
  // 2A) Buffers largos: DET estable
  buf5[idx5] = detNow_mV;
  idx5 = (idx5 + 1) % WIN_5S;
  if (cnt5 < WIN_5S)
    cnt5++;

  buf15[idx15] = detNow_mV;
  idx15 = (idx15 + 1) % WIN_15S;
  if (cnt15 < WIN_15S)
    cnt15++;

  buf60[idx60] = detNow_mV;
  idx60 = (idx60 + 1) % WIN_60S;
  if (cnt60 < WIN_60S)
    cnt60++;

  buf180[idx180] = detNow_mV;
  idx180 = (idx180 + 1) % WIN_180S;
  if (cnt180 < WIN_180S)
    cnt180++;

  // 2B) Buffer 5s rápido: MED (edge/event/stable)
  buf5_edge[idx5e] = medNow_mV;
  idx5e = (idx5e + 1) % WIN_5S;
  if (cnt5e < WIN_5S)
    cnt5e++;

  float dV5_mV = 0.0f;
  float dV15_mV = 0.0f;
  float dV60_mV = 0.0f;
  float dV180_mV = 0.0f;

  if (cnt5 == WIN_5S)
    dV5_mV = (float)(detNow_mV - ring_oldest_full(buf5, idx5));
  if (cnt15 == WIN_15S)
    dV15_mV = (float)(detNow_mV - ring_oldest_full(buf15, idx15));
  if (cnt60 == WIN_60S)
    dV60_mV = (float)(detNow_mV - ring_oldest_full(buf60, idx60));
  if (cnt180 == WIN_180S)
    dV180_mV = (float)(detNow_mV - ring_oldest_full(buf180, idx180));

  float dV5_edge_mV = 0.0f;
  if (cnt5e == WIN_5S)
    dV5_edge_mV = (float)(medNow_mV - ring_oldest_full(buf5_edge, idx5e));

  // ================= 3) Estabilidad + Bloqueo por eventos =================
  // IMPORTANTE: estabilidad basada en MED rápida (si la haces con DET estable,
  // te “falsea” la calidad).
  bool stable5s = false;
  if (cnt5e == WIN_5S) {
    int16_t mn, mx;
    ring_minmax_full(buf5_edge, WIN_5S, idx5e, mn, mx);
    stable5s = ((mx - mn) <= STABLE_RANGE_5S_mV);
  }

  // Evento de caída rápida: usar también MED rápida (DET estable puede
  // ocultarlo)
  if (cnt5e == WIN_5S && dV5_edge_mV <= -EVENT_DROP_5S_mV) {
    eventBlockUntil = now + EVENT_BLOCK_MS;
    if (chargeScore > 0)
      chargeScore -= 2;
    if (chargeScore < 0)
      chargeScore = 0;

    chargingEdgeVisual = false;
    edgeUntil = 0;
    likelyCharging = false;
    likelyUntil = 0;
  }
  const bool blockedByEvent = ((int32_t)(now - eventBlockUntil) < 0);

  const bool welcomeDone = (s_welcomeLedsDone && s_welcomeDisplayDone);
  if (welcomeDone) {
    if (welcomeDoneSince == 0)
      welcomeDoneSince = now;
  } else {
    welcomeDoneSince = 0;
  }
  if (firstSampleMs == 0)
    firstSampleMs = now;

  const bool blockedByBoot =
      ((uint32_t)(now - firstSampleMs) < STARTUP_MIN_MS) || (!welcomeDone) ||
      (welcomeDoneSince == 0) ||
      ((uint32_t)(now - welcomeDoneSince) < WELCOME_GUARD_MS);

  float slope60_mVmin = 0.0f;
  float slope180_mVmin = 0.0f;
  if (cnt60 == WIN_60S)
    slope60_mVmin = linear_slope_mV_per_min_full(buf60, WIN_60S, idx60);
  if (cnt180 == WIN_180S)
    slope180_mVmin = linear_slope_mV_per_min_full(buf180, WIN_180S, idx180);

  // ================= 4) Detección rápida: edge y likely =================
  // 4A) Edge: subida brusca (SIEMPRE sobre MED rápida)
  if (!blockedByBoot && !blockedByEvent && !vetoRiseByLoadDrop) {
    const bool edgeNow = ((float)dv1s_mV >= EDGE_RISE_1S_mV) ||
                         (cnt5e == WIN_5S && dV5_edge_mV >= EDGE_RISE_5S_mV);

    if (edgeNow) {
      chargingEdgeVisual = true;
      edgeUntil = now + EDGE_HOLD_MS;
    }
  }
  if (chargingEdgeVisual && (int32_t)(now - edgeUntil) >= 0) {
    chargingEdgeVisual = false;
  }

  // 4B) Likely: subida sostenida (usa DET estable -> menos falsos por
  // cuantización)
  if (!blockedByBoot && !blockedByEvent && !vetoRiseByLoadDrop) {
    bool likelyNow = false;

    if (cnt15 == WIN_15S) {
      likelyNow = (dV15_mV >= LIKELY_RISE_15S_mV) &&
                  (cnt5e != WIN_5S || dV5_edge_mV >= LIKELY_NO_FALL_5S_mV);
    }

    if (!likelyNow && cnt60 == WIN_60S) {
      likelyNow = (dV60_mV >= LIKELY_RISE_60S_mV) &&
                  (slope60_mVmin >= LIKELY_SLOPE60_mVmin) &&
                  (cnt5e != WIN_5S || dV5_edge_mV >= LIKELY_NO_FALL_5S_mV);
    }

    if (likelyNow) {
      likelyCharging = true;
      likelyUntil = now + LIKELY_HOLD_MS;
    } else {
      if (likelyCharging && (int32_t)(now - likelyUntil) >= 0) {
        if (cnt60 == WIN_60S && dV60_mV <= 0.0f)
          likelyCharging = false;
        else if (cnt15 == WIN_15S && dV15_mV <= 0.0f)
          likelyCharging = false;
      }
    }
  } else {
    if (likelyCharging && (int32_t)(now - likelyUntil) >= 0)
      likelyCharging = false;
  }

  // ================= 5) allowInference =================
  const bool allowInference =
      !blockedByBoot && !blockedByEvent &&
      (stable5s || likelyCharging || chargingEdgeVisual);

  // ================= 6) Evaluación periódica (score) =================
  // Salida dura por caída 5s: usar MED rápida para no “perdonar” una caída real
  if (cnt5e == WIN_5S && dV5_edge_mV <= FALL_5S_EXIT_mV) {
    chargeScore = 0;
    if (chargeState == CHARGING)
      chargeState = NOT_CHARGING;

    likelyCharging = false;
    likelyUntil = 0;
    chargingEdgeVisual = false;
    edgeUntil = 0;

    vetoByAbsoluteDrop = false;
    chargeEntryLockRef_mV = 0;
  }

  if ((uint32_t)(now - lastEval) >= EVAL_PERIOD_MS) {
    lastEval += EVAL_PERIOD_MS;

    if (allowInference) {
      if (cnt60 == WIN_60S && dV60_mV <= FALL_60S_EXIT_mV) {
        chargeScore -= 2;
      } else {
        const bool have60 = (cnt60 == WIN_60S);
        const bool have180 = (cnt180 == WIN_180S);

        if (have60 && have180 && dV60_mV >= RISE_60S_mV_ENTER &&
            dV180_mV >= RISE_180S_mV_ENTER) {
          chargeScore += 2;
          if (dV180_mV >= RISE_180S_mV_STRONG)
            chargeScore += 1;
        } else if (have60 && !have180 && dV60_mV >= RISE_60S_mV_ENTER) {
          chargeScore += 1;
        } else {
          chargeScore -= 1;
        }
      }
    } else {
      if (chargeScore > 0)
        chargeScore -= 1;
    }

    if (chargeScore < 0)
      chargeScore = 0;
    if (chargeScore > 10)
      chargeScore = 10;
  }

  // ================= 7) FSM =================
  if (chargeState != CHARGING && chargeState != CHARGED) {
    if (allowInference && chargeScore >= SCORE_ENTER) {
      chargeState = CHARGING;
    }
  } else if (chargeState == CHARGING) {
    if (chargeScore <= SCORE_EXIT)
      chargeState = NOT_CHARGING;
  }

  // CHARGED por VBAT alto y estable
  if (chargeState == CHARGING) {
    if (vBatFiltered >= VBAT_CV_ENTER) {
      if (chargedStableSince == 0)
        chargedStableSince = now;
      if ((uint32_t)(now - chargedStableSince) >= CHARGED_STABLE_MS) {
        chargeState = CHARGED;
        chargedStableSince = 0;
      }
    } else {
      chargedStableSince = 0;
    }
  } else {
    chargedStableSince = 0;
  }
  if (chargeState == CHARGED && vBatFiltered < VBAT_CV_EXIT) {
    chargeState = NOT_CHARGING;
  }

  // ================= 7.5) Veto por caída absoluta bajo “carga”
  // =================
  const bool uiChargingNow = (chargeState == CHARGING) ||
                             (chargeState == CHARGED) || likelyCharging ||
                             chargingEdgeVisual;

  if (uiChargingNow && chargeEntryLockRef_mV == 0) {
    chargeEntryLockRef_mV = (int16_t)(vBatLock * 1000.0f + 0.5f);
  }
  if (!uiChargingNow) {
    vetoByAbsoluteDrop = false;
    chargeEntryLockRef_mV = 0;
  } else {
    const int16_t lockNow_mV = (int16_t)(vBatLock * 1000.0f + 0.5f);
    if (!vetoByAbsoluteDrop &&
        (lockNow_mV <= (int16_t)(chargeEntryLockRef_mV - 80))) {
      vetoByAbsoluteDrop = true;

      likelyCharging = false;
      likelyUntil = 0;
      chargingEdgeVisual = false;
      edgeUntil = 0;
      chargeScore = 0;
      chargeState = NOT_CHARGING;
    }
  }

  // ================= 8) Porcentajes / iconos =================
  batteryPercentage = constrain((vBatFiltered - VBAT_EMPTY) /
                                    (VBAT_MAX_VISUAL - VBAT_EMPTY) * 100.0f,
                                0.0f, 100.0f);

  batteryVoltagePercent = mapVoltageToPercent(vBatFiltered);

  const float lockPct = constrain((vBatLock - VBAT_EMPTY) /
                                      (VBAT_MAX_VISUAL - VBAT_EMPTY) * 100.0f,
                                  0.0f, 100.0f);
  (void)lockPct;

  const bool visualCharging = uiChargingNow && !vetoByAbsoluteDrop;
  batteryVisualPercentage =
      visualCharging ? BAT_VIS_CHARGING : batteryPercentage;

  // ================= 9) Lock crítico / warning =================
  if (!criticalBatteryLock) {
    if (batteryVoltagePercent < BATTERY_LOCK_THRESHOLD && !visualCharging) {
      criticalBatteryLock = true;
      showCriticalBatteryMessage();
    }
  } else {
    if (batteryVoltagePercent >= BATTERY_UNLOCK_THRESHOLD || visualCharging) {
      criticalBatteryLock = false;
    }
  }

  if (!criticalBatteryLock && batteryVoltagePercent < BATTERY_WARN_THRESHOLD &&
      !visualCharging) {
    constexpr uint32_t warnInterval = 15000UL;
    if ((uint32_t)(now - lastBatteryWarningTime) >= warnInterval) {
      doitPlayer.play_file(99, 8);
      lastBatteryWarningTime = now;
    }
  }

  // ================= 10) Icono =================
  int newIconIndex = lastBatteryIconIndex;

  if (visualCharging) {
    newIconIndex = -1;
  } else {
    const float iconPercent = batteryVoltagePercent;
    if (iconPercent > 60.0f)
      newIconIndex = 1;
    else if (iconPercent > 25.0f)
      newIconIndex = 3;
    else
      newIconIndex = 2;
  }

  static bool prevVisual = false;
  static int prevIcon = -999;
  if (visualCharging != prevVisual || newIconIndex != prevIcon) {
    if (displayOn && isInMainMenu())
      drawCurrentElement();
    prevVisual = visualCharging;
    prevIcon = newIconIndex;
    lastBatteryIconIndex = newIconIndex;
  }

#ifdef DEBUG
  // const char* st = "UNKNOWN";
  // switch (chargeState) {
  //     case NOT_CHARGING: st = "NOT_CHARGING"; break;
  //     case CHARGING:     st = "CHARGING";     break;
  //     case CHARGED:      st = "CHARGED";      break;
  //     default: break;
  // }

  // const uint32_t blockLeft =
  //     ((int32_t)(now - eventBlockUntil) < 0) ? (uint32_t)(eventBlockUntil -
  //     now) / 1000UL : 0;

  // Serial.printf(
  //     "[BATDBG] raw=%.3f med=%.3f det=%.3f filt=%.3f lockV=%.3f | "
  //     "med_mV=%d dv1s(edge)=%dmV dV5(edge)=%.1fmV | "
  //     "det_mV=%d dV5=%.1fmV dV15=%.1fmV dV60=%.1fmV dV180=%.1fmV |
  //     slope60=%.2f slope180=%.2f | " "score=%d allow=%d block=%lus | pct=%.1f
  //     vPct=%.1f | visual=%d | state=%s | icon=%d | likely=%d | edge=%d |
  //     veto=%d | loadBits=0x%02X\n", vBatRaw, vBatMedian, vBatDet,
  //     vBatFiltered, vBatLock, (int)medNow_mV, (int)dv1s_mV, dV5_edge_mV,
  //     (int)detNow_mV, dV5_mV, dV15_mV, dV60_mV, dV180_mV,
  //     slope60_mVmin, slope180_mVmin,
  //     chargeScore, allowInference ? 1 : 0,
  //     (unsigned long)blockLeft,
  //     batteryPercentage, batteryVoltagePercent,
  //     visualCharging ? 1 : 0,
  //     st,
  //     newIconIndex,
  //     likelyCharging ? 1 : 0,
  //     chargingEdgeVisual ? 1 : 0,
  //     vetoByAbsoluteDrop ? 1 : 0,
  //     loadBits
  // );
#endif
}

static bool isBootOtaRequestedByEncoderBtn() {
  pinMode(ENC_BUTTON, INPUT_PULLUP);
  delay(25); // estabilización eléctrica

  const bool p1 = (digitalRead(ENC_BUTTON) == LOW);
  delay(25); // debounce
  const bool p2 = (digitalRead(ENC_BUTTON) == LOW);

  return p1 && p2;
}

static void enterOtaModeBlocking(BOTONERA_ *element) {
  element->activarAP_OTA();

  // Bucle dedicado OTA (sin resto del sistema)
  while (true) {
    ArduinoOTA.handle();
    delay(2);
  }
}
