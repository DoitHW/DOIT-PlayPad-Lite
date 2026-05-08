#include <Colors_DMS/Color_DMS.h>
#include <defines_DMS/defines_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <Arduino.h>
#include <stdint.h>
#include <map>
#include <math.h>
#include <FastLED.h>
#include <DynamicLEDManager_DMS/DynamicLEDManager_DMS.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <encoder_handler/encoder_handler.h>
#include <microphone_DMS/microphone_DMS.h>
#include <play_DMS/play_DMS.h>
#include <RelayManager_DMS/RelayStateManager.h>
#include <Pulsadores_handler/Pulsadores_handler.h>


#ifdef MIC
  extern MICROPHONE_ doitMic;
  extern int micSensRuntime;
#endif

extern ELEMENT_ *element;


bool COLORHANDLER_::color_mix_handler(int color1, int color2, byte *resultado) {
 
  static std::map <std::pair<int, int>, int> colorMixMap = {
    {{WHITE, YELLOW},       CREMA},
    {{YELLOW, WHITE},       CREMA},
    {{WHITE, RED},          ROSA},
    {{RED, WHITE},          ROSA},
    {{WHITE, ORANGE},       SALMON},
    {{ORANGE, WHITE},       SALMON},
    {{WHITE, VIOLET},       LILA},
    {{VIOLET, WHITE},       LILA},
    {{WHITE, BLUE},         CELESTE_CLARO},
    {{BLUE, WHITE},         CELESTE_CLARO},
    {{WHITE, LIGHT_BLUE},   TURQUESA},
    {{LIGHT_BLUE, WHITE},   TURQUESA},
    {{WHITE, GREEN},        VERDE_CLARO},
    {{GREEN, WHITE},        VERDE_CLARO},
    {{YELLOW, RED},         NARANJA},
    {{RED, YELLOW},         NARANJA},
    {{YELLOW, VIOLET},      ROSA_OSCURO},
    {{VIOLET, YELLOW},      ROSA_OSCURO},
    {{YELLOW, BLUE},        VERDE_OLIVA},
    {{BLUE, YELLOW},        VERDE_OLIVA},
    {{YELLOW, LIGHT_BLUE},  VERDE_LIMA},
    {{LIGHT_BLUE, YELLOW},  VERDE_LIMA},
    {{YELLOW, GREEN},       VERDE_AMARILLO},
    {{GREEN, YELLOW},       VERDE_AMARILLO},
    {{RED, VIOLET},         MAGENTA},
    {{VIOLET, RED},         MAGENTA},
    {{RED, BLUE},           MORADO},
    {{BLUE, RED},           MORADO},
    {{RED, LIGHT_BLUE},     VERDE_AZULADO},
    {{LIGHT_BLUE, RED},     VERDE_AZULADO},
    {{RED, GREEN},          MARRON},
    {{GREEN, RED},          MARRON},
    {{VIOLET, BLUE},        INDIGO},
    {{BLUE, VIOLET},        INDIGO},
    {{VIOLET, LIGHT_BLUE},  AZUL_VERDOSO},
    {{LIGHT_BLUE, VIOLET},  AZUL_VERDOSO},
    {{VIOLET, GREEN},       ESMERALDA},
    {{GREEN, VIOLET},       ESMERALDA},
    {{BLUE, LIGHT_BLUE},    CYAN},
    {{LIGHT_BLUE, BLUE},    CYAN},
    {{BLUE, GREEN},         VERDE_AZULADO},
    {{GREEN, BLUE},         VERDE_AZULADO},
    {{LIGHT_BLUE, GREEN},   VERDE_MENTA},
    {{GREEN, LIGHT_BLUE},   VERDE_MENTA},
    {{ORANGE, YELLOW},      AMARILLO_ANARANJADO},
    {{YELLOW, ORANGE},      AMARILLO_ANARANJADO},
    {{ORANGE, RED},         ROJO_ANARANJADO},
    {{RED, ORANGE},         ROJO_ANARANJADO},
    {{ORANGE, VIOLET},      ROSA_FUERTE},
    {{VIOLET, ORANGE},      ROSA_FUERTE},
    {{ORANGE, LIGHT_BLUE},  VERDE_GRISACEO},
    {{LIGHT_BLUE, ORANGE},  VERDE_GRISACEO},
    {{ORANGE, BLUE},        MARRON_OSCURO},
    {{BLUE, ORANGE},        MARRON_OSCURO},
    {{ORANGE, GREEN},       VERDE_OLIVA_CLARO},
    {{GREEN, ORANGE},       VERDE_OLIVA_CLARO}
 
  };

    auto it = colorMixMap.find({color1, color2});
    if (it != colorMixMap.end()) {
        *resultado = it->second;
        return true;
    } else {
        return false;
    }
}

COLORHANDLER_::COLORHANDLER_() : leds(nullptr), numLeds(0){}

void COLORHANDLER_::begin(int numLeds) {
    #if  defined (BOTONERA)
      constexpr byte dataPin = BOTONERA_DATA_PIN;
    #endif

    this->numLeds = numLeds;
    if (this->leds != nullptr) {
        delete[] leds;  // Liberar memoria previa si ya estaba asignada
    }
    this->leds = new CRGB[numLeds];
    
    FastLED.addLeds<WS2812, dataPin, RGB>(leds, numLeds);
    FastLED.setBrightness(255);
}


#ifdef NOELEM
void COLORHANDLER_::welcomeEffect() {
 
  const int totalLEDs = NUM_LEDS;  // Asumimos 9 LEDs

  // 1. Definir los colores finales para cada LED.
  //    LED 0 se mantiene apagado.
    CRGB targetColors[totalLEDs] = {
        CRGB::Black,      // LED 0: apagado
        CRGB::White,      // LED 1: blanco
        CRGB::Red,        // LED 2: rojo
        CRGB::Cyan,       // LED 3: celeste
        CRGB::Yellow,     // LED 4: amarillo
        CRGB(0xFF, 0x59, 0x00), // LED 5: naranja
        CRGB::Green,      // LED 6: verde
        CRGB::Purple,     // LED 7: violeta
        CRGB::Blue        // LED 8: azul
    };

  // 2. Limpiar la tira: apagar todos los LEDs.
  fill_solid(this->leds, totalLEDs, CRGB::Black);
  FastLED.show();
  delay(50);  // Breve retardo para asegurar la actualización

  // 3. Configurar el efecto "breath": número de pasos y retardo entre ellos.
  const int steps = 50;        // Número de pasos en el fade-in
  const int stepDelay = 100;   // Retardo en milisegundos entre cada paso

  // 4. Para cada paso, se calcula el nivel de brillo (de 0 a 255)
  //    y se actualiza cada LED (excepto el 0) con su color escalado.
  for (int step = 0; step <= steps; step++) {
      uint8_t brightness = map(step, 0, steps, 0, 255);
      for (int i = 0; i < totalLEDs; i++) {
          if (i == 0) {
              this->leds[i] = CRGB::Black;
          } else {
              // Se copia el color objetivo y se escala su brillo
              CRGB c = targetColors[i];
              c.nscale8_video(brightness);
              this->leds[i] = c;
          }
      }
      FastLED.show();
      delay(stepDelay);
  }

  // 5. Asegurar que el estado final es el brillo completo.
  for (int i = 0; i < totalLEDs; i++) {
      this->leds[i] = targetColors[i];
  }
  FastLED.show();
  
}
#endif


void COLORHANDLER_::setAllButtonsActive(bool val) {
    bool mask[9];
    for (int i = 0; i < 9; ++i) mask[i] = val;
    PulsadoresHandler::setButtonActiveMask(mask);
}

void COLORHANDLER_::syncButtonsWithLEDs() {
    bool mask[9] = {false,false,false,false,false,false,false,false,false};
    const int n = min(9, this->numLeds);
    for (int i = 0; i < n; ++i) {
        mask[i] = !isBlack(this->leds[i]);
    }
    PulsadoresHandler::setButtonActiveMask(mask);
}

void COLORHANDLER_::setPatternBotonera(byte mode, DynamicLEDManager& ledManager) {
  ledManager.clearEffects(); // Limpiar efectos dinámicos previos
  /*────────────────  COMUNICADOR · elemento sólo-relé  ────────────────*/
  if (currentFile == "Comunicador") {
      const TARGETNS& activeNS = ::communicatorActiveNS;
      if (memcmp(&activeNS, &NS_ZERO, sizeof(TARGETNS)) != 0) {
          uint8_t cfg[2] = {0};
          if (RelayStateManager::getModeConfigForNS(activeNS, cfg)) {
              bool hasCol = getModeFlag(cfg, HAS_BASIC_COLOR) ||
                            getModeFlag(cfg, HAS_ADVANCED_COLOR);
              bool hasRel = getModeFlag(cfg, HAS_RELAY);

              if (!hasCol && hasRel) {   // SÓLO RELÉ
                  fill_solid(leds, NUM_LEDS, CRGB::Black);
                  ledManager.clearEffects();
                  ledManager.addEffect(new FadeEffect(*this, 0, CRGB::Blue, CRGB::Cyan, 50));
                  leds[8] = CRGB::Blue;
                  leds[0] = CRGB::Blue;
                  this->syncButtonsWithLEDs();
                  FastLED.show();
                  return;
              }
          }
      }
  }


  if (mode >= 16) {
#ifdef DEBUG
      DEBUG__________printf("⚠️ Modo inválido: %d\n", mode);
#endif
      return;
  }

  byte modeConfig[2] = {0};
  if (!getModeConfig(currentFile, mode, modeConfig)) {
#ifdef DEBUG
      DEBUG__________ln("⚠️ No se pudo obtener la configuración del modo.");
#endif
      return;
  }

#ifdef DEBUG
  DEBUG__________printf("Mode %d - Byte Config: 0x%02X%02X\n", mode, modeConfig[0], modeConfig[1]);
#endif

  // --- FICHAS ---
  if (currentFile == "Fichas") {
      fill_solid(leds, NUM_LEDS, CRGB::Black);

      if (mode == 0) {
          ledManager.addEffect(new FadeEffect(*this, 0, CRGB::Blue, CRGB::Cyan, 50));
      } else if (mode == 1 || mode == 2) {
          ledManager.addEffect(new FadeEffect(*this, 0, CRGB::Blue, CRGB::Cyan, 50));
          leds[8] = CRGB::White;
      }
      this->syncButtonsWithLEDs();
      FastLED.show();
      return;
  }

  // --- FLAGS DE RELÉ ---
  bool hasRelay    = getModeFlag(modeConfig, HAS_RELAY);
  bool hasRelayN1  = getModeFlag(modeConfig, HAS_RELAY_N1);
  bool hasRelayN2  = getModeFlag(modeConfig, HAS_RELAY_N2);

  int relayCount = 0;

  if (!hasRelay && hasRelayN1 && hasRelayN2) {
      relayCount = 4; // caso aromaterapia
  } else if (hasRelay) {
      if (!hasRelayN1 && !hasRelayN2)     relayCount = 1;
      else if (!hasRelayN1 && hasRelayN2) relayCount = 2;
      else if (hasRelayN1 && !hasRelayN2) relayCount = 3;
      else if (hasRelayN1 && hasRelayN2)  relayCount = 4;
  }


  bool isMultiRelay = relayCount > 1;
  bool isAromaterapia = (!hasRelay && hasRelayN1 && hasRelayN2);

  // --- COLORES DE LOS BOTONES (excepto LED 0) ---
  if (getModeFlag(modeConfig, HAS_PATTERNS)) {
      fill_solid(leds + 1, NUM_LEDS - 1, CRGB::White);
  } else if (getModeFlag(modeConfig, HAS_PASSIVE)) {
      fill_solid(leds + 1, NUM_LEDS - 1, CRGB::Black);
      leds[8] = CRGB::White;
  } else if (getModeFlag(modeConfig, HAS_BASIC_COLOR) || getModeFlag(modeConfig, HAS_ADVANCED_COLOR)) {
      CRGB colorMap[] = {
          CRGB::Black, CRGB::White, CRGB::Red, CRGB::Cyan,
          CRGB::Yellow, CRGB(0xFF, 0x59, 0x00), CRGB::Green,
          CRGB(0xFF, 0x00, 0xD2), CRGB::Blue
      };
      for (int i = 1; i < NUM_LEDS; i++) {
          leds[i] = colorMap[i];
      }
  } else {
      fill_solid(leds + 1, NUM_LEDS - 1, CRGB::Black);
  }
  DEBUG__________ln("relayCount: " + String(relayCount));
  // --- VISUALIZACIÓN DE RELÉS ---
  if (relayCount == 1 && hasRelay) {
      // Caso: solo un relé → usar LED 0 con efecto
      ledManager.addEffect(new FadeEffect(*this, 0, CRGB::Blue, CRGB::Cyan, 50));
  } else if (isMultiRelay || isAromaterapia) {
      // Caso: 2 o más relés → usar LEDs 9, 7, 5, 3 en blanco
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      if (relayCount >= 1) leds[8] = CRGB::Blue;
      if (relayCount >= 2) leds[6] = CRGB::Green;
      if (relayCount >= 3) leds[4] = CRGB::Yellow;
      if (relayCount >= 4) leds[2] = CRGB::Red;

  } else {
      leds[0] = CRGB::Black;
  }
  
  this->syncButtonsWithLEDs();
  FastLED.show();
}

void COLORHANDLER_::mapCognitiveLEDs() {
  DEBUG__________ln("Mapeo de LEDs cognitivos: ");
  colorHandler.setPatternBotonera(0, ledManager); // Desactiva todos
  for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
  }
  leds[2] = CRGB::Red;
  leds[4] = CRGB::Yellow;
  leds[6] = CRGB::Green;
  leds[8] = CRGB::Blue;
  FastLED.show();
}





