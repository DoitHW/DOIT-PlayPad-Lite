#pragma once
#include <FastLED.h>
#include <Frame_DMS/Frame_DMS.h> // para COLORPAD_BTNMAP
#include <vector>

// Forward declaration de COLORHANDLER_ para evitar ciclos de inclusión
class COLORHANDLER_;

// =============================================================
// Clase base de efectos dinámicos
// =============================================================
class DynamicEffect {
public:
    DynamicEffect(COLORHANDLER_& handler, int index, unsigned int updateInterval);
    virtual ~DynamicEffect();
    virtual void update() = 0;
    virtual void reset() {};

protected:
    COLORHANDLER_&  colorHandler;
    int             ledIndex;      // -1 => efecto global (si aplica)
    unsigned long   lastUpdate;
    unsigned int    interval;      // ms
};

// =============================================================
// Efecto existente (tu clásico): transición A <-> B
// =============================================================
class FadeEffect : public DynamicEffect {
public:
    // periodMs = intervalo de actualización; internamente la fase avanza a ritmo suave
    FadeEffect(COLORHANDLER_& handler, int index, CRGB colorA, CRGB colorB, unsigned int periodMs);
    void update() override;

private:
    CRGB    ca, cb;
    uint8_t t;  // fase 0..255
};

// =============================================================
// Onda (fade color->negro->color). Velocidad vía 'interval'
// =============================================================
class WaveEffect : public DynamicEffect {
public:
    WaveEffect(COLORHANDLER_& handler, int index, CRGB base, unsigned int updateMs);
    void update() override;

private:
    CRGB    color;
    uint8_t t;      // fase 0..255
};

// =============================================================
// Arcoíris en bucle por LED (HSV), ignora color base
// =============================================================
class RainbowLoopEffect : public DynamicEffect {
public:
    RainbowLoopEffect(COLORHANDLER_& handler, int index, uint8_t startHue, unsigned int updateMs);
    void update() override;

private:
    uint8_t hue;
};

// =============================================================
// Burbujeo: chisporroteos aleatorios sobre color base
//  - Si ledIndex >= 0 → actúa en un LED
//  - Si ledIndex == -1 → versión simple “global”
// =============================================================
class BubblesEffect : public DynamicEffect {
public:
    BubblesEffect(COLORHANDLER_& handler, int index, CRGB base, unsigned int updateMs, uint8_t density = 25);
    void update() override;

private:
    CRGB    color;
    uint8_t dens;   // probabilidad 0..255 por tick
};

/* ============================================================
 *    HeartbeatEffect - Latido de corazón realista
 * ============================================================ */
class HeartbeatEffect : public DynamicEffect {
private:
    CRGB color;
    uint8_t beatPhase;   // 0..5
    uint16_t beatTimer;  // ms acumulados en subfase

public:
    HeartbeatEffect(COLORHANDLER_& handler, int index, CRGB heartColor, unsigned int updateMs);
    void update() override;
    void reset() override { beatPhase = 0; beatTimer = 0; }
};

/* ============================================================
 *    AuroraEffect - Aurora boreal suave
 * ============================================================ */
class AuroraEffect : public DynamicEffect {
private:
    uint16_t phase;
    CRGB color1, color2;

public:
    AuroraEffect(COLORHANDLER_& handler, int index, CRGB c1, CRGB c2, unsigned int updateMs);
    void update() override;
    void reset() override { phase = 0; }
};

/* ============================================================
 *    StrobeEffect - Estroboscópico configurable
 * ============================================================ */
class StrobeEffect : public DynamicEffect {
private:
    CRGB color;
    uint8_t flashCount;
    uint8_t currentFlash;
    bool isOn;
    unsigned long flashDuration;
    unsigned long pauseDuration;
    unsigned long phaseTimer;   // <- necesario

public:
    StrobeEffect(COLORHANDLER_& handler, int index, CRGB strobeColor,
                 unsigned int updateMs, uint8_t flashes = 3);
    void update() override;
    void reset() override { currentFlash = 0; isOn = false; phaseTimer = 0; }
};

/* ============================================================
 *    ColorWipeEffect - Barrido progresivo de color
 * ============================================================ */
class ColorWipeEffect : public DynamicEffect {
private:
    CRGB color;
    int currentLed;
    int direction;
    bool fill;

public:
    ColorWipeEffect(COLORHANDLER_& handler, int index, CRGB wipeColor,
                    unsigned int updateMs, bool fillMode = true);
    void update() override;
    void reset() override { currentLed = 0; direction = 1; }
};

/* ============================================================
 *    TheaterChaseEffect - Persecución estilo marquesina
 * ============================================================ */
class TheaterChaseEffect : public DynamicEffect {
private:
    CRGB color;
    uint8_t spacing;
    uint8_t offset;

public:
    TheaterChaseEffect(COLORHANDLER_& handler, int index, CRGB chaseColor,
                       unsigned int updateMs, uint8_t space = 3);
    void update() override;
    void reset() override { offset = 0; }
};

// =============================================================
// Gestor de efectos (no cambio firmas públicas)
// =============================================================
class DynamicLEDManager {
public:
    DynamicLEDManager(COLORHANDLER_& handler);
    ~DynamicLEDManager();

    void addEffect(DynamicEffect* effect); // toma propiedad y lo deletea en clear/dtor
    void clearEffects();                   // borra y limpia todos los efectos
    void update();                         // llama a update() de cada efecto y hace FastLED.show()

private:
    std::vector<DynamicEffect*> effects;
    COLORHANDLER_&              colorHandler;
};

// Efecto: “persecución por ruta” (global sencillo)
// Si quieres, ponlo al final del .h junto al resto de clases.
class PathChaseEffect : public DynamicEffect {
public:
    // 'order' es un puntero a una lista de índices físicos de LED (0..8).
    // 'len' longitud de la lista. 'stepMs' intervalo entre pasos.
    PathChaseEffect(COLORHANDLER_& h, const uint8_t* order, uint8_t len,
                    CRGB color, unsigned int stepMs)
    : DynamicEffect(h, /*index*/-1, stepMs),
      seq(order), seqLen(len), col(color), pos(0) {}

    void update() override;

private:
    const uint8_t* seq;
    uint8_t        seqLen;
    CRGB           col;
    uint8_t        pos;
};

// --- Global: cometa con estela (ruta configurable) ---
class CometTrailEffect : public DynamicEffect {
public:
    CometTrailEffect(COLORHANDLER_& h, const uint8_t* order, uint8_t len,
                     CRGB color, unsigned int stepMs, uint8_t decay);
    void update() override;
private:
    const uint8_t* seq; uint8_t seqLen; CRGB col; uint8_t pos; uint8_t fade; // 0..255
};

// --- Global: theater chase ---
class TheaterChaseGlobalEffect : public DynamicEffect {
public:
    TheaterChaseGlobalEffect(COLORHANDLER_& h, CRGB color, unsigned int stepMs, uint8_t spacing);
    void update() override;
private:
    CRGB col; uint8_t off; uint8_t sp; // offset y espaciado
};

// --- Global: color wipe (orden configurable) ---
class ColorWipeGlobalEffect : public DynamicEffect {
public:
    ColorWipeGlobalEffect(COLORHANDLER_& h, const uint8_t* order, uint8_t len,
                          CRGB color, unsigned int stepMs);
    void update() override;
private:
    const uint8_t* seq; uint8_t seqLen; CRGB col; int8_t pos;
};

// --- Global: strobe ---
class StrobeGlobalEffect : public DynamicEffect {
public:
    StrobeGlobalEffect(COLORHANDLER_& h, CRGB color, unsigned int updateMs, uint8_t flashes, unsigned int gapMs);
    void update() override;
private:
    CRGB col; uint8_t burst; uint8_t count; unsigned int gap; bool onPhase;
};

// --- Global: sparkle ---
class SparkleGlobalEffect : public DynamicEffect {
public:
    SparkleGlobalEffect(COLORHANDLER_& h, CRGB base, unsigned int tickMs, uint8_t density, uint8_t fade);
    void update() override;
private:
    CRGB baseCol; uint8_t dens; uint8_t fadeAmt;
};

// --- Global: fire (flicker cálido) ---
class FireGlobalEffect : public DynamicEffect {
public:
    FireGlobalEffect(COLORHANDLER_& h, unsigned int tickMs);
    void update() override;
private:
    uint8_t t;
};

// --- Global: aurora (gradiente frío en movimiento) ---
class AuroraGlobalEffect : public DynamicEffect {
public:
    AuroraGlobalEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs);
    void update() override;
private:
    uint8_t hue; CRGB tintCol;
};

// --- Global: plasma noise ---
class PlasmaNoiseEffect : public DynamicEffect {
public:
    PlasmaNoiseEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs);
    void update() override;
private:
    uint16_t z; CRGB tintCol;
};

class CandleFlickerEffect : public DynamicEffect {
public:
    CandleFlickerEffect(COLORHANDLER_& h, int index, CRGB warm, unsigned int tickMs)
    : DynamicEffect(h, index, tickMs), base(warm), phase(0) {}
    void update() override;
private:
    CRGB base;
    uint8_t phase;
};

class SparkleLocalEffect : public DynamicEffect {
public:
    SparkleLocalEffect(COLORHANDLER_& h, int index, CRGB tint, unsigned int tickMs)
    : DynamicEffect(h, index, tickMs), base(tint), hold(0) {}
    void update() override;
private:
    CRGB base;
    uint8_t hold; // frames a mantener el destello
};

class CometLocalEffect : public DynamicEffect {
public:
    CometLocalEffect(COLORHANDLER_& h, int index, CRGB head, unsigned int periodMs, uint8_t decay)
    : DynamicEffect(h, index, periodMs), headColor(head), t(0), decayPct(decay) {}
    void update() override;
private:
    CRGB headColor;
    uint8_t t;        // fase 0..255
    uint8_t decayPct; // decaimiento del pulso
};

class PlasmaLocalEffect : public DynamicEffect {
public:
    PlasmaLocalEffect(COLORHANDLER_& h, int index, CRGB tint, unsigned int tickMs)
    : DynamicEffect(h, index, tickMs), base(tint), t(0) {}
    void update() override;
private:
    CRGB base;
    uint8_t t;
};

class BubblesGlobalEffect : public DynamicEffect {
public:
    BubblesGlobalEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs, uint8_t density, uint8_t fade)
    : DynamicEffect(h, -1, tickMs), base(tint), dens(density), fadeAmt(fade) {}
    void update() override;
private:
    CRGB base;
    uint8_t dens;
    uint8_t fadeAmt;
};

class LoadingGlobalEffect : public DynamicEffect {
public:
    LoadingGlobalEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs, uint8_t trail, uint8_t baseDim)
    : DynamicEffect(h, -1, tickMs), base(tint), pos(0), trailLen(trail), ambientDim(baseDim) {}

    void update() override;

private:
    CRGB base;           // color del punto/cola
    int  pos;            // posición actual del “cabeza”
    uint8_t trailLen;    // longitud de cola
    uint8_t ambientDim;  // intensidad ambiente de fondo (0..255)
};

class LoadingEffect : public DynamicEffect {
public:
    LoadingEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs)
    : DynamicEffect(h, -1, tickMs), base(tint), step(0), prevLogical(255) {}

    void update() override;

    // Configura el path global (válido para la única instancia activa)
    static void configurePath(const uint8_t* path, uint8_t len);

private:
    CRGB base;
    uint8_t step;
    uint8_t prevLogical; // último LED encendido (para apagar solo ese)

    // Path global y su longitud (compartidos por la única instancia activa)
    static uint8_t kPath[16];   // capacidad máxima (ajústala a tus necesidades)
    static uint8_t kPathLen;    // longitud efectiva del path
};



// Declaración externa del manager (si ya la usas en otros módulos)
extern DynamicLEDManager ledManager;