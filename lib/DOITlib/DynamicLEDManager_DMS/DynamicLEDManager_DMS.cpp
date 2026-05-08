#include "DynamicLEDManager_DMS.h"
#include <Colors_DMS/Color_DMS.h>  
#include <Frame_DMS/Frame_DMS.h>
#include <FastLED.h>


namespace {
// Utilidades locales (nombres distintos para evitar colisiones)
static uint8_t ease8InOutCubic_u(uint8_t x) {
    if (x < 128) {
        uint16_t xx = x * 2;
        return (xx * xx * xx) / 16384;
    } else {
        uint16_t xx = (255 - x) * 2;
        return 255 - ((xx * xx * xx) / 16384);
    }
}

static CRGB lerpColor_u(const CRGB& a, const CRGB& b, uint8_t t) {
    return CRGB(
        lerp8by8(a.r, b.r, t),
        lerp8by8(a.g, b.g, t),
        lerp8by8(a.b, b.b, t)
    );
}

inline void writeSingleOrAll(COLORHANDLER_& h, int ledIndex, const CRGB& c) {
    if (!h.leds || h.numLeds <= 0) return;
    if (ledIndex >= 0 && ledIndex < h.numLeds) {
        h.leds[ledIndex] = c;
    } else {
        fill_solid(h.leds, h.numLeds, c);
    }
}
} // namespace

/* ============================================================
 *    DynamicEffect (base)
 * ============================================================ */
DynamicEffect::DynamicEffect(COLORHANDLER_& handler, int index, unsigned int updateInterval)
    : colorHandler(handler), ledIndex(index), lastUpdate(0), interval(updateInterval)
{
}

DynamicEffect::~DynamicEffect() {}

/* ============================================================
 *    FadeEffect  (A <-> B, tu efecto clásico)
 * ============================================================ */
FadeEffect::FadeEffect(COLORHANDLER_& handler, int index, CRGB colorA, CRGB colorB, unsigned int periodMs)
    : DynamicEffect(handler, index, periodMs), ca(colorA), cb(colorB), t(0)
{
}

void FadeEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    // Onda triangular sobre 0..255
    t = uint8_t(t + 3);
    uint8_t level = (t < 128) ? (t << 1) : ((255 - t) << 1); // 0..255..0

    // Interpolación entre A y B con 'level'
    auto mix8 = [&](uint8_t a, uint8_t b, uint8_t lv) -> uint8_t {
        // lv 0 => a, lv 255 => b (aprox)
        uint16_t x = (uint16_t(a) * (255 - lv)) + (uint16_t(b) * lv);
        return uint8_t(x >> 8);
    };

    CRGB out(
        mix8(ca.r, cb.r, level),
        mix8(ca.g, cb.g, level),
        mix8(ca.b, cb.b, level)
    );

    if (ledIndex >= 0 && ledIndex < colorHandler.numLeds && colorHandler.leds) {
        colorHandler.leds[ledIndex] = out;
    }
}

/* ============================================================
 *    WaveEffect  (color ↔ negro)
 * ============================================================ */
WaveEffect::WaveEffect(COLORHANDLER_& handler, int index, CRGB base, unsigned int updateMs)
    : DynamicEffect(handler, index, updateMs), color(base), t(0)
{
}

void WaveEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    t = uint8_t(t + 5); // velocidad por tick
    uint8_t level = (t < 128) ? (t << 1) : ((255 - t) << 1); // 0..255..0

    CRGB out(
        (uint16_t(color.r) * level) >> 8,
        (uint16_t(color.g) * level) >> 8,
        (uint16_t(color.b) * level) >> 8
    );

    if (ledIndex >= 0 && ledIndex < colorHandler.numLeds && colorHandler.leds) {
        colorHandler.leds[ledIndex] = out;
    }
}

/* ============================================================
 *    RainbowLoopEffect (HSV hue++ por LED)
 * ============================================================ */
RainbowLoopEffect::RainbowLoopEffect(COLORHANDLER_& handler, int index, uint8_t startHue, unsigned int updateMs)
    : DynamicEffect(handler, index, updateMs), hue(startHue)
{
}

void RainbowLoopEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    hue += 2; // giro del color
    if (ledIndex >= 0 && ledIndex < colorHandler.numLeds && colorHandler.leds) {
        colorHandler.leds[ledIndex] = CHSV(hue, 255, 255);
    }
}

/* ============================================================
 *    BubblesEffect  (destellos aleatorios)
 * ============================================================ */
BubblesEffect::BubblesEffect(COLORHANDLER_& handler, int index, CRGB base, unsigned int updateMs, uint8_t density)
    : DynamicEffect(handler, index, updateMs), color(base), dens(density)
{
}

void BubblesEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    auto spark = [&](int idx){
        // chispa: levantar brillo hacia blanco mezclado con base
        CRGB mix = color;
        mix.r = qadd8(mix.r, 255 - (mix.r >> 1));
        mix.g = qadd8(mix.g, 255 - (mix.g >> 1));
        mix.b = qadd8(mix.b, 255 - (mix.b >> 1));
        colorHandler.leds[idx] = mix;
    };

    if (!colorHandler.leds || colorHandler.numLeds <= 0) return;

    if (ledIndex >= 0) {
        // Modo local: chispa ocasional en un LED
        if (random8() < dens) spark(ledIndex);
        else                  colorHandler.leds[ledIndex] = color; // reposo
    } else {
        // Modo global simple: unas pocas chispas dispersas
        int bursts = max(1, colorHandler.numLeds / 6);
        for (int n = 0; n < bursts; ++n) {
            if (random8() < dens) {
                int idx = random8() % colorHandler.numLeds;
                spark(idx);
            }
        }
    }
}

/* ============================================================
 *    HeartbeatEffect - Latido de corazón
 * ============================================================ */
 HeartbeatEffect::HeartbeatEffect(COLORHANDLER_& handler, int index, CRGB heartColor, unsigned int updateMs)
: DynamicEffect(handler, index, updateMs), color(heartColor), beatPhase(0), beatTimer(0) {}

void HeartbeatEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    uint16_t dt = (uint16_t)(now - lastUpdate);
    lastUpdate = now;

    beatTimer += dt;

    // Duraciones de sub-fases (ms)
    const uint16_t T_UP1   = 80;
    const uint16_t T_DOWN1 = 120;
    const uint16_t T_PAUSE = 80;
    const uint16_t T_UP2   = 60;
    const uint16_t T_DOWN2 = 140;
    const uint16_t T_GAP   = 400;

    uint8_t b = 0;

    switch (beatPhase) {
        case 0: // subida 1
            if (beatTimer >= T_UP1) { beatTimer = 0; beatPhase = 1; }
            b = ease8InOutCubic_u(scale8((uint8_t)map(beatTimer, 0, T_UP1, 0, 255), 255));
            break;
        case 1: // bajada 1
            if (beatTimer >= T_DOWN1) { beatTimer = 0; beatPhase = 2; }
            b = 255 - ease8InOutCubic_u(scale8((uint8_t)map(beatTimer, 0, T_DOWN1, 0, 255), 255));
            b = std::max<uint8_t>(b, 40);
            break;
        case 2: // pausa
            if (beatTimer >= T_PAUSE) { beatTimer = 0; beatPhase = 3; }
            b = 30;
            break;
        case 3: // subida 2 (más corta)
            if (beatTimer >= T_UP2) { beatTimer = 0; beatPhase = 4; }
            b = ease8InOutCubic_u(scale8((uint8_t)map(beatTimer, 0, T_UP2, 0, 255), 255));
            b = scale8(b, 200);
            break;
        case 4: // bajada 2
            if (beatTimer >= T_DOWN2) { beatTimer = 0; beatPhase = 5; }
            b = 200 - ease8InOutCubic_u(scale8((uint8_t)map(beatTimer, 0, T_DOWN2, 0, 255), 255));
            b = std::max<uint8_t>(b, 20);
            break;
        default: // 5: pausa larga
            if (beatTimer >= T_GAP) { beatTimer = 0; beatPhase = 0; }
            b = 15;
            break;
    }

    CRGB out = color; out.nscale8(b);
    writeSingleOrAll(colorHandler, ledIndex, out);
}


/* ============================================================
 *    Aurora - Aurora boreal
 * ============================================================ */
AuroraEffect::AuroraEffect(COLORHANDLER_& handler, int index, CRGB c1, CRGB c2, unsigned int updateMs)
: DynamicEffect(handler, index, updateMs), phase(0), color1(c1), color2(c2) {}

void AuroraEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds <= 0) return;

    phase += 23; // velocidad moderada

    if (ledIndex >= 0 && ledIndex < colorHandler.numLeds) {
        uint8_t t = (sin8(phase) + cos8(phase / 2)) / 2;
        CRGB c = lerpColor_u(color1, color2, t);
        colorHandler.leds[ledIndex] = c;
        return;
    }

    // Tira completa: ondas superpuestas que se deslizan
    for (int i = 0; i < colorHandler.numLeds; ++i) {
        uint8_t s1 = sin8(phase + i * 7);     // onda lenta
        uint8_t s2 = cos8((phase / 2) + i * 13); // componente secundaria
        uint8_t t  = (uint16_t(s1) * 3 + uint16_t(s2)) >> 2; // mezcla ponderada
        // Suavizado adicional
        t = ease8InOutCubic_u(t);
        colorHandler.leds[i] = lerpColor_u(color1, color2, t);
    }
}

/* ============================================================
 *    STROBE - Estroboscópico
 * ============================================================ */
 StrobeEffect::StrobeEffect(COLORHANDLER_& handler, int index, CRGB strobeColor, 
                           unsigned int updateMs, uint8_t flashes)
: DynamicEffect(handler, index, updateMs),
  color(strobeColor),
  flashCount(std::max<uint8_t>(1, flashes)),
  currentFlash(0),
  isOn(false),
  flashDuration(std::max<unsigned long>(updateMs, 20UL)), // destello corto
  pauseDuration(updateMs * 6UL), // pausa entre ráfagas
  phaseTimer(0) {}

void StrobeEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    unsigned long dt = now - lastUpdate;
    lastUpdate = now;

    phaseTimer += dt;

    if (currentFlash < flashCount) {
        // En secuencia de destellos
        if (!isOn) {
            // Encender flash
            writeSingleOrAll(colorHandler, ledIndex, color);
            isOn = true;
            phaseTimer = 0;
        } else {
            // Mantener encendido hasta flashDuration
            if (phaseTimer >= flashDuration) {
                // Apagar y preparar el siguiente
                writeSingleOrAll(colorHandler, ledIndex, CRGB::Black);
                isOn = false;
                currentFlash++;
                phaseTimer = 0;
            }
        }
    } else {
        // Pausa entre ráfagas
        if (phaseTimer >= pauseDuration) {
            currentFlash = 0;
            isOn = false;
            phaseTimer = 0;
        } else {
            // Asegurar apagado durante la pausa
            writeSingleOrAll(colorHandler, ledIndex, CRGB::Black);
        }
    }
}
 /* ============================================================
 *    COLOR_WIPE - Barrido de color
 * ============================================================ */
 ColorWipeEffect::ColorWipeEffect(COLORHANDLER_& handler, int index, CRGB wipeColor, 
                                 unsigned int updateMs, bool fillMode)
: DynamicEffect(handler, index, updateMs), color(wipeColor), currentLed(0), direction(1), fill(fillMode) {}

void ColorWipeEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds <= 0) return;

    if (ledIndex >= 0 && ledIndex < colorHandler.numLeds) {
        // Caso LED único: simple pulso de encendido secuencial (on/off)
        static bool on = false;
        colorHandler.leds[ledIndex] = on ? color : CRGB::Black;
        on = !on;
        return;
    }

    if (fill) {
        // Pintado progresivo
        // Si es el primer LED, limpiar al reiniciar ciclo
        if (currentLed == 0) fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);

        if (currentLed < colorHandler.numLeds) {
            colorHandler.leds[currentLed] = color;
            currentLed++;
        } else {
            // Lleno -> limpiar y reiniciar
            fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);
            currentLed = 0;
        }
    } else {
        // Pixel viajero
        fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);
        colorHandler.leds[currentLed] = color;

        currentLed += direction;
        if (currentLed >= colorHandler.numLeds) currentLed = 0;
        if (currentLed < 0) currentLed = colorHandler.numLeds - 1;
    }
}

 /* ============================================================
 *    THEATER_CHASE - Persecución teatral
 * ============================================================ */
TheaterChaseEffect::TheaterChaseEffect(COLORHANDLER_& handler, int index, CRGB chaseColor, 
                                       unsigned int updateMs, uint8_t space)
: DynamicEffect(handler, index, updateMs), color(chaseColor), spacing(std::max<uint8_t>(2, space)), offset(0) {}

void TheaterChaseEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds <= 0) return;

    if (ledIndex >= 0 && ledIndex < colorHandler.numLeds) {
        // Para un único LED, alterna on/off con patrón
        static bool on = false;
        colorHandler.leds[ledIndex] = on ? color : CRGB::Black;
        on = !on;
        return;
    }

    fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);

    for (int i = offset; i < colorHandler.numLeds; i += spacing) {
        colorHandler.leds[i] = color;
    }

    offset++;
    if (offset >= spacing) offset = 0;
}

/* ============================================================
 *    DynamicLEDManager
 * ============================================================ */
DynamicLEDManager::DynamicLEDManager(COLORHANDLER_& handler)
    : colorHandler(handler)
{
}

DynamicLEDManager::~DynamicLEDManager() {
    clearEffects();
}

void DynamicLEDManager::addEffect(DynamicEffect* effect) {
    if (!effect) return;
    effects.push_back(effect);
}

void DynamicLEDManager::clearEffects() {
    for (auto* e : effects) delete e;
    effects.clear();
}

void DynamicLEDManager::update() {
    // Actualizar todos los efectos
    for (auto* e : effects) {
        if (e) e->update();
    }
    // Render
    if (colorHandler.leds) {
        FastLED.show();
    }
}

void PathChaseEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds == 0 || !seq || seqLen == 0)
        return;

    // Apaga todo y enciende el LED de la posición actual de la ruta
    fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);

    uint8_t idx = seq[pos % seqLen];
    if (idx < colorHandler.numLeds) {
        colorHandler.leds[idx] = col;
    }

    pos = (pos + 1) % seqLen;
}


/* ========== CometTrailEffect ========== */
CometTrailEffect::CometTrailEffect(COLORHANDLER_& h, const uint8_t* order, uint8_t len,
                                   CRGB color, unsigned int stepMs, uint8_t decay)
: DynamicEffect(h, -1, stepMs), seq(order), seqLen(len), col(color), pos(0), fade(decay) {}

void CometTrailEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || !seq || seqLen == 0) return;

    // Estela: desvanecer todo
    fadeToBlackBy(colorHandler.leds, colorHandler.numLeds, fade);

    // Cabeza del cometa
    uint8_t idx = seq[pos % seqLen];
    if (idx < colorHandler.numLeds) {
        // sumamos color para intensificar sobre la estela
        colorHandler.leds[idx] += col;
    }
    pos = (pos + 1) % seqLen;
}

/* ========== TheaterChaseGlobalEffect ========== */
TheaterChaseGlobalEffect::TheaterChaseGlobalEffect(COLORHANDLER_& h, CRGB color, unsigned int stepMs, uint8_t spacing)
: DynamicEffect(h, -1, stepMs), col(color), off(0), sp(max<uint8_t>(1, spacing)) {}

void TheaterChaseGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);
    for (int i = off; i < colorHandler.numLeds; i += sp) {
        colorHandler.leds[i] = col;
    }
    off = (off + 1) % sp;
}

/* ========== ColorWipeGlobalEffect ========== */
ColorWipeGlobalEffect::ColorWipeGlobalEffect(COLORHANDLER_& h, const uint8_t* order, uint8_t len,
                                             CRGB color, unsigned int stepMs)
: DynamicEffect(h, -1, stepMs), seq(order), seqLen(len), col(color), pos(-1) {}

void ColorWipeGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || !seq || seqLen == 0) return;

    pos = (pos + 1) % (seqLen + 1); // incluye estado todo apagado
    fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);
    for (int i = 0; i < pos && i < seqLen; ++i) {
        uint8_t idx = seq[i];
        if (idx < colorHandler.numLeds) colorHandler.leds[idx] = col;
    }
}

/* ========== StrobeGlobalEffect ========== */
StrobeGlobalEffect::StrobeGlobalEffect(COLORHANDLER_& h, CRGB color, unsigned int updateMs, uint8_t flashes, unsigned int gapMs)
: DynamicEffect(h, -1, updateMs), col(color), burst(flashes), count(0), gap(gapMs), onPhase(true) {}

void StrobeGlobalEffect::update() {
    unsigned long now = millis();
    if (onPhase) {
        if (now - lastUpdate < interval) return;
        lastUpdate = now;
        fill_solid(colorHandler.leds, colorHandler.numLeds, col);
        onPhase = false;
        count++;
    } else {
        if (now - lastUpdate < interval) return;
        lastUpdate = now;
        fill_solid(colorHandler.leds, colorHandler.numLeds, CRGB::Black);
        onPhase = true;
        if (count >= burst) {
            // pausa entre ráfagas
            lastUpdate = now + (gap - interval); // crude delay comp
            count = 0;
        }
    }
}

/* ========== SparkleGlobalEffect ========== */
SparkleGlobalEffect::SparkleGlobalEffect(COLORHANDLER_& h, CRGB base, unsigned int tickMs, uint8_t density, uint8_t fade)
: DynamicEffect(h, -1, tickMs), baseCol(base), dens(density), fadeAmt(fade) {}

void SparkleGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds) return;

    // Atenúa todo un poco
    nscale8_video(colorHandler.leds, colorHandler.numLeds, 255 - fadeAmt);

    // Añade destellos aleatorios
    uint8_t tries = dens / 8 + 1;
    while (tries--) {
        if (random8() < dens) {
            int idx = random8() % colorHandler.numLeds;
            colorHandler.leds[idx] = CRGB::White;
            colorHandler.leds[idx] += baseCol; // tinte
        }
    }
}

/* ========== FireGlobalEffect ========== */
FireGlobalEffect::FireGlobalEffect(COLORHANDLER_& h, unsigned int tickMs)
: DynamicEffect(h, -1, tickMs), t(0) {}

void FireGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    for (int i = 0; i < colorHandler.numLeds; ++i) {
        uint8_t heat = qadd8(random8(160, 255), (i % 2) ? 20 : 0); // base cálida con vibración
        // Mapeo simple a color fuego (rojo-ámbar)
        CRGB c = CHSV( map(heat,160,255, 0,25), 255, heat );
        colorHandler.leds[i] = c;
    }
}

/* ========== AuroraGlobalEffect ========== */
AuroraGlobalEffect::AuroraGlobalEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs)
: DynamicEffect(h, -1, tickMs), hue(96), tintCol(tint) {}

void AuroraGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    hue += 1;
    for (int i = 0; i < colorHandler.numLeds; ++i) {
        uint8_t h = hue + i * 7;
        CRGB c = CHSV(h, 180, 200);
        // mezclar con tinte
        c.r = qadd8(c.r, (tintCol.r >> 1));
        c.g = qadd8(c.g, (tintCol.g >> 1));
        c.b = qadd8(c.b, (tintCol.b >> 1));
        colorHandler.leds[i] = c;
    }
}

/* ========== PlasmaNoiseEffect ========== */
PlasmaNoiseEffect::PlasmaNoiseEffect(COLORHANDLER_& h, CRGB tint, unsigned int tickMs)
: DynamicEffect(h, -1, tickMs), z(0), tintCol(tint) {}

void PlasmaNoiseEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    z += 8; // “tiempo” del ruido
    for (int i = 0; i < colorHandler.numLeds; ++i) {
        uint8_t n = inoise8(i * 32, z);      // 0..255
        CRGB c = CHSV( n, 200, 220 );
        // ligero tinte
        c.r = qadd8(c.r, tintCol.r >> 2);
        c.g = qadd8(c.g, tintCol.g >> 2);
        c.b = qadd8(c.b, tintCol.b >> 2);
        colorHandler.leds[i] = c;
    }
}

void CandleFlickerEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    phase += random8(18, 36);                 // jitter de fase
    uint8_t jitter = qadd8(sin8(phase), random8(0, 80));
    uint8_t level  = ease8InOutCubic(jitter); // 0..255 suavizado

    CRGB out = base;
    out.nscale8(140 + (level >> 1));          // 140..255 brillo
    if (ledIndex >= 0 && colorHandler.leds && ledIndex < colorHandler.numLeds) {
        colorHandler.leds[ledIndex] = out;
    }
}

void SparkleLocalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (ledIndex < 0 || !colorHandler.leds || ledIndex >= colorHandler.numLeds) return;

    CRGB out = base;
    if (hold) {
        // mantener destello
        out = CRGB::White; 
        hold--;
    } else if (random8() < 18) { // probabilidad de chispa
        out = CRGB::White;
        hold = 2;                // 2 ticks de brillo
    }
    colorHandler.leds[ledIndex] = out;
}

void CometLocalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (ledIndex < 0 || !colorHandler.leds || ledIndex >= colorHandler.numLeds) return;

    t += 10; // avanza fase
    // Pulso asimétrico (cabeza pasa y decae)
    uint8_t head = triwave8(t);              // 0..255
    uint8_t tail = scale8(head, 255 - decayPct);
    CRGB out = headColor;
    out.nscale8(qadd8(32, head));            // 32..255

    // “cola” simulada bajando a negro
    out.fadeToBlackBy(255 - tail);
    colorHandler.leds[ledIndex] = out;
}

void PlasmaLocalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (ledIndex < 0 || !colorHandler.leds || ledIndex >= colorHandler.numLeds) return;

    t += 7;
    uint8_t n = inoise8(t*3, ledIndex*37);
    // Modula saturación/valor sobre el tinte base
    CHSV hsv = rgb2hsv_approximate(base);
    hsv.s = qadd8(hsv.s, n>>2);
    hsv.v = qadd8(hsv.v, n>>1);
    colorHandler.leds[ledIndex] = hsv;
}

void BubblesGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds <= 0) return;

    // Decae toda la tira hacia el color base
    for (int i = 0; i < colorHandler.numLeds; ++i) {
        // mezcla suave hacia base y un pequeño fade
        colorHandler.leds[i].nscale8(255 - fadeAmt);
        CRGB scaled = base;
        scaled.nscale8(32);  // atenúa el color base
        colorHandler.leds[i] += scaled;
        // “ambiente” del tinte
    }

    // Genera burbujas (píxeles que suben rápido y se disipan)
    for (uint8_t k = 0; k < dens; ++k) {
        int p = random16(colorHandler.numLeds);
        colorHandler.leds[p] += CRGB(40, 40, 40); // destello
        // leve “pop” adicional
        if (random8() < 32) {
            colorHandler.leds[p] += CRGB(30, 30, 30);
        }
    }

    FastLED.show();
}

void LoadingGlobalEffect::update() {
    unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds <= 0) return;

    const int N = colorHandler.numLeds;

    // 1) Fondo ambiente tenue (mezcla al “base” muy atenuado)
    for (int i = 0; i < N; ++i) {
        // Baja brillo actual ligeramente para evitar “fantasmas”
        colorHandler.leds[i].nscale8(240);  // fade suave del frame anterior

        // añade un “ambiente” basado en base
        if (ambientDim > 0) {
            CRGB amb = base;
            amb.nscale8(ambientDim);        // ambiente bajo (p.e. 16/255)
            colorHandler.leds[i] += amb;
        }
    }

    // 2) Cabeza del “loading”
    int head = pos % N;
    colorHandler.leds[head] = base;  // cabeza al color pleno

    // 3) Cola decreciente (pos-1, pos-2, ...), wrap en los extremos
    uint8_t level = 180; // brillo inicial de la primera cola
    for (uint8_t t = 1; t <= trailLen; ++t) {
        int p = head + t;
        if (p >= N) p -= N;
        CRGB c = base;
        c.nscale8(level);              // atenuación por segmento de cola
        colorHandler.leds[p] = c;
        // cae exponencialmente la cola
        level = scale8(level, 150);    // ~60% del anterior
    }

    // 4) Avanza la posición (izq -> dcha, cíclico)
    pos--;
    if (pos < 0 ) pos = N - 1;
}

uint8_t LoadingEffect::kPath[16] = {0};
uint8_t LoadingEffect::kPathLen  = 0;

void LoadingEffect::configurePath(const uint8_t* path, uint8_t len) {
    if (!path || len == 0) {
        kPathLen = 0;
        return;
    }
    if (len > sizeof(kPath)) len = sizeof(kPath);
    for (uint8_t i = 0; i < len; ++i) kPath[i] = path[i];
    kPathLen = len;
}

void LoadingEffect::update() {
    const unsigned long now = millis();
    if (now - lastUpdate < interval) return;
    lastUpdate = now;

    if (!colorHandler.leds || colorHandler.numLeds <= 0 || kPathLen == 0) return;

    const int N = colorHandler.numLeds;

    // 1) Apaga solo el LED anterior si era válido
    if (prevLogical != 255 && prevLogical < N) {
        colorHandler.leds[prevLogical] = CRGB::Black;
    }

    // 2) Enciende el LED actual según el path
    const uint8_t logical = kPath[step % kPathLen];
    if (logical < N) {
        colorHandler.leds[logical] = base;
        prevLogical = logical;
    } else {
        prevLogical = 255; // índice fuera de rango
    }

    // 3) Avanza paso
    step = (step + 1) % kPathLen;
}