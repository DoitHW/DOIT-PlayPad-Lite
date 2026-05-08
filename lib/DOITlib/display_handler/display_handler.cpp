#include <display_handler/display_handler.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <encoder_handler/encoder_handler.h>
#include <Colors_DMS/Color_DMS.h>
#include <DynamicLEDManager_DMS/DynamicLEDManager_DMS.h>
#include "rom/rtc.h"    // Opcional, si quieres info de reset
#include <icons_64x64_DMS/icons_64x64_DMS.h>
#include <Translations_handler/translations.h>
#include <botonera_DMS/botonera_DMS.h>
#include <string.h>
// Definir dimensiones
#define CARD_WIDTH 110
#define CARD_HEIGHT 20
#define CARD_MARGIN 5
#define SCROLL_BAR_WIDTH 5

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite uiSprite = TFT_eSprite(&tft);

bool brightnessMenuActive = false;
uint8_t currentBrightness = 100;       // Valor actual en porcentaje
uint8_t tempBrightness = currentBrightness;  // Valor temporal mientras se ajusta

// Caché global para el menú de modos (evita leer SPIFFS en el loop de render)
std::vector<ModeCacheEntry> cachedModes;

/**
 * @brief  Muestra una lista de opciones de brillo (normal/atenuado) con cursor de selección,
 * título centrado con icono de sol y recorte de texto según el ancho disponible.
 */
void drawBrightnessMenu()
{
    // ────────────────────── Constantes de layout ────────────────────────
    constexpr int kScreenW        = 128;  // Ancho de pantalla/sprite en px
    constexpr int kScrollBarW     = 6;    // Ancho de la barra de scroll (px)
    constexpr int kVisibleOptions = 2;    // Nº de opciones visibles
    constexpr int kTotalOptions   = 2;    // Nº total de opciones del menú

    // Título e icono
    constexpr int kIconOuterR     = 12;   // Radio exterior del icono de sol (rayos)
    constexpr int kIconW          = kIconOuterR * 2;
    constexpr int kTitleGap       = 6;    // Espacio icono-texto
    constexpr int kTitleY         = 5;    // Y del título
    constexpr int kIconYOffset    = 10;   // Alineación Y del icono respecto al texto

    // Lista
    constexpr int kListX          = 9;    // Margen izquierdo de la lista
    constexpr int kScrollBarXPad  = 3;    // Separación derecha antes de la barra de scroll
    constexpr int kTextPad        = 2;    // Relleno lateral del área de texto
    constexpr int kCardRadius     = 3;    // Radio de esquinas en la tarjeta resaltada
    constexpr int kCardOutlinePad = 3;    // Expansión horizontal del resaltado
    constexpr int kCardYPadding   = 1;    // Expansión vertical del resaltado
    constexpr int kListYOffset    = 30;   // Desplazamiento vertical inicial de la lista

    // ────────────────────── Traducciones (opciones) ─────────────────────
    const char* options[kTotalOptions] = {
        getTranslation("BRILLO_NORMAL"),
        getTranslation("BRILLO_ATENUADO")
    };

    // Fondo
    uiSprite.fillSprite(BACKGROUND_COLOR);

    // ────────────────────── Título con icono de sol ─────────────────────
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextSize(1);

    const char* titleTxt = getTranslation("BRILLO");
    // Ancho del texto con la fuente actual
    const int titleW = uiSprite.textWidth(titleTxt);

    // Calcular centrado horizontal del bloque [icono + gap + texto]
    const int totalTitleW = kIconW + kTitleGap + titleW;
    const int titleLeft   = (kScreenW - totalTitleW) / 2;

    // Dibujo icono y texto (TL_DATUM)
    drawSunIcon(titleLeft + kIconOuterR, kTitleY + kIconYOffset, TFT_YELLOW);
    uiSprite.setTextDatum(TL_DATUM);
    uiSprite.drawString(String(titleTxt), titleLeft + kIconW + kTitleGap, kTitleY);

    // ────────────────────── Lista de opciones ───────────────────────────
    const int scrollBarX = kScreenW - kScrollBarW - kScrollBarXPad;
    const int textAreaW  = scrollBarX - kListX - kTextPad;

    // Índices de scroll y cursor
    static int startIndex = 0;

    // Clamp defensivo de brightnessMenuIndex a rango válido [0, kTotalOptions-1]
    int cursor = brightnessMenuIndex;
    if (cursor < 0) cursor = 0;
    if (cursor >= kTotalOptions) cursor = kTotalOptions - 1;

    // Ajuste de ventana de scroll para mantener el cursor visible
    const int maxStart = (kTotalOptions > kVisibleOptions) ? (kTotalOptions - kVisibleOptions) : 0;
    if ((cursor - startIndex) > (kVisibleOptions - 2) && startIndex < maxStart) {
        ++startIndex;
    } else if ((cursor - startIndex) < 1 && startIndex > 0) {
        --startIndex;
    }
    // Clamp por robustez (especialmente cuando total < visible)
    if (startIndex < 0) startIndex = 0;
    if (startIndex > maxStart) startIndex = maxStart;

    // Re-dibujo de la lista
    for (int i = 0; i < kVisibleOptions && (startIndex + i) < kTotalOptions; ++i) {
        const int idx = startIndex + i;
        const int y   = kListYOffset + i * (CARD_HEIGHT + CARD_MARGIN);
        const bool isCursor = (idx == cursor);

        // Resaltado de la fila seleccionada
        if (isCursor) {
            uiSprite.fillRoundRect(
                kListX - kCardOutlinePad,
                y - kCardYPadding,
                textAreaW + (kCardOutlinePad * 2),
                CARD_HEIGHT + (kCardYPadding * 2),
                kCardRadius,
                CARD_COLOR
            );
        }

        // Fuente y color por consistencia con otros menús
        uiSprite.setFreeFont(&FreeSans9pt7b);
        uiSprite.setTextSize(1);
        uiSprite.setTextDatum(TL_DATUM);
        uiSprite.setTextColor(TEXT_COLOR);

        // Texto a mostrar con recorte por ancho disponible
        const String fullText = options[idx];
        String displayText    = fullText;

        // Recorte progresivo carácter a carácter si excede el ancho del área de texto
        if (uiSprite.textWidth(fullText) > textAreaW) {
            displayText = "";
            for (int c = 0; c < fullText.length(); ++c) {
                const String tmp = displayText + fullText[c];
                if (uiSprite.textWidth(tmp) > textAreaW) break;
                displayText = tmp;
            }
        }

        uiSprite.drawString(displayText, kListX, y);
    }

    // Envío a pantalla
    uiSprite.pushSprite(0, 0);

}

/**
 * @brief Dibuja un icono de sol en la posición indicada.
 * @param x      Coordenada X del centro del sol.
 * @param y      Coordenada Y del centro del sol.
 * @param color  Color del sol (tanto círculo como rayos), en formato de 16 bits RGB565.
 */
void drawSunIcon(int16_t x, int16_t y, uint16_t color)
{
    // ---- Círculo central ----
    uiSprite.fillCircle(x, y, 6, color);

    // ---- Rayos del sol ----
    for (int i = 0; i < 8; i++) {
        // Ángulo en radianes (0, 45°, 90°, ...)
        float angle = i * PI / 4.0f;

        // Coordenadas de inicio del rayo (8 px desde el centro)
        int x1 = x + static_cast<int>(8 * cos(angle));
        int y1 = y + static_cast<int>(8 * sin(angle));

        // Coordenadas de fin del rayo (12 px desde el centro)
        int x2 = x + static_cast<int>(12 * cos(angle));
        int y2 = y + static_cast<int>(12 * sin(angle));

        uiSprite.drawLine(x1, y1, x2, y2, color);
    }
}

/**
 * @brief Inicializa el TFT y crea el sprite de trabajo a pantalla completa.
 * @pre `tft` (TFT_eSPI) y `uiSprite` (TFT_eSprite) deben estar instanciados..
 * @return void
 * @note La profundidad de color del sprite se fija a 16 bpp (RGB565).
 * @warning Si no hay RAM suficiente para el sprite, se mantiene el TFT
 *          inicializado y limpio, pero la función retorna sin sprite.
 */
void display_init(void) noexcept
{
    // Preferencia: constantes como constexpr (evita enums anónimos)
    constexpr uint8_t kInitialRotation = 0;   // 0: orientación por defecto
    constexpr uint8_t kColorDepth      = 16;  // 16 bpp = RGB565, óptimo con TFT_eSPI

    static_assert(kColorDepth == 8 || kColorDepth == 16,
                  "Profundidad de color soportada: 8 o 16 bpp");

    // 1) TFT: init básico y estado conocido
    tft.init();
    tft.setRotation(kInitialRotation);
    tft.setSwapBytes(true);                    // RGB565 con orden correcto
    tft.fillScreen(BACKGROUND_COLOR);          // Fondo consistente

    // 2) Medidas *reales* del panel tras rotación
    const int16_t screenWidth  = tft.width();
    const int16_t screenHeight = tft.height();

#ifdef DEBUG
    // Validación defensiva en debug: dimensiones válidas (>0)
    if (screenWidth <= 0 || screenHeight <= 0) {
    #if defined(SERIAL_DEBUG)
        Serial.println(F("[display_init] ERROR: dimensiones de pantalla no válidas."));
    #endif
        return;
    }
#endif

    // 3) Sprite: evita fugas si ya existía
    if (uiSprite.created()) {
        uiSprite.deleteSprite();
    }

    // Configuración del sprite antes de reservar su framebuffer
    uiSprite.setColorDepth(kColorDepth);
    uiSprite.setSwapBytes(true);

    // 4) Reserva del framebuffer del sprite (doble buffer en RAM)
    // Nota: TFT_eSprite::createSprite devuelve puntero base o nullptr si falla.
    uint8_t* fb = static_cast<uint8_t*>(uiSprite.createSprite(screenWidth, screenHeight));
    if (fb == nullptr) {
        // Sin RAM suficiente: dejamos el TFT limpio y abortamos silenciosamente.
        // (Opcional) Traza de diagnóstico si hay serie.
        #if defined(SERIAL_DEBUG)
        Serial.println(F("[display_init] ERROR: sin RAM para crear el sprite."));
        #endif
        return;
    }

    // 5) Estado inicial del sprite y primer volcado
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.pushSprite(0, 0);                 // Deja pantalla y buffer sincronizados
}

/**
 * @brief Muestra una animación de bienvenida con logo centrado y partículas orbitando.
 * @warning Coste en CPU por trigonometría por frame; aceptable para 12 partículas.
 */
void showWelcomeAnimation()
{
    // -------------------- Constantes de animación --------------------
    constexpr int   kLogoSizePx          = 64;
    constexpr int   kFramesIntro         = 30;
    constexpr int   kParticles           = 12;
    constexpr float kDegToRad            = 3.14159265f / 180.0f;
    constexpr float kFrameAngleStepDeg   = 0.2f;     // avance angular por frame (deg)
    constexpr float kParticleSpacingDeg  = 360.0f / kParticles;
    constexpr float kBaseRadiusPx        = 35.0f;    // radio base de la órbita
    constexpr float kRadiusAmplitudePx   = 10.0f;    // oscilación radial
    constexpr float kRadiusFreq          = 0.1f;     // frecuencia sinusoidal del radio
    constexpr float kBrightFreq          = 0.3f;     // frecuencia sinusoidal de brillo
    constexpr int   kParticleRadiusPx    = 2;
    constexpr int   kDelayIntroMs        = 80;       // retardo por frame (intro)
    constexpr int   kFadeSteps           = 10;       // pasos de fundido
    constexpr int   kDelayFadeMs         = 100;      // retardo por paso (fade)

    // -------------------- Helper local: clamp sin <algorithm> --------------------
    auto clampInt = [](int v, int lo, int hi) -> int {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    };

    // -------------------- Validación defensiva --------------------
    // Evita dibujar si el sprite aún no está creado (p. ej. fallo de RAM).
    if (!uiSprite.created()) {
    #if defined(SERIAL_DEBUG)
        Serial.println(F("[showWelcomeAnimation] Sprite no creado; animación cancelada."));
    #endif
        return;
    }

    // -------------------- Geometría de pantalla y logo --------------------
    const int16_t screenW = tft.width();
    const int16_t screenH = tft.height();
#ifdef DEBUG
    if (screenW <= 0 || screenH <= 0) {
    #if defined(SERIAL_DEBUG)
        Serial.println(F("[showWelcomeAnimation] Dimensiones de pantalla no válidas."));
    #endif
        return;
    }
#endif
    const int centerX       = (screenW - kLogoSizePx) / 2;  // esquina sup-izq del logo
    const int centerY       = (screenH - kLogoSizePx) / 2;
    const int screenCenterX = screenW / 2;                  // centro geométrico
    const int screenCenterY = screenH / 2;

    // -------------------- Animación principal (logo + partículas) --------------------
    for (int frame = 0; frame < kFramesIntro; ++frame) {
        uiSprite.fillSprite(BACKGROUND_COLOR);

        // Dibuja el logo centrado
        uiSprite.pushImage(
            centerX, centerY,
            kLogoSizePx, kLogoSizePx,
            reinterpret_cast<const uint16_t*>(doitLogo_64x64)
        );

        // Partículas orbitando
        for (int p = 0; p < kParticles; ++p) {
            const float angleDeg = (frame * kFrameAngleStepDeg) + (p * kParticleSpacingDeg);
            const float angleRad = angleDeg * kDegToRad;

            const float radius   = kBaseRadiusPx + sinf(frame * kRadiusFreq) * kRadiusAmplitudePx;
            const int   px       = screenCenterX + static_cast<int>(cosf(angleRad) * radius);
            const int   py       = screenCenterY + static_cast<int>(sinf(angleRad) * radius);

            // Brillo base con oscilación; se mantiene el rango 0..255
            int brightness = static_cast<int>(150.0f + sinf(frame * kBrightFreq + p) * 100.0f);
            brightness = clampInt(brightness, 0, 255);

            // Color animado tipo “rueda” (API externa existente)
            const uint16_t particleColor = colorWheel((frame * 8 + p * 32) & 0xFF);

            uiSprite.fillCircle(px, py, kParticleRadiusPx, particleColor);
        }

        uiSprite.pushSprite(0, 0);
        delay(kDelayIntroMs); // Retardo bloqueante deseado en la animación
    }

    // -------------------- Fundido de salida del logo --------------------
    for (int i = kFadeSteps; i >= 0; --i) {
        uiSprite.fillSprite(BACKGROUND_COLOR);

        // Mantiene el logo visible en los primeros pasos para un fade “elegante”
        if (i > 3) {
            uiSprite.pushImage(
                centerX, centerY,
                kLogoSizePx, kLogoSizePx,
                reinterpret_cast<const uint16_t*>(doitLogo_64x64)
            );
        }

        uiSprite.pushSprite(0, 0);
        delay(kDelayFadeMs);
    }
}

/**
 * @brief Genera un color RGB565 a partir de una posición en la rueda de color.
 *
 * Recorre gradualmente la secuencia RGB en tres tramos de 85 pasos cada uno:
 * - 0–84:   Rojo ascendente, verde descendente, azul constante en 0.
 * - 85–169: Verde constante en 0, rojo descendente, azul ascendente.
 * - 170–255: Azul descendente, verde ascendente, rojo constante en 0.
 *
 * @param pos Posición de 0..255 en la rueda de color.
 * @return Color en formato RGB565 compatible con `tft.color565()`.
 * @note El cálculo asume que `tft.color565(r,g,b)` recibe valores de 0..255.
 */
uint16_t colorWheel(uint8_t pos) {
  if (pos < 85)   return tft.color565(pos * 3, 255 - pos * 3, 0);
  if (pos < 170) { pos -= 85;  return tft.color565(255 - pos * 3, 0, pos * 3); }
  pos -= 170;     return tft.color565(0, pos * 3, 255 - pos * 3);
}

/**
 * @brief Muestra un mensaje con animación de “cargando” durante un tiempo dado.
 * @param message Mensaje a mostrar (UTF-8). Si es `nullptr`, se muestra una línea vacía.
 * @param delayTime Duración total en milisegundos (bloqueante).
 * @note  Usa `delay()` (≈30 FPS). Si se requiere no bloquear, migrar a bucle con `millis()`.
 * @warning El centrado vertical no se calcula; el texto comienza en Y=10 px.
 */
void showMessageWithLoading(const char* message, unsigned long delayTime) {
    ignoreInputs = true;
    unsigned long startTime = millis();
    int frameCount = 0;
    const int maxWidth = tft.width() - 20; // Margen de 10 píxeles a cada lado
    const int lineHeight = 20; // Altura aproximada de una línea de texto
  
    while (millis() - startTime < delayTime) {
      uiSprite.fillSprite(BACKGROUND_COLOR);
      
      // Dibuja el mensaje con wrap
      uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
      uiSprite.setTextDatum(TL_DATUM); // Alineación arriba-izquierda
      uiSprite.setFreeFont(&FreeSansBold9pt7b);
      uiSprite.setTextSize(1);
  
      int cursorY = 10; // Empezamos 10 píxeles desde arriba
      String currentLine = "";
      String remainingText = String(message);
  
      while (remainingText.length() > 0) {
        int spaceIndex = remainingText.indexOf(' ');
        String word = (spaceIndex == -1) ? remainingText : remainingText.substring(0, spaceIndex);
        
        if (uiSprite.textWidth(currentLine + word) <= maxWidth) {
          currentLine += word + " ";
          remainingText = (spaceIndex == -1) ? "" : remainingText.substring(spaceIndex + 1);
        } else {
          uiSprite.drawString(currentLine, 10, cursorY);
          cursorY += lineHeight;
          currentLine = word + " ";
          remainingText = (spaceIndex == -1) ? "" : remainingText.substring(spaceIndex + 1);
        }
  
        if (spaceIndex == -1) {
          uiSprite.drawString(currentLine, 10, cursorY);
          break;
        }
      }
  
      // Animación de puntos girando
      int centerX = tft.width() / 2;
      int centerY = tft.height() - 30;
      int radius = 10;
      for (int i = 0; i < 8; i++) {
        float angle = frameCount * 0.2 + i * PI / 4;
        int x = centerX + radius * cos(angle);
        int y = centerY + radius * sin(angle);
        uiSprite.fillCircle(x, y, 2, TEXT_COLOR); // Puntos más grandes
      }
  
      uiSprite.pushSprite(0, 0);
  
      frameCount++;
      delay(33); // Aproximadamente 30 FPS
    }
    ignoreInputs = false;
}

/**
 * @brief Dibuja un mensaje de error centrado en pantalla.
 * @param message Cadena C con el mensaje a mostrar (UTF-8). Puede ser `nullptr`.
 * @note No se realiza *wrap*; si el texto excede el ancho, se recortará visualmente.
 * @warning La función es puramente de render; no gestiona colas de errores ni traducciones.
 */
void drawErrorMessage(const char* message)
{
    // ─────────── Constantes de estilo ───────────
    constexpr int kTextSize = 2;

    // ─────────── Render centrado ───────────
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.setTextDatum(MC_DATUM);
    uiSprite.setTextSize(kTextSize);

    // Coordenadas del centro de la pantalla física
    const int cx = tft.width()  / 2;
    const int cy = tft.height() / 2;

    // Si message es nulo, mostramos una marca neutra para evitar acceso indebido
    uiSprite.drawString((message != nullptr) ? message : "(null)", cx, cy);

    // Volcado a pantalla
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Dibuja un icono de 64×64 px desde un fichero binario guardado en SPIFFS.
 *
 * Lee el bitmap línea a línea (formato RGB565, 2 bytes/píxel) a partir de
 * `OFFSET_ICONO` y lo dibuja en `uiSprite` usando `pushImage` con tiras de 1 px de alto.
 *
 * @param f      Referencia a `fs::File` ya abierto en modo lectura.
 * @param startX Coordenada X destino en el sprite (px).
 * @param startY Coordenada Y destino en el sprite (px).
 * @pre `f` debe estar abierto y posicionado correctamente; `OFFSET_ICONO` válido.
 * @pre `lineBuffer` debe apuntar a un buffer con capacidad ≥ 64*2 bytes alineado a 16 bits.
 * @pre `uiSprite` creado y con dimensiones suficientes para pintar desde `(startX,startY)`.
 * @note Formato esperado: 64 líneas consecutivas de 64 píxeles RGB565 (little-endian).
 * @warning Si la lectura devuelve menos bytes de los esperados se aborta el dibujado (salida anticipada).
 */
void drawElementIcon(fs::File& f, int startX, int startY)
{
    // ───────── Constantes del formato ─────────
    constexpr int kIconW           = 64;
    constexpr int kIconH           = 64;
    constexpr int kBytesPerPixel   = 2;
    constexpr int kLineBytes       = kIconW * kBytesPerPixel;

    // ───────── Validaciones ligeras ─────────
    // Archivo abierto y seek correcto al inicio del icono
    if (!f) {
 #ifdef DEBUG
        DEBUG__________ln("drawElementIcon: fichero no válido.");
 #endif
        return;
    }
    if (!f.seek(OFFSET_ICONO, SeekSet)) {
 #ifdef DEBUG
        DEBUG__________ln("drawElementIcon: fallo al hacer seek a OFFSET_ICONO.");
 #endif
        return;
    }

    // ───────── Lectura línea a línea y volcado ─────────
    for (int y = 0; y < kIconH; ++y) {
        // Lee una línea completa (64 píxeles RGB565)
        const int br = f.read(reinterpret_cast<uint8_t*>(lineBuffer), kLineBytes);
        if (br != kLineBytes) {
#ifdef DEBUG
            DEBUG__________ln("drawElementIcon: lectura incompleta; abortando.");
#endif
            break; // Abortamos de forma segura (no dibujamos líneas corruptas)
        }

        // Dibuja la línea en el sprite (alto 1, ancho 64)
        uiSprite.pushImage(
            startX,
            startY + y,
            kIconW,
            1,
            reinterpret_cast<const uint16_t*>(lineBuffer)
        );
    }
}

/**
 * @brief Dibuja el nombre del elemento centrado en la parte inferior del sprite.
 * @param elementName Cadena C con el nombre a mostrar. Si es `nullptr`, se dibuja vacío.
 * @param isSelected  Indica si debe resaltarse en verde (`TFT_GREEN`) o usar `TEXT_COLOR`.
 * @warning Si `uiSprite` y `tft` tienen tamaños distintos, el centrado puede desalinearse.
 */
void drawElementName(const char* elementName, bool isSelected)
{
    // ───────── Constantes de estilo/posicionamiento ─────────
    constexpr int kBottomOffsetPx = 40; // separación inferior fija (px)
    constexpr int kTextSize       = 1;  // tamaño de texto (coincide con el original)

    // Texto seguro: evita desreferenciar puntero nulo
    const char* text = (elementName != nullptr) ? elementName : "";

    // Configuración tipográfica y color
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextColor(isSelected ? TFT_GREEN : TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.setTextDatum(TC_DATUM); // arriba-centro
    uiSprite.setTextSize(kTextSize);

    // Coordenadas centradas respecto a la pantalla física
    const int centerX = tft.width()  / 2;
    const int nameY   = tft.height() - kBottomOffsetPx;

    // Dibujo (sin pushSprite; lo gestiona el flujo superior)
    uiSprite.drawString(text, centerX, nameY);
}

/**
 * @brief Dibuja el nombre del modo centrado en la parte inferior del sprite.
 * @param modeName Cadena C con el nombre del modo. Si es `nullptr`, se dibuja vacío.
 */
void drawModeName(const char* modeName){
    // ─── Constantes de estilo/posicionamiento ───
    constexpr int kBottomOffsetPx = 15; // distancia al borde inferior
    constexpr int kTextSize       = 1;  // tamaño de texto

    // Texto seguro para evitar desreferencias nulas
    const char* text = (modeName != nullptr) ? modeName : "";

    // Configuración tipográfica
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(kTextSize);

    // Coordenadas centradas respecto a la pantalla física
    const int centerX = tft.width()  / 2;
    const int posY    = tft.height() - kBottomOffsetPx;

    // Dibujo (no se hace pushSprite aquí)
    uiSprite.drawString(text, centerX, posY);
}

/**
 * @brief Dibuja flechas de navegación izquierda/derecha.
 */
void drawNavigationArrows()
{
    // ───── Constantes de diseño ─────
    constexpr int kArrowYOffset   = 20; // desplazamiento vertical desde el centro
    constexpr int kArrowSizePx    = 8;  // “altura” de cada flecha
    constexpr int kThicknessPass  = 3;  // nº de pasadas para simular grosor
    constexpr int kLeftOuterX     = 12; // X exterior de la flecha izquierda
    constexpr int kLeftInnerX     = 5;  // X interior de la flecha izquierda
    constexpr int kRightOuterOff  = 12; // offset desde el borde derecho (exterior)
    constexpr int kRightInnerOff  = 5;  // offset desde el borde derecho (interior)

    const int h = tft.height();
    const int w = tft.width();

    const int arrowY = (h / 2) - kArrowYOffset;

    // ───── Flecha izquierda (líneas paralelas para grosor) ─────
    for (int i = 0; i < kThicknessPass; ++i) {
        uiSprite.drawLine(kLeftOuterX + i,               arrowY - kArrowSizePx + i,
                          kLeftInnerX + i,               arrowY,                   TFT_WHITE);
        uiSprite.drawLine(kLeftInnerX + i,               arrowY,
                          kLeftOuterX + i,               arrowY + kArrowSizePx - i, TFT_WHITE);
    }

    // ───── Flecha derecha (simétrica respecto al borde derecho) ─────
    for (int i = 0; i < kThicknessPass; ++i) {
        uiSprite.drawLine(w - kRightOuterOff - i,        arrowY - kArrowSizePx + i,
                          w - kRightInnerOff - i,        arrowY,                   TFT_WHITE);
        uiSprite.drawLine(w - kRightInnerOff - i,        arrowY,
                          w - kRightOuterOff - i,        arrowY + kArrowSizePx - i, TFT_WHITE);
    }
}


// Parámetros para el área del nombre (ajusta según necesites)
const int DISPLAY_WIDTH = 128;            // Ancho total del display
const int NAME_Y = tft.height() - 40;       // Posición vertical del área del nombre
const int NAME_AREA_HEIGHT = 20;          // Altura del área donde se dibuja el nombre (más reducido)
const int NAME_SCROLL_INTERVAL = 30;      // Intervalo de actualización en ms

// Variables globales para el scroll del nombre
bool nameScrollActive = false;            // Se activa si el nombre es demasiado largo
String currentDisplayName = "";           // Nombre del elemento (ya procesado)
int nameScrollOffset = 0;                 // Desplazamiento actual (en píxeles)
int nameScrollDirection = 1;              // 1: incrementa; -1: decrementa
unsigned long lastNameScrollUpdate = 0;   // Última actualización (ms)

/**
 * @brief Redibuja la zona del nombre aplicando el desplazamiento horizontal actual.
 * @note No modifica `nameScrollOffset`; solo renderiza en base a su valor.
 */
void updateNameDisplay()
{
    // ---- Configuración tipográfica (solo una vez) ----
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextSize(1);
    uiSprite.setTextDatum(TL_DATUM); // dibujamos desde la esquina sup-izq

    // ---- Cálculos de layout ----
    const int drawY = NAME_Y;
    const int textWidth = uiSprite.textWidth(currentDisplayName);

    // Centrado horizontal menos offset (scroll horizontal “vaivén”)
    // NOTA: si se desea scroll desde el borde izquierdo, usar: int drawX = -nameScrollOffset;
    const int centered = (DISPLAY_WIDTH - textWidth) / 2;
    int drawX = centered - nameScrollOffset;

    // ---- Limpieza del área de nombre ----
    uiSprite.fillRect(0, drawY, DISPLAY_WIDTH, NAME_AREA_HEIGHT, BACKGROUND_COLOR);

    // ---- Color según selección lógica externa ----
    const bool isSelected = selectedStates[currentIndex];
    uiSprite.setTextColor(isSelected ? TFT_GREEN : TEXT_COLOR, BACKGROUND_COLOR);

    // ---- Render del texto ----
    // Si la cadena está vacía, solo limpiamos el área (comportamiento defensivo).
    if (!currentDisplayName.isEmpty()) {
        uiSprite.drawString(currentDisplayName, drawX, drawY);
    }

    // ---- Volcado parcial a TFT (región del nombre) ----
    uiSprite.pushSprite(0, drawY, 0, drawY, DISPLAY_WIDTH, NAME_AREA_HEIGHT);
}

/**
 * @brief Actualiza el desplazamiento horizontal (vaivén) del nombre y redibuja si toca.
 */
void updateNameScroll()
{
    // Usar la misma fuente/tamaño que el área de nombre
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextSize(1);

    const int textWidth = uiSprite.textWidth(currentDisplayName);

    // Si el texto cabe totalmente, no hay scroll.
    if (textWidth <= DISPLAY_WIDTH) {
        nameScrollActive  = false;
        nameScrollOffset  = 0;
        // No forzamos redibujado aquí para no cambiar el flujo original.
        return;
    }

    // Texto más ancho que la zona: activar scroll (vaivén)
    nameScrollActive = true;

    // --- Límites del vaivén respecto al centrado ---
    // En render: drawX = centered - nameScrollOffset;
    // left-edge (texto tocando borde izq):   drawX = 0           => offset = centered
    // right-edge (texto tocando borde dcho): drawX + W = DISP_W  => offset = centered + W - DISP_W
    const int centered = (DISPLAY_WIDTH - textWidth) / 2;               // < 0 cuando W > DISP_W
    const int minScroll = centered;                                     // límite negativo
    const int maxScroll = centered + textWidth - DISPLAY_WIDTH;         // límite positivo

    // Actualización temporal (no bloqueante)
    const unsigned long now = millis();
    if ((now - lastNameScrollUpdate) >= NAME_SCROLL_INTERVAL) {
        nameScrollOffset += nameScrollDirection;

        // Rebotar en los extremos y corregir sobrepaso
        if (nameScrollOffset <= minScroll) {
            nameScrollOffset   = minScroll;
            nameScrollDirection = 1;
        } else if (nameScrollOffset >= maxScroll) {
            nameScrollOffset   = maxScroll;
            nameScrollDirection = -1;
        }

        lastNameScrollUpdate = now;

        // Redibujar la región del nombre con el nuevo offset
        updateNameDisplay();
    }
}

// Parámetros para el área del nombre del modo
const int MODE_DISPLAY_WIDTH = 128;        // Ancho total del display (o zona asignada)
const int MODE_Y = tft.height() - 15;        // Posición vertical donde se dibuja el nombre del modo (igual que drawModeName)
const int MODE_AREA_HEIGHT = 20;             // Altura de la zona a actualizar (ajusta para que no se solape)
const int MODE_SCROLL_INTERVAL = 60;         // Intervalo de actualización (ms)

// Variables globales para el scroll del nombre del modo
bool modeScrollActive = false;             // Se activa si el texto del modo es demasiado largo
String currentModeDisplayName = "";        // Cadena que contiene el nombre del modo (ya procesado)
int modeScrollOffset = 0;                  // Desplazamiento actual en píxeles para el modo
int modeScrollDirection = 1;               // Dirección: 1 para incrementar, -1 para decrementar
unsigned long lastModeScrollUpdate = 0;    // Última actualización en ms



/**
 * @brief Redibuja el área del nombre del modo aplicando el desplazamiento horizontal.
 */
void updateModeDisplay(){
    // ---- Configuración tipográfica (antes de medir) ----
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextSize(1);

    // ---- Medición y cálculo de layout ----
    const int textWidth = uiSprite.textWidth(currentModeDisplayName);
    const int centerX   = (MODE_DISPLAY_WIDTH - textWidth) / 2;
    const int drawX     = centerX - modeScrollOffset; // centrado menos desplazamiento
    const int drawY     = MODE_Y;

    // ---- Limpiar el área destinada al nombre del modo ----
    uiSprite.fillRect(0, drawY, MODE_DISPLAY_WIDTH, MODE_AREA_HEIGHT, BACKGROUND_COLOR);

    // ---- Dibujo del texto (alineación arriba-izquierda) ----
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextSize(1);
    uiSprite.setTextDatum(TL_DATUM);
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    if (currentModeDisplayName && currentModeDisplayName[0] != '\0') { // defensivo, no cambia semántica
        uiSprite.drawString(currentModeDisplayName, drawX, drawY);
    }

    // ---- Volcado parcial solo de la franja afectada ----
    uiSprite.pushSprite(0, drawY, 0, drawY, MODE_DISPLAY_WIDTH, MODE_AREA_HEIGHT); // si tu macro es MODE_AREA_HEIGHT, usa esa
}

// Actualiza el desplazamiento del nombre del modo y llama a updateModeDisplay()
void updateModeScroll() {
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextSize(1);
    int textWidth = uiSprite.textWidth(currentModeDisplayName);

    // Si el texto cabe en el área, desactivar scroll
    if (textWidth <= MODE_DISPLAY_WIDTH) {
         modeScrollActive = false;
         modeScrollOffset = 0;
         return;
    }

    int extraMargin = 10;
    
    // Ajuste del desplazamiento máximo para centrar correctamente
    int maxScroll = (textWidth - MODE_DISPLAY_WIDTH) / 2 + extraMargin;
    int minScroll = -maxScroll; // Permitir que el texto se desplace en ambas direcciones

    if (millis() - lastModeScrollUpdate >= MODE_SCROLL_INTERVAL) {
         modeScrollOffset += modeScrollDirection;

         // Invertir dirección del scroll al alcanzar los límites
         if (modeScrollOffset <= minScroll) {
              modeScrollOffset = minScroll;
              modeScrollDirection = 1;
         } else if (modeScrollOffset >= maxScroll) {
              modeScrollOffset = maxScroll;
              modeScrollDirection = -1;
         }

         lastModeScrollUpdate = millis();
         updateModeDisplay();
    }
}

extern float batteryVisualPercentage;

/**
 * @brief Dibuja en pantalla el elemento actual (icono, nombre y modo), con scroll si es necesario.
 *
 * Carga/actualiza los estados alternativos del elemento, renderiza el icono (RAM o SPIFFS),
 * muestra el nombre (centrado con scroll vaivén cuando excede el ancho), y el nombre del modo
 * (también con scroll si procede). Dibuja además las flechas de navegación, el icono de batería
 * y, si aplica, el candado. Al final vuelca el sprite a la TFT.
 */
void drawCurrentElement()
{   
    if (!isInMainMenu()) return;
    
    // ───────── Constantes de layout (evitan “números mágicos”) ─────────
    constexpr int kIconSizePx        = 64;   // icono 64x64
    constexpr int kIconYOffsetPx     = 20;   // eleva el icono 20 px respecto al centro
    constexpr int kModeBottomOffset  = 15;   // offset Y para el nombre del modo (desde abajo)

    uiSprite.fillSprite(BACKGROUND_COLOR);

    // Clamp del índice actual al rango válido
    currentIndex = constrain(currentIndex, 0, static_cast<int>(elementFiles.size()) - 1);
    String currentFile = elementFiles[currentIndex];
    byte   currentMode = 0;

    // ───────── Cargar/recuperar estados alternativos del elemento ─────────
    if (elementAlternateStates.find(currentFile) != elementAlternateStates.end()) {
        currentAlternateStates = elementAlternateStates[currentFile];
    } else {
        currentAlternateStates = initializeAlternateStates(currentFile);
        elementAlternateStates[currentFile] = currentAlternateStates;
    }

    // Helper local: nombre “amigable” desde el nombre de archivo (sin prefijos/sufijos)
    auto getDisplayName = [](const String& fileName) -> String {
        String baseName = fileName.substring(fileName.lastIndexOf("/") + 1,
                                             fileName.lastIndexOf(".bin"));
        baseName.replace("element_", "");
        int underscoreIndex = baseName.lastIndexOf("_");
        String name;
        if (underscoreIndex != -1) {
            String possibleNumber = baseName.substring(underscoreIndex + 1);
            if (possibleNumber.toInt() > 0) {
                name = baseName.substring(0, underscoreIndex);
            } else {
                name = baseName;
            }
        } else {
            name = baseName;
        }
        int count = 0;
        for (const auto& file : elementFiles) {
            if (file.startsWith("/element_" + name)) {
                count++;
                if (file == fileName) break;
            }
        }
        return (count > 1) ? (name + "(" + String(count) + ")") : name;
    };

    // ─────────────────────────────── Caso 1: Fijos ───────────────────────────────
    if (currentFile == "Ambientes" || currentFile == "Fichas" || currentFile == "Comunicador" || currentFile == "Dado")
    {
        INFO_PACK_T* option = nullptr;
        if      (currentFile == "Ambientes")   option = &ambientesOption;
        else if (currentFile == "Fichas")      option = &fichasOption;
        else if (currentFile == "Comunicador") option = &comunicadorOption;
        else                                   option = &dadoOption;


        currentMode = option->currentMode;

        // Icono centrado horizontal y ligeramente elevado
        const int startX = (tft.width()  - kIconSizePx) / 2;
        const int startY = (tft.height() - kIconSizePx) / 2 - kIconYOffsetPx;
        for (int y = 0; y < kIconSizePx; ++y) {
            // Copia una línea (64 píxeles, RGB565 => 2 bytes/píxel)
            memcpy(lineBuffer, option->icono[y], kIconSizePx * 2);
            uiSprite.pushImage(startX, startY + y, kIconSizePx, 1, lineBuffer);
        }

        // Nombre del elemento (traducido)
        const String displayName =
        (currentFile == "Ambientes")   ? String(getTranslation("AMBIENTES"))   :
        (currentFile == "Fichas")      ? String(getTranslation("FICHAS"))      :
        (currentFile == "Comunicador") ? String(getTranslation("COMUNICADOR")) :
                                        String(getTranslation("DADO"));     


        // Medición con fuente grande
        uiSprite.setFreeFont(&FreeSansBold12pt7b);
        uiSprite.setTextSize(1);
        const int elementTextWidth = uiSprite.textWidth(displayName);

        // Scroll del nombre si excede el ancho disponible
        if (elementTextWidth > DISPLAY_WIDTH) {
            nameScrollActive       = true;
            currentDisplayName     = displayName;
            nameScrollOffset       = 0;
            nameScrollDirection    = 1;
            lastNameScrollUpdate   = millis();
            updateNameDisplay();
        } else {
            nameScrollActive = false;
            drawElementName(displayName.c_str(), selectedStates[currentIndex]);
        }

        // Construcción/actualización del vector de estados alternativos visibles
        int visibleModeIndex = -1;
        if (elementAlternateStates.find(currentFile) == elementAlternateStates.end()) {
            std::vector<bool> tempAlternate;
            int count = 0;
            for (int i = 0; i < 16; ++i) {
                if (strlen((char*)option->mode[i].name) > 0 &&
                    checkMostSignificantBit(option->mode[i].config))
                {
                    tempAlternate.push_back(false);
                    if (i == option->currentMode) visibleModeIndex = count;
                    ++count;
                }
            }
            currentAlternateStates = tempAlternate;
            elementAlternateStates[currentFile] = currentAlternateStates;
        } else {
            currentAlternateStates = elementAlternateStates[currentFile];
            int count = 0;
            for (int i = 0; i < 16; ++i) {
                if (strlen((char*)option->mode[i].name) > 0 &&
                    checkMostSignificantBit(option->mode[i].config))
                {
                    if (i == option->currentMode) { visibleModeIndex = count; break; }
                    ++count;
                }
            }
        }

        // Nombre del modo (clave → traducción), con posible sufijo según estado alternativo
        String modeNameStr = String((char*)option->mode[option->currentMode].name);
        modeNameStr = String(getTranslation(modeNameStr.c_str()));

        String modeDisplay;
        if (visibleModeIndex >= 0 && currentAlternateStates.size() > static_cast<size_t>(visibleModeIndex)) {
            modeDisplay = getModeDisplayName(modeNameStr, currentAlternateStates[visibleModeIndex]);
        } else {
            modeDisplay = modeNameStr;
        }

        const int modeTextWidth = uiSprite.textWidth(modeDisplay);
        if (modeTextWidth > MODE_DISPLAY_WIDTH) {
            modeScrollActive        = true;
            currentModeDisplayName  = modeDisplay;
            modeScrollOffset        = 0;
            modeScrollDirection     = 1;
            lastModeScrollUpdate    = millis();
            updateModeDisplay();
        } else {
            modeScrollActive = false;
            uiSprite.setFreeFont(&FreeSans9pt7b);
            uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
            uiSprite.setTextDatum(TC_DATUM);
            uiSprite.setTextSize(1);
            uiSprite.drawString(modeDisplay, tft.width() / 2, tft.height() - kModeBottomOffset);
        }
    }
    // ─────────────────────────────── Caso 2: Apagar ───────────────────────────────
    else if (currentFile == "Apagar")
    {
        INFO_PACK_T* option = &apagarSala;

        const int startX = (tft.width()  - kIconSizePx) / 2;
        const int startY = (tft.height() - kIconSizePx) / 2 - kIconYOffsetPx;
        for (int y = 0; y < kIconSizePx; ++y) {
            memcpy(lineBuffer, option->icono[y], kIconSizePx * 2);
            uiSprite.pushImage(startX, startY + y, kIconSizePx, 1, lineBuffer);
        }

        // Estados alternativos: uno falso por defecto
        currentAlternateStates.clear();
        currentAlternateStates.push_back(false);
        elementAlternateStates[currentFile] = currentAlternateStates;

        // Nombre del elemento (traducido) y del modo (tal cual en original)
        drawElementName(getTranslation("APAGAR"), false);
        drawModeName((char*)option->mode[currentMode].name);

        nameScrollActive = false;
        modeScrollActive = false;
    }
    // ─────────────────────── Caso 3: Elemento en SPIFFS ──────────────────────────
    else
    {
        fs::File f = SPIFFS.open(currentFile, "r");
        if (!f) {
            drawErrorMessage("Error leyendo elemento");
            return;
        }

        char elementName[25] = {0};
        char modeName[25]    = {0};
        int  startX = 0, startY = 0;

        if (!readElementData(f, elementName, modeName, startX, startY)) {
            drawErrorMessage("Datos incompletos");
            return;
        }

        // Icono desde fichero (usa `startX/startY` leídos)
        drawElementIcon(f, startX, startY);

        // Nombre del elemento (amigable) y scroll si excede
        uiSprite.setFreeFont(&FreeSansBold12pt7b);
        uiSprite.setTextSize(1);
        const String displayName = getDisplayName(currentFile);
        const int elementTextWidth = uiSprite.textWidth(displayName);

        if (elementTextWidth > DISPLAY_WIDTH) {
            nameScrollActive       = true;
            currentDisplayName     = displayName;
            nameScrollOffset       = 0;
            nameScrollDirection    = 1;
            lastNameScrollUpdate   = millis();
            updateNameDisplay();
        } else {
            nameScrollActive = false;
            drawElementName(displayName.c_str(), selectedStates[currentIndex]);
        }

        // Modo real almacenado
        int realModeIndex = 0;
        f.seek(OFFSET_CURRENTMODE, SeekSet);
        f.read((uint8_t*)&realModeIndex, 1);

        // Estados alternativos (hasta 16)
        const int OFFSET_ALTERNATE_STATES = OFFSET_CURRENTMODE + 1;
        byte storedStates[16] = {0};
        f.seek(OFFSET_ALTERNATE_STATES, SeekSet);
        const size_t bytesRead = f.read(storedStates, 16);

        if (elementAlternateStates.find(currentFile) == elementAlternateStates.end()) {
            std::vector<bool> tempAlternate;
            int visibleModeIndex = -1;
            int count = 0;

            for (int i = 0; i < 16; ++i) {
                char modeBuf[25]   = {0};
                byte modeConfig[2] = {0};
                f.seek(OFFSET_MODES + i * SIZE_MODE, SeekSet);
                f.read((uint8_t*)modeBuf, 24);
                f.seek(OFFSET_MODES + i * SIZE_MODE + 216, SeekSet);
                f.read(modeConfig, 2);

                if (strlen(modeBuf) > 0 && checkMostSignificantBit(modeConfig)) {
                    const bool hasAlt  = getModeFlag(modeConfig, HAS_ALTERNATIVE_MODE);
                    const bool altState = (bytesRead > 0 && count < 16) ? (storedStates[count] > 0) : false;

                    tempAlternate.push_back(hasAlt ? altState : false);
                    if (i == realModeIndex) visibleModeIndex = count;
                    ++count;
                }
            }

            currentAlternateStates = tempAlternate;
            elementAlternateStates[currentFile] = currentAlternateStates;
            DEBUG__________ln("DEBUG: [SPIFFS] currentAlternateStates.size() = " + String(currentAlternateStates.size()));
        } else {
            currentAlternateStates = elementAlternateStates[currentFile];
        }

        // Mapear índice de modo real a índice visible
        int visibleIndex = -1;
        {
            int count = 0;
            for (int i = 0; i < 16; ++i) {
                char modeBuf[25]  = {0};
                byte tempConfig[2] = {0};
                f.seek(OFFSET_MODES + i * SIZE_MODE, SeekSet);
                f.read((uint8_t*)modeBuf, 24);
                f.seek(OFFSET_MODES + i * SIZE_MODE + 216, SeekSet);
                f.read(tempConfig, 2);

                if (strlen(modeBuf) > 0 && checkMostSignificantBit(tempConfig)) {
                    if (i == realModeIndex) { visibleIndex = count; break; }
                    ++count;
                }
            }
        }

        // Nombre del modo desde fichero (sin traducción en el original)
        String modeDisplay = String(modeName);
        if (visibleIndex >= 0 && currentAlternateStates.size() > static_cast<size_t>(visibleIndex)) {
            modeDisplay = getModeDisplayName(modeDisplay, currentAlternateStates[visibleIndex]);
        }

        const int modeTextWidth = uiSprite.textWidth(modeDisplay);

        if (selectedStates[currentIndex]) {
            if (modeTextWidth > MODE_DISPLAY_WIDTH) {
                modeScrollActive        = true;
                currentModeDisplayName  = modeDisplay;
                modeScrollOffset        = 0;
                modeScrollDirection     = 1;
                lastModeScrollUpdate    = millis();
                updateModeDisplay();
            } else {
                modeScrollActive = false;
                drawModeName(modeDisplay.c_str());
            }
        } else {
            // Si está deseleccionado, limpiar la zona del modo
            uiSprite.fillRect(0,
                              tft.height() - MODE_AREA_HEIGHT,
                              MODE_DISPLAY_WIDTH,
                              MODE_AREA_HEIGHT,
                              BACKGROUND_COLOR);
            modeScrollActive = false;
        }

        f.close();
        currentMode = realModeIndex;
    }

    // ───────── Estado final/auxiliar ─────────
    currentModeIndex = currentMode;
    colorHandler.setCurrentFile(currentFile);
    //colorHandler.setPatternBotonera(currentModeIndex, ledManager);

    drawNavigationArrows();

    // Candado (solo en menú principal)
    if (isInMainMenu() && systemLocked) {
        uiSprite.pushImage(0, 0, 16, 16, device_lock);
    }

    drawBatteryIconMini(batteryPercentage);


    uiSprite.pushSprite(0, 0);

    lastDisplayInteraction = millis();
    displayOn = true;
}

/**
 * @brief Apaga la pantalla por inactividad.
 *
 * Rellena la pantalla con negro y marca `displayOn` como falso para indicar
 * que está apagada. Es llamada cuando expira el tiempo de inactividad definido
 * por la lógica de tu aplicación.
 */
void display_sleep() {
    if (!displayOn) return;
    uiSprite.fillSprite(TFT_BLACK);
    uiSprite.pushSprite(0, 0);
    displayOn = false;
    //DEBUG__________ln("Pantalla apagada por inactividad.");
}

/**
 * @brief Reactiva la pantalla y redibuja la interfaz principal.
 * Marca `displayOn` como activa, repinta el menú correspondiente (cognitivo o
 * elemento actual) y actualiza el instante de última interacción.
 */
void display_wakeup() {
    displayOn = true;
    // Repintar vista según el contexto actual
    if (inCognitiveMenu) drawCognitiveMenu();
    else drawCurrentElement();
    lastDisplayInteraction = millis();
    //DEBUG__________ln("Pantalla reactivada por interacción.");
  }

/**
 * @brief Carga la lista de modos en RAM para evitar leer SPIFFS durante el renderizado.
 * Se debe llamar UNA VEZ antes de empezar a dibujar el menú de modos.
 */
void loadModesCache(const String& currentFile) {
    cachedModes.clear();

    // 1) Acción de Encender/Apagar (Siempre primera)
    ModeCacheEntry actionToggle;
    actionToggle.originalIndex = -3;
    actionToggle.label = selectedStates[currentIndex] ? getTranslation("APAGAR") : getTranslation("ENCENDER");
    actionToggle.isAlternate = false;
    cachedModes.push_back(actionToggle);

    // 2) Cargar modos del elemento (RAM o SPIFFS)
    if (currentFile == "Ambientes" || currentFile == "Fichas" || currentFile == "Comunicador" || currentFile == "Dado") {
        // -- Elementos en RAM --
        INFO_PACK_T* option = nullptr;
        if (currentFile == "Ambientes")        option = &ambientesOption;
        else if (currentFile == "Fichas")      option = &fichasOption;
        else if (currentFile == "Comunicador") option = &comunicadorOption;
        else                                   option = &dadoOption;

        // Recalcular estados alternativos si no existen
        if (elementAlternateStates.find(currentFile) == elementAlternateStates.end()) {
            std::vector<bool> tempStates(16, false);
            elementAlternateStates[currentFile] = tempStates;
        }
        currentAlternateStates = elementAlternateStates[currentFile];

        // Iterar modos
        int visibleCount = 0;
        for (int i = 0; i < 16; ++i) {
            if (strlen((char*)option->mode[i].name) > 0 &&
                checkMostSignificantBit(option->mode[i].config)) {

                ModeCacheEntry entry;
                entry.originalIndex = i;
                
                String rawName        = String((char*)option->mode[i].name);
                String translatedName = getTranslation(rawName.c_str());

                bool altActive = (visibleCount < (int)currentAlternateStates.size())
                                ? currentAlternateStates[visibleCount]
                                : false;

                // 👇 Muy importante: la caché guarda SIEMPRE el nombre base
                entry.label       = translatedName; 
                entry.isAlternate = altActive;  // meta, por si quieres usarlo en otros sitios

                cachedModes.push_back(entry);
                ++visibleCount;
            }
        }

    } 
    else if (currentFile != "Apagar") { 
        // -- Elementos en SPIFFS --
        fs::File f = SPIFFS.open(currentFile, "r");
        if (f) {
            const int OFFSET_ALTERNATE = OFFSET_CURRENTMODE + 1;
            f.seek(OFFSET_ALTERNATE, SeekSet);
            byte storedStates[16] = {0};
            f.read(storedStates, 16);

            std::vector<bool> tempStates;   // estados alternativos por índice visible

            int visibleCount = 0;
            for (int i = 0; i < 16; ++i) {
                char modeName[25] = {0};
                byte modeCfg[2]   = {0};
                
                f.seek(OFFSET_MODES + i * SIZE_MODE, SeekSet);
                f.read((uint8_t*)modeName, 24);
                f.seek(OFFSET_MODES + i * SIZE_MODE + 216, SeekSet);
                f.read(modeCfg, 2);

                if (strlen(modeName) > 0 && checkMostSignificantBit(modeCfg)) {
                    ModeCacheEntry entry;
                    entry.originalIndex = i;
                    
                    String baseName = String(modeName);

                    bool hasAltFeature = getModeFlag(modeCfg, HAS_ALTERNATIVE_MODE);
                    bool altActive     = (hasAltFeature && visibleCount < 16)
                                        ? (storedStates[visibleCount] > 0)
                                        : false;

                    // 👇 de nuevo, solo nombre base en la caché
                    entry.label       = baseName;
                    entry.isAlternate = altActive;

                    cachedModes.push_back(entry);

                    // Mantener el vector de estados alternativos alineado por índice visible
                    tempStates.push_back(altActive);
                    ++visibleCount;
                }
            }

            // Ahora sí: sincronizas el estado global con lo leído de SPIFFS
            currentAlternateStates              = tempStates;
            elementAlternateStates[currentFile] = currentAlternateStates;

            f.close();
        }

    }

    // 3) Acción de Volver (Siempre última)
    ModeCacheEntry actionBack;
    actionBack.originalIndex = -2;
    actionBack.label = getTranslation("VOLVER");
    cachedModes.push_back(actionBack);
    
    // Copiar el mapa de índices globales para la navegación (necesario para handleEncoder)
    totalModes = cachedModes.size();
    for(int i=0; i<totalModes && i<18; i++) {
        globalVisibleModesMap[i] = cachedModes[i].originalIndex;
    }
}

std::map<String, std::vector<bool>> elementAlternateStates;
std::vector<bool> currentAlternateStates;

/**
 * @brief Dibuja la pantalla de selección de modos con lista desplazable y ticker horizontal.
 *
 * Construye el mapa de modos visibles para el elemento actual (modos válidos + acciones
 * especiales “Encender/Apagar” y “Volver”), centra el título, pinta las filas visibles,
 * resalta la seleccionada y aplica ticker horizontal si el texto no cabe. Finalmente
 * actualiza el desplazamiento vertical suave (no intrusivo).
 */
void drawModesScreen()
{
    /*────────────────── Seguridad: Carga diferida ──────────────────*/
    if (cachedModes.empty()) {
        loadModesCache(elementFiles[currentIndex]);
    }

    /*────────────────── Constantes de layout ──────────────────*/
    const int screenW           = tft.width();
    const int kTitleCenterX     = screenW / 2;
    constexpr int kTitleY       = 5;
    constexpr int kListStartY   = 30;
    constexpr int kCardRadius   = 3;
    constexpr int kHLPadX       = 3;
    constexpr int kHLPadY       = 1;
    constexpr int kVisibleRows  = 4;
    constexpr int kTickerFrameMs= 50;
    constexpr int kRowTopInset  = (kHLPadY + 1);

    constexpr int kListLeftX    = 10;
    constexpr int kRightMargin  = 10;
    const int textAreaW         = screenW - kListLeftX - kRightMargin;

    const int stepY             = CARD_HEIGHT + CARD_MARGIN;

    /*────────────────── Variables estáticas (Scroll/Ticker) ──────────────────*/
    static int  scrollOffset        = 0;
    static int  targetScrollOffset  = 0;
    static int  modeTickerOffset    = 0;
    static int  modeTickerDirection = 1;
    static unsigned long modeLastFrameTime = 0;
    static int  lastSelectedMode    = -1;
    static int  startIndex          = 0;

    static TFT_eSprite tickerSprite(&tft);
    static int tickerW = 0, tickerH = 0;

    static TFT_eSprite listSprite(&tft);
    static int listW = 0, listH = 0;

    const int totalEntries = (int)cachedModes.size();

    /*────────────────── Cabecera ──────────────────*/
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString(getTranslation("MODOS"), kTitleCenterX, kTitleY);

    // Si no hay nada que mostrar, no sigas.
    if (totalEntries <= 0) {
        uiSprite.setFreeFont(&FreeSans9pt7b);
        uiSprite.setTextDatum(TC_DATUM);
        uiSprite.drawString(getTranslation("SIN_DATOS"), kTitleCenterX, kListStartY + 25);
        uiSprite.pushSprite(0, 0);
        return;
    }

    // Normalizar selección
    if (currentModeIndex < 0) currentModeIndex = 0;
    if (currentModeIndex >= totalEntries) currentModeIndex = totalEntries - 1;

    /*────────────────── Gestión de ventana (Scroll) ──────────────────*/
    if ((currentModeIndex - startIndex) > (kVisibleRows - 2) && startIndex < (totalEntries - kVisibleRows)) {
        ++startIndex;
    } else if ((currentModeIndex - startIndex) < 1 && startIndex > 0) {
        --startIndex;
    }

    if (startIndex < 0) startIndex = 0;
    const int lastStart = (totalEntries > kVisibleRows) ? (totalEntries - kVisibleRows) : 0;
    if (startIndex > lastStart) startIndex = lastStart;

    /*────────────────── Suavizado del Scroll ──────────────────*/
    targetScrollOffset = startIndex * stepY;
    const int maxScrollPixel = lastStart * stepY;

    if (targetScrollOffset < 0) targetScrollOffset = 0;
    if (targetScrollOffset > maxScrollPixel) targetScrollOffset = maxScrollPixel;

    const int delta = targetScrollOffset - scrollOffset;
    if (delta >= -1 && delta <= 1) scrollOffset = targetScrollOffset;
    else scrollOffset += (delta / 4);

    /*────────────────── Viewport (sprite) con fallback ──────────────────*/
    const int wantedListW = screenW;
    const int wantedListH = kVisibleRows * stepY;

    bool listReady = true;
    if (!listSprite.created() || listW != wantedListW || listH != wantedListH) {
        if (listSprite.created()) listSprite.deleteSprite();
        void* fb = listSprite.createSprite(wantedListW, wantedListH);
        if (!fb) {
            listReady = false;
            listW = listH = 0;
            DEBUG__________ln("[UI][MODOS] listSprite.createSprite FAIL");
        } else {
            listW = wantedListW;
            listH = wantedListH;
        }
    }

    // Limpiar área de lista en uiSprite (siempre), para que el fallback sea limpio.
    uiSprite.fillRect(0, kListStartY, screenW, wantedListH, BACKGROUND_COLOR);

    // Preparar destino de dibujo
    TFT_eSprite* dst = listReady ? &listSprite : &uiSprite;

    if (listReady) {
        dst->fillSprite(BACKGROUND_COLOR);
    }

    dst->setFreeFont(&FreeSans9pt7b);
    dst->setTextSize(1);
    dst->setTextDatum(TL_DATUM);
    dst->setTextColor(TEXT_COLOR);

    const int listYOffset = -(scrollOffset - startIndex * stepY);
    const int firstVI = startIndex - 1;
    const int lastVI  = startIndex + kVisibleRows;

    // Helper local: truncar a ancho (sin heap extra)
    auto drawTruncated = [&](TFT_eSprite* spr, const String& s, int x, int y, int maxW) {
        char buf[64];
        const char* src = s.c_str();
        strncpy(buf, src ? src : "", sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = 0;

        size_t len = strlen(buf);
        while (len > 0 && spr->textWidth(buf) > maxW) {
            buf[--len] = 0;
        }
        spr->drawString(buf, x, y);
    };

    for (int i = firstVI; i <= lastVI; ++i) {
        if (i < 0 || i >= totalEntries) continue;

        const ModeCacheEntry& item = cachedModes[i];
        const int realIndex        = item.originalIndex;
        const String& baseLabelRef = item.label;
        String label               = baseLabelRef;

        if (realIndex >= 0) {
            const int altIdx = i - 1;
            bool alternateActive = false;
            if (altIdx >= 0 && (size_t)altIdx < currentAlternateStates.size()) {
                alternateActive = currentAlternateStates[altIdx];
            }
            label = getModeDisplayName(baseLabelRef, alternateActive);
        }

        int yRel = kRowTopInset + listYOffset + (i - startIndex) * stepY;

        // Convertir a coordenadas del dst
        int y = yRel;
        if (!listReady) {
            // Dibujando en uiSprite: desplazar al área de lista
            y = kListStartY + yRel;
        }

        // Culling
        const int dstH = listReady ? listH : (kListStartY + wantedListH);
        if (listReady) {
            if (yRel <= -CARD_HEIGHT + kHLPadY || yRel >= listH - kHLPadY) continue;
        } else {
            // Culling en área lista
            if (y <= (kListStartY - CARD_HEIGHT + kHLPadY) || y >= (kListStartY + wantedListH - kHLPadY)) continue;
        }

        const bool isSelected = (i == currentModeIndex);

        if (isSelected) {
            int rectY = (listReady ? yRel : y) - kHLPadY;
            int rectH = CARD_HEIGHT + (kHLPadY * 2);

            // Clamp solo cuando usamos listSprite (viewport real). En fallback, clamp al área lista.
            if (listReady) {
                if (rectY < 0) { rectH += rectY; rectY = 0; }
                if (rectY + rectH > listH) { rectH = listH - rectY; }
            } else {
                const int top = kListStartY;
                const int bot = kListStartY + wantedListH;
                if (rectY < top) { rectH -= (top - rectY); rectY = top; }
                if (rectY + rectH > bot) { rectH = bot - rectY; }
            }

            if (rectH > 0) {
                dst->fillRoundRect(
                    kListLeftX - kHLPadX,
                    rectY,
                    textAreaW + (kHLPadX * 2),
                    rectH,
                    kCardRadius,
                    CARD_COLOR
                );
            }
        }

        // Texto y ticker
        if (isSelected) {
            if (currentModeIndex != lastSelectedMode) {
                modeTickerOffset = 0;
                modeTickerDirection = 1;
                lastSelectedMode = currentModeIndex;
            }

            const int fullW = dst->textWidth(label);
            if (fullW > textAreaW && listReady) {
                // Ticker SOLO si estamos en listSprite (si no, simplificamos y truncamos)
                const unsigned long now = millis();
                if (now - modeLastFrameTime >= kTickerFrameMs) {
                    modeTickerOffset += modeTickerDirection;
                    if (modeTickerOffset < 0) { modeTickerOffset = 0; modeTickerDirection = 1; }
                    const int maxOffset = fullW - textAreaW;
                    if (modeTickerOffset > maxOffset) { modeTickerOffset = maxOffset; modeTickerDirection = -1; }
                    modeLastFrameTime = now;
                }

                bool tickerReady = true;
                if (!tickerSprite.created() || tickerW != textAreaW || tickerH != CARD_HEIGHT) {
                    if (tickerSprite.created()) tickerSprite.deleteSprite();
                    void* fb2 = tickerSprite.createSprite(textAreaW, CARD_HEIGHT);
                    if (!fb2) {
                        tickerReady = false;
                        tickerW = tickerH = 0;
                        DEBUG__________ln("[UI][MODOS] tickerSprite.createSprite FAIL");
                    } else {
                        tickerW = textAreaW;
                        tickerH = CARD_HEIGHT;
                    }
                }

                if (tickerReady) {
                    tickerSprite.fillSprite(CARD_COLOR);
                    tickerSprite.setFreeFont(&FreeSans9pt7b);
                    tickerSprite.setTextSize(1);
                    tickerSprite.setTextDatum(TL_DATUM);
                    tickerSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
                    tickerSprite.drawString(label, -modeTickerOffset, 0);

                    // yRel es el “y dentro del viewport”
                    tickerSprite.pushToSprite(&listSprite, kListLeftX, yRel);
                } else {
                    // Si no hay ticker sprite, al menos truncar
                    drawTruncated(dst, label, kListLeftX, (listReady ? yRel : y), textAreaW);
                }
            } else {
                // Cabe, o estamos en fallback: dibuja (truncado si hace falta)
                if (fullW > textAreaW) drawTruncated(dst, label, kListLeftX, (listReady ? yRel : y), textAreaW);
                else dst->drawString(label, kListLeftX, (listReady ? yRel : y));
            }
        } else {
            // No seleccionado
            const int fullW = dst->textWidth(label);
            if (fullW > textAreaW) drawTruncated(dst, label, kListLeftX, (listReady ? yRel : y), textAreaW);
            else dst->drawString(label, kListLeftX, (listReady ? yRel : y));
        }
    }

    // Volcado final
    if (listReady) {
        listSprite.pushToSprite(&uiSprite, 0, kListStartY);
    }
    uiSprite.pushSprite(0, 0);
}

// Opciones del menú oculto
const char* menuOptions[] = {
    "IDIOMA",
    "SONIDO",
    "BRILLO",
    "MIC_CALIB",
    "CONTROL_SALA_MENU",
    "VOLVER"
};
const int numOptions = sizeof(menuOptions) / sizeof(menuOptions[0]);

// Número máximo de opciones visibles
const int visibleOptions = 4;

/**
 * @brief Dibuja el “menú oculto” (ajustes) con ventana desplazable y recorte de texto.
 *
 * Centra un título traducido, calcula una ventana de filas visibles (controlada por
 * `visibleOptions`), aplica *scroll suave* vertical mediante `scrollOffset` y
 * dibuja cada opción truncando el texto que no cabe en el ancho disponible.
 * La opción bajo el cursor se resalta con un rectángulo redondeado. Se usa un
 * sprite intermedio para realizar clipping de la lista y volcarlo al `uiSprite`.
 *
 * @param selection Índice de la opción actualmente seleccionada (0..numOptions-1).
 */
void drawHiddenMenu(int selection)
{
    /*──────── Config (dinámica y constantes) ───────*/
    const int screenW         = tft.width();
    constexpr int kTitleY     = 5;
    constexpr int kListStartY = 30;
    constexpr int kCardRadius = 3;
    constexpr int kHLPadX     = 3;
    constexpr int kHLPadY     = 1;
    constexpr int kRowTopInset= (kHLPadY + 1);

    const int cardHeight      = CARD_HEIGHT;
    const int cardMargin      = CARD_MARGIN;
    const int stepY           = cardHeight + cardMargin;

    const int x               = 9;
    constexpr int kScrollBarMargin = 3;
    constexpr int kScrollBarW      = 5;
    const int scrollBarX      = screenW - kScrollBarW - kScrollBarMargin;
    const int textAreaW       = scrollBarX - x - 2;

    /*──────── Validaciones defensivas ───────*/
    if (numOptions <= 0) {
        uiSprite.fillSprite(BACKGROUND_COLOR);
        setFontForCurrentLanguage();
        uiSprite.setTextColor(TEXT_COLOR);
        uiSprite.setTextDatum(TC_DATUM);
        uiSprite.setTextSize(1);
        uiSprite.drawString(getTranslation("MENU_AJUSTES"), screenW / 2, kTitleY);
        uiSprite.pushSprite(0, 0);
        return;
    }
    if (selection < 0) selection = 0;
    if (selection >= numOptions) selection = numOptions - 1;

    const int kVisibleRows   = (visibleOptions > 0) ? visibleOptions : 1;
    const int safeTextAreaW  = (textAreaW > 0) ? textAreaW : (screenW - x - 2);
    const int wantedListH    = kVisibleRows * stepY;

    /*──────── Estado de scroll (persistente) ───────*/
    static int scrollOffset       = 0;
    static int targetScrollOffset = 0;
    static int startIndex         = 0;

    /*──────── Cabecera ───────*/
    uiSprite.fillSprite(BACKGROUND_COLOR);
    setFontForCurrentLanguage();
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString(getTranslation("MENU_AJUSTES"), screenW / 2, kTitleY);

    /*──────── Ventana (startIndex) ───────*/
    if ((selection - startIndex) > (kVisibleRows - 2) && startIndex < (numOptions - kVisibleRows)) {
        ++startIndex;
    } else if ((selection - startIndex) < 1 && startIndex > 0) {
        --startIndex;
    }

    if (startIndex < 0) startIndex = 0;
    const int lastStart = (numOptions > kVisibleRows) ? (numOptions - kVisibleRows) : 0;
    if (startIndex > lastStart) startIndex = lastStart;

    /*──────── Scroll suave ───────*/
    targetScrollOffset = startIndex * stepY;
    const int maxScroll = lastStart * stepY;

    if (targetScrollOffset < 0)         targetScrollOffset = 0;
    if (targetScrollOffset > maxScroll) targetScrollOffset = maxScroll;

    const int delta = targetScrollOffset - scrollOffset;
    if (delta >= -1 && delta <= 1) scrollOffset = targetScrollOffset;
    else scrollOffset += (delta / 4);

    /*──────── Viewport (sprite) con fallback ───────*/
    static TFT_eSprite listSprite(&tft);
    static int listW = 0, listH = 0;

    const int wantedListW = screenW;

    bool listReady = true;
    if (!listSprite.created() || listW != wantedListW || listH != wantedListH) {
        if (listSprite.created()) listSprite.deleteSprite();
        void* fb = listSprite.createSprite(wantedListW, wantedListH);
        if (!fb) {
            listReady = false;
            listW = listH = 0;
            DEBUG__________ln("[UI][AJUSTES] listSprite.createSprite FAIL");
        } else {
            listW = wantedListW;
            listH = wantedListH;
        }
    }

    // Limpiar área lista SIEMPRE en uiSprite para que el fallback sea limpio
    uiSprite.fillRect(0, kListStartY, screenW, wantedListH, BACKGROUND_COLOR);

    TFT_eSprite* dst = listReady ? &listSprite : &uiSprite;

    if (listReady) {
        dst->fillSprite(BACKGROUND_COLOR);
    }

    dst->setFreeFont(&FreeSans9pt7b);
    dst->setTextColor(TEXT_COLOR);
    dst->setTextSize(1);
    dst->setTextDatum(TL_DATUM);

    const int listYOffset = -(scrollOffset - startIndex * stepY);
    const int firstVI = startIndex - 1;
    const int lastVI  = startIndex + kVisibleRows;

    // Helper: truncar sin usar String/remove()
    auto drawTruncCStr = [&](TFT_eSprite* spr, const char* src, int px, int py, int maxW) {
        char buf[64];
        strncpy(buf, (src ? src : ""), sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = 0;

        size_t len = strlen(buf);
        while (len > 0 && spr->textWidth(buf) > maxW) {
            buf[--len] = 0;
        }
        spr->drawString(buf, px, py);
    };

    for (int vi = firstVI; vi <= lastVI; ++vi) {
        if (vi < 0 || vi >= numOptions) continue;

        const int yRel = kRowTopInset + listYOffset + (vi - startIndex) * stepY;
        const int y    = listReady ? yRel : (kListStartY + yRel);

        // Culling
        if (listReady) {
            if (yRel <= -cardHeight + kHLPadY || yRel >= listH - kHLPadY) continue;
        } else {
            if (y <= (kListStartY - cardHeight + kHLPadY) || y >= (kListStartY + wantedListH - kHLPadY)) continue;
        }

        const bool isSelected = (vi == selection);

        if (isSelected) {
            int rectY = (listReady ? yRel : y) - kHLPadY;
            int rectH = cardHeight + (kHLPadY * 2);

            if (listReady) {
                if (rectY < 0)             { rectH += rectY; rectY = 0; }
                if (rectY + rectH > listH) { rectH = listH - rectY; }
            } else {
                const int top = kListStartY;
                const int bot = kListStartY + wantedListH;
                if (rectY < top) { rectH -= (top - rectY); rectY = top; }
                if (rectY + rectH > bot) { rectH = bot - rectY; }
            }

            if (rectH > 0) {
                dst->fillRoundRect(
                    x - kHLPadX,
                    rectY,
                    safeTextAreaW + (kHLPadX * 2),
                    rectH,
                    kCardRadius,
                    CARD_COLOR
                );
            }
        }

        const char* label = getTranslation(menuOptions[vi]);
        drawTruncCStr(dst, label, x, (listReady ? yRel : y), safeTextAreaW);
    }

    if (listReady) {
        listSprite.pushToSprite(&uiSprite, 0, kListStartY);
    }
    uiSprite.pushSprite(0, 0);
}


#ifdef MIC
extern uint8_t micSilenceThreshold;

// Colores definidos
#define UI_BAR_BG      0x2124 // Gris muy oscuro
#define UI_BAR_FILL    0x07E0 // Verde
#define UI_BAR_TRIG    0xF800 // Rojo

// void drawMicCalibMenu(uint8_t editingThreshold, uint8_t liveRaw, bool showSaved)
// {
//     // 1. Limpiar fondo
//     uiSprite.fillSprite(TFT_BLACK);

//     // --- TÍTULO ---
//     uiSprite.setTextDatum(TC_DATUM);
//     uiSprite.setTextColor(TFT_SILVER, TFT_BLACK);
//     uiSprite.setFreeFont(&FreeSans9pt7b);
//     uiSprite.drawString("CALIBRACION", 64, 2);

//     // Definición de geometría de la barra
//     const int barW = 114;
//     const int barH = 20;
//     const int barX = (128 - barW) / 2;
//     const int barY = 56; // Bajamos la barra un pelín para dar aire arriba

//     // --- ETIQUETA ---
//     // Ahora la dibujamos más arriba para que no toque el triángulo
//     uiSprite.setTextDatum(BC_DATUM);
//     uiSprite.setTextFont(1); 
//     uiSprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
//     // Antes estaba en (barY - 4), ahora (barY - 15) para dejar hueco
//     uiSprite.drawString("Ajuste sensibilidad", 64, barY - 15);

//     // 2. Fondo de la barra
//     uiSprite.fillRoundRect(barX, barY, barW, barH, 4, UI_BAR_BG);

//     // 3. Lógica de color (Verde vs Rojo)
//     uint16_t barColor = (liveRaw > editingThreshold) ? UI_BAR_TRIG : UI_BAR_FILL;

//     // 4. Relleno de la barra (Ruido actual)
//     int fillW = map((int)liveRaw, 0, 255, 0, barW);
//     fillW = constrain(fillW, 0, barW);
    
//     if (fillW > 0) {
//         uiSprite.fillRoundRect(barX, barY, fillW, barH, 4, barColor);
//         // Truco visual: parchear esquinas derechas si no está llena
//         if (fillW < barW - 4) {
//             uiSprite.fillRect(barX + fillW - 2, barY, 2, barH, barColor); 
//         }
//     }

//     // 5. Marca del Umbral (Gate)
//     int markX = map((int)editingThreshold, 0, 255, 0, barW);
//     markX = constrain(markX, 0, barW);
//     int absMarkX = barX + markX;

//     // Línea vertical
//     uiSprite.drawLine(absMarkX, barY - 3, absMarkX, barY + barH + 3, TFT_WHITE);
//     // Triángulo superior
//     uiSprite.fillTriangle(absMarkX - 4, barY - 8, absMarkX + 4, barY - 8, absMarkX, barY - 2, TFT_WHITE);
//     // Triángulo inferior
//     uiSprite.fillTriangle(absMarkX - 4, barY + barH + 8, absMarkX + 4, barY + barH + 8, absMarkX, barY + barH + 2, TFT_WHITE);


//     // --- FOOTER ---
//     uiSprite.setTextDatum(BC_DATUM);
//     uiSprite.setTextFont(1);

//     if (showSaved) {
//         uiSprite.setTextColor(TFT_GREEN, TFT_BLACK);
//         uiSprite.setFreeFont(&FreeSans9pt7b);
//         uiSprite.drawString("GUARDADO", 64, 115);
//     } else {
//         uiSprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
//         // Instrucciones apiladas
//         uiSprite.drawString("Girar: Ajustar", 64, 116);
//         uiSprite.drawString("Click: Guardar", 64, 127);
//     }

//     uiSprite.pushSprite(0, 0);
// }

void drawMicCalibMenu(uint8_t editingThreshold, uint8_t liveRaw, bool showSaved)
{
    // 1. Limpiar fondo
    uiSprite.fillSprite(TFT_BLACK);

    // --- TÍTULO ---
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextColor(TFT_SILVER, TFT_BLACK);
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.drawString("CALIBRACION", 64, 2);

    // Definición de geometría de la barra
    const int barW = 114;
    const int barH = 20;
    const int barX = (128 - barW) / 2;
    const int barY = 56;

    // --- ETIQUETA ---
    uiSprite.setTextDatum(BC_DATUM);
    uiSprite.setTextFont(1);
    uiSprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
    uiSprite.drawString("Ajuste sensibilidad", 64, barY - 15);

    // 2. Fondo de la barra
    uiSprite.fillRoundRect(barX, barY, barW, barH, 4, UI_BAR_BG);

    // 3. Lógica de color (Verde vs Rojo) -> SIEMPRE con el valor REAL
    uint16_t barColor = (liveRaw > editingThreshold) ? UI_BAR_TRIG : UI_BAR_FILL;

    // 4. Relleno de la barra (VISUAL “capado”)
    // Esto NO cambia la calibración, solo hace la UI más utilizable.
    // Ajusta este techo: si lo bajas, se llenará antes; si lo subes, será más “real”.
    constexpr uint8_t kVisualCeil = 180; // recomendado: 160..200

    uint8_t visualByte = liveRaw;
    if (visualByte >= kVisualCeil) {
        visualByte = 255;
    } else {
        visualByte = (uint8_t)map((int)visualByte, 0, (int)kVisualCeil, 0, 255);
    }

    int fillW = map((int)visualByte, 0, 255, 0, barW);
    fillW = constrain(fillW, 0, barW);

    if (fillW > 0) {
        uiSprite.fillRoundRect(barX, barY, fillW, barH, 4, barColor);
        // Truco visual: parchear esquinas derechas si no está llena
        if (fillW < barW - 4) {
            uiSprite.fillRect(barX + fillW - 2, barY, 2, barH, barColor);
        }
    }

    // 5. Marca del Umbral (Gate) -> umbral REAL (0..255)
    int markX = map((int)editingThreshold, 0, 255, 0, barW);
    markX = constrain(markX, 0, barW);
    int absMarkX = barX + markX;

    // Línea vertical
    uiSprite.drawLine(absMarkX, barY - 3, absMarkX, barY + barH + 3, TFT_WHITE);
    // Triángulo superior
    uiSprite.fillTriangle(absMarkX - 4, barY - 8, absMarkX + 4, barY - 8, absMarkX, barY - 2, TFT_WHITE);
    // Triángulo inferior
    uiSprite.fillTriangle(absMarkX - 4, barY + barH + 8, absMarkX + 4, barY + barH + 8, absMarkX, barY + barH + 2, TFT_WHITE);

    // --- FOOTER ---
    uiSprite.setTextDatum(BC_DATUM);
    uiSprite.setTextFont(1);

    if (showSaved) {
        uiSprite.setTextColor(TFT_GREEN, TFT_BLACK);
        uiSprite.setFreeFont(&FreeSans9pt7b);
        uiSprite.drawString("GUARDADO", 64, 115);
    } else {
        uiSprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
        uiSprite.drawString("Girar: Ajustar", 64, 116);
        uiSprite.drawString("Click: Guardar", 64, 127);
    }

    uiSprite.pushSprite(0, 0);
}

#endif

// ---- Mic Calibration Menu ----
// Variable runtime definida en encoder_handler.cpp
// #ifdef MIC
// extern uint8_t micSilenceThreshold;

// void drawMicCalibMenu(uint8_t editingThreshold, uint8_t liveRaw, bool showSaved)
// {
//     // Fondo
//     uiSprite.fillSprite(BACKGROUND_COLOR);

//     // Título
//     uiSprite.setTextDatum(TC_DATUM);
//     uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
//     uiSprite.setFreeFont(&FreeSans9pt7b);
//     uiSprite.drawString("Calibración micrófono", tft.width() / 2, 6);

//     // “Card” central
//     const int x = 8;
//     const int y = 28;
//     const int w = tft.width() - 16;
//     const int h = 76;

//     uiSprite.fillRoundRect(x, y, w, h, 6, TFT_DARKGREY);
//     uiSprite.drawRoundRect(x, y, w, h, 6, TEXT_COLOR);

//     // Etiquetas
//     uiSprite.setTextDatum(TL_DATUM);
//     uiSprite.setTextColor(WHITE, TFT_DARKGREY);
//     uiSprite.setFreeFont(&FreeSans9pt7b);
//     uiSprite.drawString("Umbral silencio", x + 10, y + 10);

//     // Valor grande
//     uiSprite.setTextDatum(TR_DATUM);
//     uiSprite.setFreeFont(&FreeSans12pt7b);
//     uiSprite.drawString(String(editingThreshold), x + w - 10, y + 12);

//     // Medidor de ruido actual
//     uiSprite.setTextDatum(TL_DATUM);
//     uiSprite.setFreeFont(&FreeSans9pt7b);
//     uiSprite.drawString("Ruido actual", x + 10, y + 42);

//     // Barra 0..255
//     const int barX = x + 10;
//     const int barY = y + 58;
//     const int barW = w - 20;
//     const int barH = 10;

//     uiSprite.drawRoundRect(barX, barY, barW, barH, 3, WHITE);

//     const int fillW = map((int)liveRaw, 0, 255, 0, barW - 2);
//     uiSprite.fillRoundRect(barX + 1, barY + 1, fillW, barH - 2, 2, WHITE);

//     // Marca del umbral
//     const int markX = barX + 1 + map((int)editingThreshold, 0, 255, 0, barW - 2);
//     uiSprite.drawLine(markX, barY - 3, markX, barY + barH + 3, RED);

//     // Footer UX
//     uiSprite.setTextDatum(BC_DATUM);
//     uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
//     uiSprite.setFreeFont(&FreeSans9pt7b);
//     uiSprite.drawString("Gira: ajustar  |  Click: guardar", tft.width() / 2, tft.height() - 6);

//     if (showSaved) {
//         uiSprite.setTextDatum(TC_DATUM);
//         uiSprite.setTextColor(GREEN, BACKGROUND_COLOR);
//         uiSprite.drawString("Guardado", tft.width() / 2, y + h + 6);
//     }

//     uiSprite.pushSprite(0, 0);
// }
// #endif


/**
 * @brief Dibuja un “ticker” horizontal con vaivén para la opción seleccionada
 *        en menús oculto/sonido.
 * Calcula una ventana visible de 3 filas y, si el texto de la opción en cursor
 * excede el ancho disponible (≈110 px), hace scroll horizontal de ida y vuelta.
 * @param selection Índice global de la opción actualmente en cursor.
 */
void scrollTextTickerBounce(int selection)
{
    // Solo activo en estos menús
    if (!hiddenMenuActive && !soundMenuActive) return;

    // ───────── Constantes de layout/ticker ─────────
    constexpr int   kCardWidth       = 110;        // ancho útil aprox. del texto
    const     int   kCardHeight      = CARD_HEIGHT;
    const     int   kCardMargin      = CARD_MARGIN;
    constexpr int   kVisibleOptions  = 3;          // filas visibles en este contexto
    constexpr unsigned long kFrameMs = 10;         // cadencia del ticker
    constexpr int   kTextX           = 10;         // alineado con drawHiddenMenu
    constexpr int   kListStartY      = 30;
    constexpr int   kMaxVisibleSlots = 5;          // tamaño fijo de buffers internos

    // ───────── Estado por-slot (no por selección) ─────────
    static int  charOffsets[kMaxVisibleSlots]     = {0, 0, 0, 0, 0};
    static int  scrollDirections[kMaxVisibleSlots]= {1, 1, 1, 1, 1};
    static unsigned long lastFrameTime = 0;

    // ───────── Validaciones básicas ─────────
    if (selection < 0 || selection >= numOptions) return;

    // Texto fuente (se mantiene uso de menuOptions para compatibilidad)
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setTextWrap(false);
    tft.setTextSize(1);

    String text = String(getTranslation(menuOptions[selection]));
    const int fullTextWidth = tft.textWidth(text.c_str());
    const int textVisibleW  = kCardWidth;
    if (fullTextWidth <= textVisibleW) return; // no hace falta ticker

    // Tamaño del “chunk” visible (caracteres que caben)
    int chunkSize = 0;
    while (chunkSize < text.length() &&
           tft.textWidth(text.substring(0, chunkSize + 1)) <= textVisibleW)
    {
        ++chunkSize;
    }
    if (chunkSize == 0) chunkSize = 1; // defensa

    // Ventana de filas visibles y slot local
    const int startIndex = std::max(0, std::min(selection - (kVisibleOptions / 2),
                                                numOptions - kVisibleOptions));
    const int cardIndex  = selection - startIndex; // 0..kVisibleOptions-1
    if (cardIndex < 0 || cardIndex >= kVisibleOptions) return;

    // Mapear al buffer interno de tamaño fijo (evita OOB con selection grande)
    if (cardIndex >= kMaxVisibleSlots) return; // defensa adicional

    // Coordenadas del área de texto dentro de la fila
    const int stepY     = kCardHeight + kCardMargin;
    const int cardY     = kListStartY + cardIndex * stepY;
    const int textAreaX = kTextX;
    const int textAreaY = cardY;
    const int textAreaW = textVisibleW - 2;
    const int textAreaH = kCardHeight - 2;

    // ───────── Avance del ticker (vaivén) ─────────
    const unsigned long now = millis();
    if (now - lastFrameTime >= kFrameMs) {
        charOffsets[cardIndex] += scrollDirections[cardIndex];

        if (charOffsets[cardIndex] < 0) {
            charOffsets[cardIndex]   = 0;
            scrollDirections[cardIndex] = +1;
        }

        int maxOffset = text.length() - chunkSize;
        if (maxOffset < 0) maxOffset = 0;

        if (charOffsets[cardIndex] > maxOffset) {
            charOffsets[cardIndex]   = maxOffset;
            scrollDirections[cardIndex] = -1;
        }
        lastFrameTime = now;
    }

    // ───────── Sprite temporal para el ticker ─────────
    TFT_eSprite tickerSprite(&tft);
    if (tickerSprite.createSprite(textAreaW, textAreaH) == nullptr) return;

    tickerSprite.setFreeFont(&FreeSans9pt7b);
    tickerSprite.setTextWrap(false);
    tickerSprite.setTextSize(1);
    tickerSprite.setTextColor(HIGHLIGHT_COLOR);
    tickerSprite.setTextDatum(TL_DATUM);

    // Fondo coherente con la fila seleccionada
    tickerSprite.fillSprite(CARD_COLOR);

    // Subcadena visible + desplazamiento
    int offset = charOffsets[cardIndex];
    if (offset < 0) offset = 0;
    if (offset >= static_cast<int>(text.length())) offset = static_cast<int>(text.length()) - 1;

    int endIndex = offset + chunkSize;
    if (endIndex > static_cast<int>(text.length())) endIndex = static_cast<int>(text.length());

    const String sliceStr = text.substring(offset, endIndex);

    // Nota: el desplazamiento X negativo produce el “slide” suave
    tickerSprite.drawString(sliceStr, -charOffsets[cardIndex], 0);

    // Componer sobre el sprite principal (evita que un push posterior lo tape)
    tickerSprite.pushToSprite(&uiSprite, textAreaX, textAreaY);
    tickerSprite.deleteSprite();
}

String getModeDisplayName(const String &fullModeName, bool alternateActive) {
    int sepIndex = fullModeName.indexOf('/');
    // Si no hay separador, se devuelve la cadena completa
    if (sepIndex == -1) {
        return fullModeName;
    }
    // Devuelve la primera parte si no está activa la alternativa,
    // o la segunda parte si alternateActive es true.
    return alternateActive ? fullModeName.substring(sepIndex + 1) : fullModeName.substring(0, sepIndex);
}

// Función para dibujar el menú de selección de idioma con scroll vertical
void drawLanguageMenu(int selection) {
    // Opciones de idioma: se definen 8 opciones
    const char* languageOptions[] = {"ES", "ES(MX)", "CA", "EU", "FR", "DE", "EN", "IT"};
    const int numLanguages = 8;
    const int visibleOptions = 4;

    int startIndex = 0;
    if (selection >= visibleOptions) {
        startIndex = selection - visibleOptions + 1;
    }

    uiSprite.fillSprite(BACKGROUND_COLOR);

    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString(getTranslation("IDIOMA"), 64, 5);

    const int startY = 30;
    const int cardHeight = CARD_HEIGHT;
    const int cardMargin = CARD_MARGIN;
    const int x = 10;
    const int textAreaW = 110;

    for (int i = 0; i < visibleOptions; i++) {
        int optionIndex = startIndex + i;
        if (optionIndex >= numLanguages) break;

        int y = startY + i * (cardHeight + cardMargin);
        bool isSelected = (optionIndex == selection);

        if (isSelected) {
            uiSprite.fillRoundRect(x - 3, y - 1, textAreaW + 6, cardHeight + 2, 3, CARD_COLOR);
        }

        uiSprite.setFreeFont(&FreeSans9pt7b);
        uiSprite.setTextColor(TEXT_COLOR);
        uiSprite.setTextSize(1);
        uiSprite.setTextDatum(TL_DATUM);
        uiSprite.drawString(languageOptions[optionIndex], x, y);
    }

    uiSprite.pushSprite(0, 0);
}

// Índice del primer elemento visible en la ventana
extern int bankMenuVisibleItems;

void drawBankSelectionMenu(const std::vector<byte>& bankList, const std::vector<bool>& selectedBanks, int currentSelection, int windowOffset) {
    // Limpiar el sprite completo.
    uiSprite.fillSprite(BACKGROUND_COLOR);
    
    // Título centrado
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString("Familia", 64, 5);
    
    // Total de opciones: 1 para "Confirmar" + cantidad de banks
    int totalItems = bankList.size() + 1;
    const int visibleOptions = 4; // Número de opciones visibles
    const int x = 10;
    const int textAreaW = 110;
    const int scrollBarX = 120;
    
    // Definir el color celeste para las opciones seleccionadas
    const uint32_t SELECTED_OPTION_COLOR = TFT_CYAN; // Color celeste para opciones marcadas
    
    // Dibujar cada opción visible dentro de la ventana determinada por windowOffset
    for (int i = 0; i < visibleOptions && (windowOffset + i) < totalItems; i++) {
        int menuIndex = windowOffset + i;  // Índice global del item
        int y = 30 + i * (CARD_HEIGHT + CARD_MARGIN); // Usando CARD_HEIGHT=20 y CARD_MARGIN=5, como en drawModesScreen
        bool isSelected = (menuIndex == currentSelection);
        bool isOptionChecked = (menuIndex > 0 && selectedBanks.size() > (size_t)(menuIndex - 1) && selectedBanks[menuIndex - 1]);
        
        // Determinar el color del texto según la selección actual y si está marcado
        uint32_t textColor;
        if (isSelected) {
            textColor = HIGHLIGHT_COLOR; // Prioridad al resaltado de la selección actual
        } else if (isOptionChecked) {
            textColor = SELECTED_OPTION_COLOR; // Color celeste para opciones marcadas pero no seleccionadas
        } else {
            textColor = TEXT_COLOR; // Color normal para opciones no marcadas
        }
        
        uiSprite.setFreeFont(&FreeSans9pt7b);
        uiSprite.setTextSize(1);
        uiSprite.setTextDatum(TL_DATUM);
        uiSprite.setTextColor(textColor);
        
        String label;
        if (menuIndex == 0) {
            label = "Confirmar";
        } else {
            // Obtener el nombre de la familia asociado al bank
            byte bank = bankList[menuIndex - 1];
            String familyName = getFamilyNameFromBank(bank);
            label = familyName;
        }
        
        // Si la opción seleccionada tiene texto muy largo, aplicar ticker horizontal similar a drawModesScreen.
        int fullTextWidth = uiSprite.textWidth(label);
        if (isSelected && fullTextWidth > textAreaW) {
            static int tickerOffset = 0;
            static int tickerDirection = 1;
            static unsigned long lastFrameTime = 0;
            const unsigned long frameInterval = 50;
            unsigned long now = millis();
            if (now - lastFrameTime >= frameInterval) {
                tickerOffset += tickerDirection;
                if (tickerOffset < 0) {
                    tickerOffset = 0;
                    tickerDirection = 1;
                }
                if (tickerOffset > (fullTextWidth - textAreaW)) {
                    tickerOffset = fullTextWidth - textAreaW;
                    tickerDirection = -1;
                }
                lastFrameTime = now;
            }
            TFT_eSprite tickerSprite = TFT_eSprite(&tft);
            tickerSprite.createSprite(textAreaW, CARD_HEIGHT);
            tickerSprite.fillSprite(BACKGROUND_COLOR);
            tickerSprite.setFreeFont(&FreeSans9pt7b);
            tickerSprite.setTextSize(1);
            tickerSprite.setTextDatum(TL_DATUM);
            tickerSprite.setTextColor(textColor, BACKGROUND_COLOR);
            tickerSprite.drawString(label, -tickerOffset, 0);
            tickerSprite.pushToSprite(&uiSprite, x, y);
            tickerSprite.deleteSprite();
        } else {
            uiSprite.drawString(label, x, y);
        }
    }

    
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Dibuja el menú de sonido con scroll, resaltado y estado de selección.
 *
 * Muestra 10 entradas (con separadores visuales) y resalta la fila bajo el cursor.
 * Centra el título, recorta textos que no caben y, si la opción en cursor desborda,
 * delega el scroll del texto a `scrollTextTickerBounce(selection)`. Incluye barra de
 * scroll con "thumb" proporcional.
 *
 * @note Los separadores se dibujan en gris y no son seleccionables lógicamente.
 */
// void drawSoundMenu(int selection)
// {
//     /*──────── Constantes/layout (como en Modos) ────────*/
//     const int screenW         = tft.width();
//     constexpr int kTitleY     = 5;
//     constexpr int kListStartY = 30;
//     constexpr int kCardRadius = 3;
//     constexpr int kHLPadX     = 3;
//     constexpr int kHLPadY     = 1;
//     constexpr int kRowTopInset= (kHLPadY + 1);
//     constexpr int kTickerFrameMs = 50;

//     constexpr int kVisibleOptions  = 4;
//     constexpr int kTotalOptions    = 10;
//     constexpr int kScrollBarW      = 5;
//     constexpr int kScrollBarRightP = 3;
//     constexpr int kListLeftX       = 9;
//     constexpr int kTextAreaPad     = 2;
//     static constexpr const char kSep[] = "---------";

//     const char* options[kTotalOptions] = {
//         getTranslation("MUJER"),
//         getTranslation("HOMBRE"),
//         kSep,
//         getTranslation("CON_NEG"),
//         getTranslation("SIN_NEG"),
//         kSep,
//         getTranslation("VOL_NORMAL"),
//         getTranslation("VOL_ATENUADO"),
//         kSep,
//         getTranslation("CONFIRMAR")
//     };

//     const int cardHeight  = CARD_HEIGHT;
//     const int cardMargin  = CARD_MARGIN;
//     const int stepY       = cardHeight + cardMargin;

//     const int scrollBarX  = screenW - kScrollBarW - kScrollBarRightP;
//     const int textAreaW   = scrollBarX - kListLeftX - kTextAreaPad;

//     /*──────── Estado scroll persistente ────────*/
//     static int scrollOffset       = 0;   // px actuales
//     static int targetScrollOffset = 0;   // px destino

//     /*──────── Estado del ticker (persistente) ────────*/
//     static int lastSelectedIndex  = -1;
//     static int sndTickerOffset    = 0;
//     static int sndTickerDirection = 1;   // 1→derecha, -1→izquierda
//     static unsigned long sndLastFrameTime = 0;
//     static TFT_eSprite tickerSprite(&tft);
//     static int tickerW = 0, tickerH = 0;

//     /*──────── Fondo + título ────────*/
//     uiSprite.fillSprite(BACKGROUND_COLOR);
//     uiSprite.setFreeFont(&FreeSans12pt7b);
//     uiSprite.setTextColor(TEXT_COLOR);
//     uiSprite.setTextDatum(TC_DATUM);
//     uiSprite.setTextSize(1);
//     uiSprite.drawString(getTranslation("SONIDO"), screenW/2, kTitleY);

//     /*──────── Ventana rodante (misma lógica que Modos) ────────*/
//     // selection aquí es el índice lógico 0..9 (tú ya te saltas separadores en handle)
//     static int startIndex = 0;
//     if ((selection - startIndex) > (kVisibleOptions - 2) &&
//         startIndex < (kTotalOptions - kVisibleOptions))
//         startIndex++;
//     else if ((selection - startIndex) < 1 && startIndex > 0)
//         startIndex--;

//     if (startIndex < 0) startIndex = 0;
//     const int lastStart = (kTotalOptions > kVisibleOptions) ? (kTotalOptions - kVisibleOptions) : 0;
//     if (startIndex > lastStart) startIndex = lastStart;

//     /*──────── Resolver SCROLL ANTES de dibujar ────────*/
//     targetScrollOffset = startIndex * stepY;
//     const int maxScroll = lastStart * stepY;
//     if (targetScrollOffset < 0)         targetScrollOffset = 0;
//     if (targetScrollOffset > maxScroll) targetScrollOffset = maxScroll;

//     int delta = targetScrollOffset - scrollOffset;
//     if (delta >= -1 && delta <= 1) scrollOffset = targetScrollOffset;
//     else                           scrollOffset += (delta / 4);

//     /*──────── Viewport con clipping ────────*/
//     static TFT_eSprite listSprite(&tft);
//     static int listW = 0, listH = 0;
//     const int wantedListW = screenW;
//     const int wantedListH = kVisibleOptions * stepY;

//     if (!listSprite.created() || listW != wantedListW || listH != wantedListH) {
//         if (listSprite.created()) listSprite.deleteSprite();
//         listSprite.createSprite(wantedListW, wantedListH);
//         listW = wantedListW; listH = wantedListH;
//     }

//     listSprite.fillSprite(BACKGROUND_COLOR);
//     listSprite.setFreeFont(&FreeSans9pt7b);
//     listSprite.setTextSize(1);
//     listSprite.setTextDatum(TL_DATUM);

//     const int listYOffset = -(scrollOffset - startIndex * stepY);
//     const int firstVI     = startIndex - 1;
//     const int lastVI      = startIndex + kVisibleOptions;

//     for (int vi = firstVI; vi <= lastVI; ++vi)
//     {
//         if (vi < 0 || vi >= kTotalOptions) continue;

//         const int y = kRowTopInset + listYOffset + (vi - startIndex) * stepY;
//         if (y <= -cardHeight + kHLPadY || y >= listH - kHLPadY) continue;

//         const char* labelC = options[vi];
//         const bool isSeparator = (strcmp(labelC, kSep) == 0);
//         const bool isCursor    = (vi == selection);

//         // Fondo resaltado sólo si es cursor y no es separador
//         if (isCursor && !isSeparator) {
//             int rectY = y - kHLPadY;
//             int rectH = cardHeight + (kHLPadY * 2);
//             if (rectY < 0)            { rectH += rectY; rectY = 0; }
//             if (rectY + rectH > listH){ rectH = listH - rectY; }
//             if (rectH > 0) {
//                 listSprite.fillRoundRect(
//                     kListLeftX - kHLPadX, rectY,
//                     textAreaW + (kHLPadX * 2),
//                     rectH, kCardRadius,
//                     CARD_COLOR
//                 );
//             }
//         }

//         // Color lógico seleccionado (tu misma lógica)
//         const bool isLogicallySelected =
//             (vi == 0 && selectedVoiceGender == 0) ||
//             (vi == 1 && selectedVoiceGender == 1) ||
//             (vi == 3 && negativeResponse)         ||
//             (vi == 4 && !negativeResponse)        ||
//             (vi == 6 && selectedVolume == 0)      ||
//             (vi == 7 && selectedVolume == 1);

//         const uint16_t textColor =
//             isSeparator ? TFT_DARKGREY :
//             (isLogicallySelected ? HIGHLIGHT_COLOR : TEXT_COLOR);

//         listSprite.setTextColor(textColor);

//         String label = String(labelC);

//         if (isCursor && !isSeparator) {
//             // —— TICKER integrado (igual que en Modos) ——
//             if (selection != lastSelectedIndex) {
//                 sndTickerOffset    = 0;
//                 sndTickerDirection = 1;
//                 lastSelectedIndex  = selection;
//             }

//             const int fullW = listSprite.textWidth(label);
//             if (fullW > textAreaW) {
//                 const unsigned long now = millis();
//                 if (now - sndLastFrameTime >= kTickerFrameMs) {
//                     sndTickerOffset += sndTickerDirection;
//                     if (sndTickerOffset < 0) { sndTickerOffset = 0; sndTickerDirection = 1; }
//                     const int maxOffsetX = fullW - textAreaW;
//                     if (sndTickerOffset > maxOffsetX) { sndTickerOffset = maxOffsetX; sndTickerDirection = -1; }
//                     sndLastFrameTime = now;
//                 }

//                 // (Re)crear sprite del ticker
//                 if (!tickerSprite.created() || tickerW != textAreaW || tickerH != cardHeight) {
//                     if (tickerSprite.created()) tickerSprite.deleteSprite();
//                     tickerSprite.createSprite(textAreaW, cardHeight);
//                     tickerW = textAreaW; tickerH = cardHeight;
//                 }

//                 tickerSprite.fillSprite(CARD_COLOR);
//                 tickerSprite.setFreeFont(&FreeSans9pt7b);
//                 tickerSprite.setTextSize(1);
//                 tickerSprite.setTextDatum(TL_DATUM);
//                 tickerSprite.setTextColor(textColor, CARD_COLOR);
//                 tickerSprite.drawString(label, -sndTickerOffset, 0);
//                 // ¡Dentro del viewport!
//                 tickerSprite.pushToSprite(&listSprite, kListLeftX, y);
//             } else {
//                 listSprite.drawString(label, kListLeftX, y);
//             }
//         } else {
//             // No cursor o separador: sin ticker, cortar si no cabe
//             while (listSprite.textWidth(label) > textAreaW && label.length() > 0) {
//                 label.remove(label.length() - 1);
//             }
//             listSprite.drawString(label, kListLeftX, y);
//         }
//     }

//     // Empujar viewport y volcar
//     listSprite.pushToSprite(&uiSprite, 0, kListStartY);

//     // Barra scroll (opcional; si la quieres, dibújala sobre uiSprite aquí)
//     const int scrollBarHeight = kVisibleOptions * stepY - CARD_MARGIN;
//     uiSprite.fillRect(scrollBarX, kListStartY, kScrollBarW, scrollBarHeight, TFT_DARKGREY);

//     if (kTotalOptions > kVisibleOptions) {
//         const float thumbRatio  = float(kVisibleOptions) / float(kTotalOptions);
//         const int   thumbHeight = ((int)(scrollBarHeight * thumbRatio) > 20)
//                                     ? (int)(scrollBarHeight * thumbRatio) : 20;

//         // posRatio de 0..1 según startIndex
//         const int denom = (kTotalOptions - kVisibleOptions) > 0 ? (kTotalOptions - kVisibleOptions) : 1;
//         float posRatio = float(startIndex) / float(denom);

//         // clamp manual (0..1)
//         if (posRatio < 0.0f) posRatio = 0.0f;
//         else if (posRatio > 1.0f) posRatio = 1.0f;

//         const int thumbY = kListStartY + int((scrollBarHeight - thumbHeight) * posRatio);
//         uiSprite.fillRect(scrollBarX, thumbY, kScrollBarW, thumbHeight, TFT_LIGHTGREY);
//     } else {
//         uiSprite.fillRect(scrollBarX, kListStartY, kScrollBarW, scrollBarHeight, TFT_LIGHTGREY);
//     }

//     uiSprite.pushSprite(0, 0);
// }


void drawSoundMenu(int selection)
{
    // ---------- Sincronización SPI/DMA (crítico) ----------
    auto waitTftFlush = [&]() {
#if defined(ESP32) && defined(TFT_SPI_DMA)
        // En TFT_eSPI, si DMA está activo, espera a que termine la transferencia previa
        tft.dmaWait();
#endif
        // Cede CPU (evita pisar buffers en la misma “ventana” temporal)
        yield();
    };

    // Espera a que termine cualquier push anterior antes de tocar sprites/buffers
    waitTftFlush();

    // ---------- Layout ----------
    const int screenW         = tft.width();
    constexpr int kTitleY     = 5;
    constexpr int kListStartY = 30;

    constexpr int kCardRadius = 3;
    constexpr int kHLPadX     = 3;
    constexpr int kHLPadY     = 1;
    constexpr int kRowTopInset= (kHLPadY + 1);
    constexpr int kTickerFrameMs = 50;

    constexpr int kVisibleOptions  = 4;
    constexpr int kTotalOptions    = 11;

    constexpr int kScrollBarW      = 5;
    constexpr int kScrollBarRightP = 3;
    constexpr int kListLeftX       = 9;
    constexpr int kTextAreaPad     = 2;

    static constexpr const char kSep[] = "---------";

    const char* options[kTotalOptions] = {
        getTranslation("MUJER"),
        getTranslation("HOMBRE"),
        kSep,
        getTranslation("CON_NEG"),
        getTranslation("SIN_NEG"),
        kSep,
        getTranslation("VOL_MAXIMO"),
        getTranslation("VOL_NORMAL"),
        getTranslation("VOL_ATENUADO"),
        kSep,
        getTranslation("CONFIRMAR")
    };

    const int cardHeight  = CARD_HEIGHT;
    const int cardMargin  = CARD_MARGIN;
    const int stepY       = cardHeight + cardMargin;

    const int scrollBarX  = screenW - kScrollBarW - kScrollBarRightP;
    const int textAreaW   = scrollBarX - kListLeftX - kTextAreaPad;

    // ---------- Estado persistente: ventana + ticker ----------
    static int startIndex = 0;

    static int lastSelectedIndex  = -1;
    static int sndTickerOffset    = 0;
    static int sndTickerDirection = 1;
    static unsigned long sndLastFrameTime = 0;

    static TFT_eSprite tickerSprite(&tft);
    static int tickerW = 0, tickerH = 0;

    // ---------- Snapshot de selección (fuente para verde) ----------
    const uint8_t vg  = (selectedVoiceGender > 0) ? 1 : 0;
    const bool    neg = negativeResponse;
    const uint8_t vol = (selectedVolume > 2) ? 1 : (uint8_t)selectedVolume;

    auto isSel = [&](int vi)->bool {
        if (vi == 0) return (vg == 0);     // Mujer
        if (vi == 1) return (vg == 1);     // Hombre
        if (vi == 3) return neg;           // Con respuesta negativa
        if (vi == 4) return !neg;          // Sin respuesta negativa
        if (vi == 6) return (vol == 0);    // Vol máximo
        if (vi == 7) return (vol == 1);    // Vol normal
        if (vi == 8) return (vol == 2);    // Vol atenuado
        return false;
    };

    // ---------- Ventana visible (scroll por índices, no por pixeles) ----------
    // Mantenemos "selection" como índice real (0..10) aunque haya separadores.
    if ((selection - startIndex) > (kVisibleOptions - 2) &&
        startIndex < (kTotalOptions - kVisibleOptions)) {
        startIndex++;
    } else if ((selection - startIndex) < 1 && startIndex > 0) {
        startIndex--;
    }

    if (startIndex < 0) startIndex = 0;
    const int lastStart = (kTotalOptions > kVisibleOptions) ? (kTotalOptions - kVisibleOptions) : 0;
    if (startIndex > lastStart) startIndex = lastStart;

    // ---------- Fondo + título ----------
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.drawString(getTranslation("SONIDO"), screenW / 2, kTitleY);

    // Limpieza determinista del área lista (evita “ghosting”)
    const int listHpx = kVisibleOptions * stepY;
    uiSprite.fillRect(0, kListStartY, screenW, listHpx, BACKGROUND_COLOR);

    // Fuente lista
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextDatum(TL_DATUM);
    uiSprite.setTextSize(1);

    // ---------- Dibujar filas visibles ----------
    for (int row = 0; row < kVisibleOptions; ++row) {
        const int vi = startIndex + row;
        if (vi < 0 || vi >= kTotalOptions) continue;

        const int y = kListStartY + kRowTopInset + row * stepY;

        const char* labelC     = options[vi];
        const bool isSeparator = (strcmp(labelC, kSep) == 0);
        const bool isCursor    = (vi == selection);

        const uint16_t bgRow   = (isCursor && !isSeparator) ? CARD_COLOR : BACKGROUND_COLOR;

        // Limpieza determinista de la fila con SU fondo final
        uiSprite.fillRect(0, y - 4, screenW, cardHeight + 8, bgRow);

        // Card cursor (encima del fondo)
        if (isCursor && !isSeparator) {
            uiSprite.fillRoundRect(
                kListLeftX - kHLPadX, (y - kHLPadY),
                textAreaW + (kHLPadX * 2),
                cardHeight + (kHLPadY * 2),
                kCardRadius,
                CARD_COLOR
            );
        }

        const uint16_t fg =
            isSeparator ? TFT_DARKGREY :
            (isSel(vi) ? HIGHLIGHT_COLOR : TEXT_COLOR);

        String label(labelC);

        // Cursor: ticker si no cabe
        if (isCursor && !isSeparator) {
            if (selection != lastSelectedIndex) {
                sndTickerOffset    = 0;
                sndTickerDirection = 1;
                lastSelectedIndex  = selection;
            }

            const int fullW = uiSprite.textWidth(label);
            if (fullW > textAreaW) {
                const unsigned long now = millis();
                if (now - sndLastFrameTime >= kTickerFrameMs) {
                    sndTickerOffset += sndTickerDirection;
                    if (sndTickerOffset < 0) { sndTickerOffset = 0; sndTickerDirection = 1; }
                    const int maxOffsetX = fullW - textAreaW;
                    if (sndTickerOffset > maxOffsetX) { sndTickerOffset = maxOffsetX; sndTickerDirection = -1; }
                    sndLastFrameTime = now;
                }

                bool tickerReady = true;
                if (!tickerSprite.created() || tickerW != textAreaW || tickerH != cardHeight) {
                    if (tickerSprite.created()) tickerSprite.deleteSprite();
                    void* tfb = tickerSprite.createSprite(textAreaW, cardHeight);
                    if (!tfb) tickerReady = false;
                    else { tickerW = textAreaW; tickerH = cardHeight; }
                }

                if (tickerReady) {
                    tickerSprite.fillSprite(bgRow);
                    tickerSprite.setFreeFont(&FreeSans9pt7b);
                    tickerSprite.setTextSize(1);
                    tickerSprite.setTextDatum(TL_DATUM);
                    tickerSprite.setTextColor(fg, bgRow);
                    tickerSprite.drawString(label, -sndTickerOffset, 0);
                    tickerSprite.pushToSprite(&uiSprite, kListLeftX, y);
                } else {
                    // Sin RAM: recorte simple
                    uiSprite.setTextColor(fg, bgRow);
                    while (uiSprite.textWidth(label) > textAreaW && label.length() > 0)
                        label.remove(label.length() - 1);
                    uiSprite.drawString(label, kListLeftX, y);
                }
            } else {
                uiSprite.setTextColor(fg, bgRow);
                uiSprite.drawString(label, kListLeftX, y);
            }
        } else {
            // No cursor: recorte
            uiSprite.setTextColor(fg, bgRow);
            while (!isSeparator && uiSprite.textWidth(label) > textAreaW && label.length() > 0)
                label.remove(label.length() - 1);
            uiSprite.drawString(label, kListLeftX, y);
        }
    }

    // ---------- Scrollbar ----------
    const int scrollBarHeight = kVisibleOptions * stepY - CARD_MARGIN;
    uiSprite.fillRect(scrollBarX, kListStartY, kScrollBarW, scrollBarHeight, TFT_DARKGREY);

    if (kTotalOptions > kVisibleOptions) {
        const float thumbRatio  = float(kVisibleOptions) / float(kTotalOptions);
        const int thumbHeight = (int)(scrollBarHeight * thumbRatio);
        const int clampedThumb = (thumbHeight > 20) ? thumbHeight : 20;

        const int denom = (kTotalOptions - kVisibleOptions) > 0 ? (kTotalOptions - kVisibleOptions) : 1;
        float posRatio = float(startIndex) / float(denom);
        if (posRatio < 0.0f) posRatio = 0.0f;
        if (posRatio > 1.0f) posRatio = 1.0f;

        const int thumbY = kListStartY + int((scrollBarHeight - clampedThumb) * posRatio);
        uiSprite.fillRect(scrollBarX, thumbY, kScrollBarW, clampedThumb, TFT_LIGHTGREY);
    } else {
        uiSprite.fillRect(scrollBarX, kListStartY, kScrollBarW, scrollBarHeight, TFT_LIGHTGREY);
    }

    // ---------- Push final ----------
    uiSprite.pushSprite(0, 0);

    // Espera a que el push termine antes de que el loop vuelva a tocar buffers
    waitTftFlush();
}


// /**
//  * @brief Ticker de texto con rebote para la opción seleccionada del menú de sonido.
//  * Desplaza horizontalmente (vaivén) el texto de la fila seleccionada cuando no cabe
//  * en el ancho visible. 
//  * @param selection Índice lógico de la fila (0..9).
//  */
// void scrollTextTickerBounceSound(int selection)
// {
//     if (!soundMenuActive) return;

//     // ── Layout consistente con drawSoundMenu() ─────────────────────────
//     constexpr int kVisibleOptions      = 4;
//     constexpr int kScrollBarWidth      = 5;
//     constexpr int kScrollBarRightPad   = 3;
//     constexpr int kListLeftX           = 9;   // x de inicio del texto en la lista
//     constexpr int kTextPad             = 2;   // padding interno usado en el cálculo del ancho
//     constexpr int kListTopY            = 30;
//     constexpr int kFrameIntervalMs     = 10;  // ~100 FPS para el ticker
//     constexpr int kTotalOptions        = 10;
//     constexpr int kScrollMargin        = 10;  // margen extra al final del texto para el rebote

//     // Validación de índice
//     if (selection < 0 || selection >= kTotalOptions) return;

//     // Opciones (incluye separadores)
//     static constexpr const char* kSep = "---------";
//     const char* options[kTotalOptions] = {
//         getTranslation("MUJER"),
//         getTranslation("HOMBRE"),
//         kSep,
//         getTranslation("CON_NEG"),
//         getTranslation("SIN_NEG"),
//         kSep,
//         getTranslation("VOL_NORMAL"),
//         getTranslation("VOL_ATENUADO"),
//         kSep,
//         getTranslation("CONFIRMAR")
//     };

//     const char* labelC = options[selection];
//     // No hacer ticker sobre separadores
//     if (strcmp(labelC, kSep) == 0) return;

//     // Texto a medir y mostrar (la fuente ya está configurada por drawSoundMenu)
//     const String text = String(labelC);

//     // Anchos dinámicos según pantalla y barra
//     const int screenW   = tft.width();
//     const int scrollBarX= screenW - kScrollBarWidth - kScrollBarRightPad;
//     const int textAreaW = scrollBarX - kListLeftX - kTextPad;
//     const int textAreaH = CARD_HEIGHT - 2; // igual que en el menú (fondo resaltado)

//     // Si el texto cabe, no se requiere ticker
//     const int fullTextWidth = uiSprite.textWidth(text);
//     if (fullTextWidth <= textAreaW) return;

//     // Ventana vertical (igual criterio que en drawSoundMenu)
//     const int startIndex = max(0, min(selection - (kVisibleOptions / 2),
//                                       kTotalOptions - kVisibleOptions));
//     const int cardIndex  = selection - startIndex;
//     if (cardIndex < 0 || cardIndex >= kVisibleOptions) return;

//     const int textAreaX = kListLeftX;
//     const int textAreaY = kListTopY + cardIndex * (CARD_HEIGHT + CARD_MARGIN);

//     // Color lógico (coincidir con el menú)
//     const bool isLogicallySelected =
//         (selection == 0 && selectedVoiceGender == 0) ||
//         (selection == 1 && selectedVoiceGender == 1) ||
//         (selection == 3 && negativeResponse)         ||
//         (selection == 4 && !negativeResponse)        ||
//         (selection == 6 && selectedVolume == 0)      ||
//         (selection == 7 && selectedVolume == 1);
//     const uint16_t textColor = isLogicallySelected ? HIGHLIGHT_COLOR : TEXT_COLOR;

//     // ── Estado del ticker por opción ───────────────────────────────────
//     static int  pixelOffsets[kTotalOptions]     = {0};
//     static int  scrollDirections[kTotalOptions] = {1,1,1,1,1,1,1,1,1,1};
//     static unsigned long lastFrameTime = 0;

//     const unsigned long now = millis();
//     if (now - lastFrameTime >= kFrameIntervalMs) {
//         int& off = pixelOffsets[selection];
//         int& dir = scrollDirections[selection];

//         off += dir;

//         const int maxOffset = (fullTextWidth - textAreaW) + kScrollMargin;
//         if (off <= 0) {
//             off = 0;
//             dir = 1;
//         } else if (off >= maxOffset) {
//             off = maxOffset;
//             dir = -1;
//         }

//         lastFrameTime = now;
//     }

//     // ── Sprite estático para el ticker (evita crear/borrar por frame) ─
//     static TFT_eSprite ticker(&tft);
//     static int sW = 0, sH = 0;

//     if (!ticker.created() || sW != textAreaW || sH != textAreaH) {
//         if (ticker.created()) ticker.deleteSprite();
//         if (ticker.createSprite(textAreaW, textAreaH) == nullptr) {
//             // Sin memoria para el sprite auxiliar: salir silenciosamente
//             return;
//         }
//         sW = textAreaW; sH = textAreaH;
//     }

//     // Fondo del resaltado y texto
//     ticker.fillSprite(CARD_COLOR);
//     ticker.setFreeFont(&FreeSans9pt7b);
//     ticker.setTextWrap(false);
//     ticker.setTextSize(1);
//     ticker.setTextDatum(TL_DATUM);
//     ticker.setTextColor(textColor, CARD_COLOR);

//     // Desplazar el texto hacia la izquierda (offset positivo)
//     ticker.drawString(text, -pixelOffsets[selection], 0);

//     // Componer sobre el sprite principal (NO directamente a la TFT)
//     ticker.pushToSprite(&uiSprite, textAreaX, textAreaY);
// }

/**
 * @brief Dibuja el menú de “formato/control de sala” con scroll y ticker en la fila activa.
 *
 * Lista 6 acciones (actualizar, borrar, formatear, mostrar ID, restaurar elementos y volver),
 * con ventana de 4 filas visibles, resaltado de la seleccionada y ticker horizontal si la
 * etiqueta no cabe en el ancho disponible. Aplica suavizado al scroll vertical y usa sprites
 * intermedios para evitar parpadeos. Centrado/tamaños calculados dinámicamente según `tft`.
 *
 * @param selection Índice lógico actualmente seleccionado (0..N-1); se clampa a rango válido.
 */



void drawFormatMenu(int selection)
{

    if (!formatSubMenuActive) return;

    /*──────── Constantes/layout ────────*/
    const int screenW         = tft.width();
    constexpr int kTitleY     = 5;
    constexpr int kListStartY = 30;
    constexpr int kCardRadius = 3;
    constexpr int kHLPadX     = 3;
    constexpr int kHLPadY     = 1;
    constexpr int kRowTopInset= (kHLPadY + 1);
    constexpr int kTickerFrameMs = 50;

    const int scrollBarWidth  = 6;
    const int visibleOptions  = 4;
    //const int totalOptions    = 6;
    
    
    const char* options[] = {
        getTranslation("UPDATE_SALA"),
        getTranslation("DELETE_ELEMENT"),
        getTranslation("FORMATEAR"),
        //getTranslation("SHOW_ID"),
        getTranslation("RESTORE_ELEM"),
        getTranslation("ELEMENTOS_ADICIONALES"),
        getTranslation("VOLVER")
    };
    const int totalOptions = sizeof(options) / sizeof(options[0]);
    const int cardHeight  = CARD_HEIGHT;
    const int cardMargin  = CARD_MARGIN;
    const int stepY       = cardHeight + cardMargin;

    const int x              = 9;
    const int scrollBarX     = screenW - scrollBarWidth - 3;
    const int textAreaW      = scrollBarX - x - 2;

    /*──────── Estado scroll persistente ────────*/
    static int scrollOffset       = 0;   // px actuales
    static int targetScrollOffset = 0;   // px destino

    /*──────── Estado del ticker (persistente) ────────*/
    static int lastSelectedIndex  = -1;
    static int fmtTickerOffset    = 0;
    static int fmtTickerDirection = 1;   // 1→derecha, -1→izquierda
    static unsigned long fmtLastFrameTime = 0;
    static TFT_eSprite tickerSprite(&tft);
    static int tickerW = 0, tickerH = 0;

    /*──────── Cabecera ────────*/
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString(String(getTranslation("CONTROL_SALA_MENU")), screenW/2, kTitleY);

    /*──────── Ventana rodante (misma lógica que Modos) ────────*/
    static int startIndex = 0;
    if ((selection - startIndex) > (visibleOptions - 2) &&
        startIndex < (totalOptions - visibleOptions))
        startIndex++;
    else if ((selection - startIndex) < 1 && startIndex > 0)
        startIndex--;

    if (startIndex < 0) startIndex = 0;
    const int lastStart = (totalOptions > visibleOptions) ? (totalOptions - visibleOptions) : 0;
    if (startIndex > lastStart) startIndex = lastStart;

    /*──────── Resolver SCROLL ANTES de dibujar ────────*/
    targetScrollOffset = startIndex * stepY;
    const int maxScroll = lastStart * stepY;

    if (targetScrollOffset < 0)         targetScrollOffset = 0;
    if (targetScrollOffset > maxScroll) targetScrollOffset = maxScroll;

    int delta = targetScrollOffset - scrollOffset;
    if (delta >= -1 && delta <= 1) scrollOffset = targetScrollOffset;
    else                           scrollOffset += (delta / 4);

    /*──────── Viewport con clipping ────────*/
    static TFT_eSprite listSprite(&tft);
    static int listW = 0, listH = 0;
    const int wantedListW = screenW;
    const int wantedListH = visibleOptions * stepY;

    if (!listSprite.created() || listW != wantedListW || listH != wantedListH) {
        if (listSprite.created()) listSprite.deleteSprite();
        listSprite.createSprite(wantedListW, wantedListH);
        listW = wantedListW; listH = wantedListH;
    }

    listSprite.fillSprite(BACKGROUND_COLOR);
    listSprite.setFreeFont(&FreeSans9pt7b);
    listSprite.setTextSize(1);
    listSprite.setTextDatum(TL_DATUM);
    listSprite.setTextColor(TEXT_COLOR);

    const int listYOffset = -(scrollOffset - startIndex * stepY);
    const int firstVI     = startIndex - 1;
    const int lastVI      = startIndex + visibleOptions;

    for (int vi = firstVI; vi <= lastVI; ++vi)
    {
        if (vi < 0 || vi >= totalOptions) continue;

        const int y = kRowTopInset + listYOffset + (vi - startIndex) * stepY;
        if (y <= -cardHeight + kHLPadY || y >= listH - kHLPadY) continue;

        const bool isSelected = (vi == selection);

        // Fondo resaltado con clamp al viewport
        if (isSelected) {
            int rectY = y - kHLPadY;
            int rectH = cardHeight + (kHLPadY * 2);
            if (rectY < 0)            { rectH += rectY; rectY = 0; }
            if (rectY + rectH > listH){ rectH = listH - rectY; }
            if (rectH > 0) {
                listSprite.fillRoundRect(
                    x - kHLPadX, rectY,
                    textAreaW + (kHLPadX * 2),
                    rectH, kCardRadius,
                    CARD_COLOR
                );
            }
        }

        // Texto / ticker (como en Modos)
        String label = String(options[vi]);

        if (isSelected) {
            // Reset del ticker al cambiar de fila
            if (selection != lastSelectedIndex) {
                fmtTickerOffset    = 0;
                fmtTickerDirection = 1;
                lastSelectedIndex  = selection;
            }

            const int fullW = listSprite.textWidth(label);
            if (fullW > textAreaW) {
                const unsigned long now = millis();
                if (now - fmtLastFrameTime >= kTickerFrameMs) {
                    fmtTickerOffset += fmtTickerDirection;
                    if (fmtTickerOffset < 0) { fmtTickerOffset = 0; fmtTickerDirection = 1; }
                    const int maxOffsetX = fullW - textAreaW;
                    if (fmtTickerOffset > maxOffsetX) { fmtTickerOffset = maxOffsetX; fmtTickerDirection = -1; }
                    fmtLastFrameTime = now;
                }

                // (Re)crear sprite del ticker si cambia el tamaño
                if (!tickerSprite.created() || tickerW != textAreaW || tickerH != cardHeight) {
                    if (tickerSprite.created()) tickerSprite.deleteSprite();
                    tickerSprite.createSprite(textAreaW, cardHeight);
                    tickerW = textAreaW; tickerH = cardHeight;
                }

                tickerSprite.fillSprite(CARD_COLOR);
                tickerSprite.setFreeFont(&FreeSans9pt7b);
                tickerSprite.setTextSize(1);
                tickerSprite.setTextDatum(TL_DATUM);
                tickerSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
                tickerSprite.drawString(label, -fmtTickerOffset, 0);
                // ¡Dentro del viewport!
                tickerSprite.pushToSprite(&listSprite, x, y);
            } else {
                listSprite.drawString(label, x, y);
            }
        } else {
            // No seleccionada: dibuja sin ticker
            listSprite.drawString(label, x, y);
        }
    }

    listSprite.pushToSprite(&uiSprite, 0, kListStartY);

    uiSprite.pushSprite(0, 0);
}

bool forceDrawDeleteElementMenu = false;

/**
 * @brief Dibuja el menú de borrado de elementos con scroll suave y ticker en la fila activa.
 *
 * Lista los ficheros eliminables (vector `deletableElementFiles`) en una ventana de
 * 4 filas visibles, resalta la fila bajo el cursor y aplica ticker horizontal
 * (ida/vuelta) cuando la etiqueta no cabe. Usa sprites intermedios (viewport y ticker)
 * para evitar parpadeos y realiza un volcado final a la pantalla.
 *
 * @param selection Índice lógico actualmente seleccionado. Se clampa a [0..N-1].
 */
void drawDeleteElementMenu(int selection)
{
    /*──────── Constantes/layout ────────*/
    const int screenW         = tft.width();
    constexpr int kTitleY     = 5;
    constexpr int kListStartY = 30;
    constexpr int kCardRadius = 3;
    constexpr int kHLPadX     = 3;
    constexpr int kHLPadY     = 1;
    constexpr int kRowTopInset= (kHLPadY + 1);
    constexpr int kTickerFrameMs = 50;

    constexpr int kVisibleOptions  = 4;
    constexpr int kScrollBarWidth  = 5;
    constexpr int kScrollBarRightP = 3;
    constexpr int kTextAreaPad     = 2;

    const int x               = 9;
    const int scrollBarX      = screenW - kScrollBarWidth - kScrollBarRightP;
    const int textAreaW       = scrollBarX - x - kTextAreaPad;

    const int cardHeight = CARD_HEIGHT;
    const int cardMargin = CARD_MARGIN;
    const int stepY      = cardHeight + cardMargin;

    const int totalOptions = static_cast<int>(deletableElementFiles.size());

    /*──────── Fondo + título ────────*/
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString(String(getTranslation("DELETE_ELEMENT_MENU")), screenW/2, kTitleY);

    // Sin elementos -> mensaje y salir
    if (totalOptions == 0) {
        uiSprite.setFreeFont(&FreeSans9pt7b);
        uiSprite.setTextDatum(TC_DATUM);
        uiSprite.setTextColor(HIGHLIGHT_COLOR);
        uiSprite.setTextSize(1);
        uiSprite.drawString("No hay elementos", screenW/2, 64);
        uiSprite.pushSprite(0, 0);
        return;
    }

    /*──────── Normalización de selección ────────*/
    int sel = selection;
    if (sel < 0) sel = 0;
    if (sel >= totalOptions) sel = totalOptions - 1;

    /*──────── Ventana rodante (misma lógica que Modos) ────────*/
    static int startIndex = 0; // índice de la primera fila visible
    if ((sel - startIndex) > (kVisibleOptions - 2) &&
        startIndex < (totalOptions - kVisibleOptions))
        startIndex++;
    else if ((sel - startIndex) < 1 && startIndex > 0)
        startIndex--;

    if (startIndex < 0) startIndex = 0;
    const int lastStart = (totalOptions > kVisibleOptions) ? (totalOptions - kVisibleOptions) : 0;
    if (startIndex > lastStart) startIndex = lastStart;

    /*──────── Scroll suave (resolver ANTES de dibujar) ────────*/
    static int scrollOffset       = 0; // px actuales
    static int targetScrollOffset = 0; // px destino

    targetScrollOffset = startIndex * stepY;
    const int maxScroll = lastStart * stepY;

    if (targetScrollOffset < 0)         targetScrollOffset = 0;
    if (targetScrollOffset > maxScroll) targetScrollOffset = maxScroll;

    {
        const int delta = targetScrollOffset - scrollOffset;
        if (delta >= -1 && delta <= 1)  scrollOffset = targetScrollOffset; // snap
        else                            scrollOffset += (delta / 4);       // easing
    }

    /*──────── Ticker horizontal (estado persistente) ────────*/
    static int lastSelectedIndex  = -1;
    static int delTickerOffset    = 0;
    static int delTickerDirection = 1;  // 1→derecha, -1→izquierda
    static unsigned long delLastFrameTime = 0;

    // Sprite estático para ticker (reutilizable)
    static TFT_eSprite tickerSprite(&tft);
    static int tickerW = 0, tickerH = 0;

    /*──────── Viewport con clipping ────────*/
    static TFT_eSprite listSprite(&tft);
    static int listW = 0, listH = 0;
    const int wantedListW = screenW;
    const int wantedListH = kVisibleOptions * stepY;

    bool listSpriteOK = true;
    if (!listSprite.created() || listW != wantedListW || listH != wantedListH) {
        if (listSprite.created()) listSprite.deleteSprite();
        if (listSprite.createSprite(wantedListW, wantedListH) == nullptr) {
            listSpriteOK = false; // Fallback: dibujar directo en uiSprite
        } else {
            listW = wantedListW; listH = wantedListH;
        }
    }

    TFT_eSprite& dst = listSpriteOK ? listSprite : uiSprite;

    // Preparar destino
    dst.fillSprite(BACKGROUND_COLOR);
    dst.setFreeFont(&FreeSans9pt7b);
    dst.setTextSize(1);
    dst.setTextDatum(TL_DATUM);
    dst.setTextColor(TEXT_COLOR);

    const int listYOffset = -(scrollOffset - startIndex * stepY);
    const int firstVI     = startIndex - 1;
    const int lastVI      = startIndex + kVisibleOptions;

    // Lambda para una fila (válida para listSprite o uiSprite)
    auto drawRow = [&](int vi, int y, bool isCursor) {
        // Clamp de resaltado al viewport sólo si dst es listSprite
        if (isCursor) {
            int rectY = y - kHLPadY;
            int rectH = cardHeight + (kHLPadY * 2);
            if (listSpriteOK) {
                const int dstH = dst.height();
                if (rectY < 0)            { rectH += rectY; rectY = 0; }
                if (rectY + rectH > dstH) { rectH = dstH - rectY; }
            }
            if (rectH > 0) {
                dst.fillRoundRect(
                    x - kHLPadX, rectY,
                    textAreaW + (kHLPadX * 2),
                    rectH, kCardRadius,
                    CARD_COLOR
                );
            }
        }

        // Etiqueta base (nombre del fichero)
        String label = deletableElementFiles[vi];

        // Si está bajo cursor, activar ticker si no cabe
        if (isCursor) {
            if (sel != lastSelectedIndex) {
                delTickerOffset    = 0;
                delTickerDirection = 1;
                delLastFrameTime   = 0;
                lastSelectedIndex  = sel;
            }

            const int fullW = dst.textWidth(label);
            if (fullW > textAreaW) {
                const unsigned long now = millis();
                if (delLastFrameTime == 0 || (now - delLastFrameTime) >= kTickerFrameMs) {
                    delTickerOffset += delTickerDirection;
                    if (delTickerOffset < 0) { delTickerOffset = 0; delTickerDirection = 1; }
                    const int maxOffsetX = fullW - textAreaW;
                    if (delTickerOffset > maxOffsetX) { delTickerOffset = maxOffsetX; delTickerDirection = -1; }
                    delLastFrameTime = now;
                }

                // (Re)crear sprite del ticker si cambian dimensiones
                if (!tickerSprite.created() || tickerW != textAreaW || tickerH != cardHeight) {
                    if (tickerSprite.created()) tickerSprite.deleteSprite();
                    if (tickerSprite.createSprite(textAreaW, cardHeight) == nullptr) {
                        // Sin RAM para ticker → escribir texto estático
                        dst.drawString(label, x, y);
                        return;
                    }
                    tickerW = textAreaW; tickerH = cardHeight;
                }

                tickerSprite.fillSprite(CARD_COLOR);
                tickerSprite.setFreeFont(&FreeSans9pt7b);
                tickerSprite.setTextSize(1);
                tickerSprite.setTextDatum(TL_DATUM);
                tickerSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
                tickerSprite.drawString(label, -delTickerOffset, 0);

                if (listSpriteOK) {
                    tickerSprite.pushToSprite(&dst, x, y);
                } else {
                    // Fallback: destino es uiSprite; dibuja directamente en pantalla
                    tickerSprite.pushToSprite(&dst, x, y);
                }
                return;
            }
        } else {
            // No seleccionado: recorta para evitar desbordes
            while (dst.textWidth(label) > textAreaW && label.length() > 0) {
                label.remove(label.length() - 1);
            }
        }

        // Dibujo sin ticker
        dst.drawString(label, x, y);
    };

    // Iterar filas visibles (con margen arriba/abajo)
    for (int vi = firstVI; vi <= lastVI; ++vi)
    {
        if (vi < 0 || vi >= totalOptions) continue;

        // Y relativo al destino
        int y = kRowTopInset + listYOffset + (vi - startIndex) * stepY;

        // Culling con margen si usamos sprite de lista
        if (listSpriteOK) {
            if (y <= -cardHeight + kHLPadY || y >= listH - kHLPadY) continue;
        }

        const bool isCursor = (vi == sel);
        drawRow(vi, y, isCursor);
    }

    // Componer la lista debajo del título y volcar a pantalla
    if (listSpriteOK) {
        listSprite.pushToSprite(&uiSprite, 0, kListStartY);
    }
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Dibuja el diálogo de confirmación de borrado con ticker del nombre y botones Sí/No.
 *
 * Muestra el título del menú, el nombre del fichero centrado bajo el título (con ticker
 * horizontal ping-pong si no cabe en el área) y dos botones (“Sí, borrar” / “No, cancelar”),
 * resaltando el botón actualmente seleccionado.
 *
 * @param fileName Nombre del fichero a borrar (etiqueta mostrada tal cual).
 */
void drawConfirmDelete(const String& fileName)
{
    // ── Constantes de layout ───────────────────────────────────────────
    const int screenW         = tft.width();
    constexpr int kTitleY     = 5;
    constexpr int kCardRadius = 4;

    // Viewport del nombre (centrado bajo el título)
    constexpr int kNameAreaW  = 120;
    constexpr int kNameAreaH  = 18;
    const int     kNameAreaX  = (screenW - kNameAreaW) / 2;
    const int     kNameAreaY  = 35;   // posición vertical fija

    // Botones Sí/No
    const char* opciones[] = { getTranslation("YES_DELETE"), getTranslation("NO_DELETE") };
    constexpr int kButtons = 2;
    constexpr int xBtn     = 20;
    constexpr int yBase    = 64;
    constexpr int boxW     = 88;
    constexpr int boxH     = 22;
    constexpr int spacing  = 28;

    // ── Estado del ticker del nombre ───────────────────────────────────
    constexpr int kTickerFrameMs = 50;
    static String lastFileShown;
    static int nameTickerOffset    = 0;
    static int nameTickerDirection = 1;   // 1→derecha, -1→izquierda
    static unsigned long nameLastFrame = 0;
    static TFT_eSprite nameSprite(&tft);
    static int nameW = 0, nameH = 0;

    // ── Fondo + título ─────────────────────────────────────────────────
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(1);
    uiSprite.drawString(getTranslation("DELETE_ELEMENT_MENU"), screenW / 2, kTitleY);

    // ── Medición con la fuente real de la etiqueta del nombre ─────────
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextWrap(false);
    uiSprite.setTextSize(1);
    const int fullW = uiSprite.textWidth(fileName);

    // (Re)crear sprite del nombre si cambian dimensiones
    if (!nameSprite.created() || nameW != kNameAreaW || nameH != kNameAreaH) {
        if (nameSprite.created()) nameSprite.deleteSprite();
        if (nameSprite.createSprite(kNameAreaW, kNameAreaH) == nullptr) {
            // Fallback: no hay RAM para el sprite → dibujar directamente el nombre (sin ticker)
            uiSprite.setTextDatum(TC_DATUM);
            uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
            const int cx = kNameAreaX + kNameAreaW / 2;
            const int cy = kNameAreaY + kNameAreaH / 2;
            uiSprite.drawString(fileName, cx, cy);
            // Continuar con los botones igualmente
        } else {
            nameW = kNameAreaW; nameH = kNameAreaH;
        }
    }

    // Si el sprite se creó correctamente, procesar ticker/centrado en él
    if (nameSprite.created()) {
        // Reset del ticker si cambia el fichero
        if (fileName != lastFileShown) {
            nameTickerOffset    = 0;
            nameTickerDirection = 1;
            nameLastFrame       = 0;
            lastFileShown       = fileName;
        }

        nameSprite.fillSprite(BACKGROUND_COLOR);
        nameSprite.setFreeFont(&FreeSans9pt7b);
        nameSprite.setTextWrap(false);
        nameSprite.setTextSize(1);
        nameSprite.setTextDatum(ML_DATUM); // izquierda + centro vertical
        nameSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);

        if (fullW <= kNameAreaW) {
            // Centrado visual cuando cabe
            const int xCentered = (kNameAreaW - fullW) / 2;
            nameSprite.drawString(fileName, xCentered, kNameAreaH / 2);
        } else {
            // Ticker “ping-pong”
            const unsigned long now = millis();
            if (nameLastFrame == 0 || (now - nameLastFrame) >= kTickerFrameMs) {
                nameTickerOffset += nameTickerDirection;
                if (nameTickerOffset < 0) {
                    nameTickerOffset = 0; nameTickerDirection = 1;
                }
                const int maxOffset = fullW - kNameAreaW;
                if (nameTickerOffset > maxOffset) {
                    nameTickerOffset = maxOffset; nameTickerDirection = -1;
                }
                nameLastFrame = now;
            }
            nameSprite.drawString(fileName, -nameTickerOffset, kNameAreaH / 2);
        }

        nameSprite.pushToSprite(&uiSprite, kNameAreaX, kNameAreaY);
    }

    // ── Botones con texto centrado ─────────────────────────────────────
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextSize(1);
    uiSprite.setTextDatum(MC_DATUM);

    // Clamp defensivo del índice seleccionado (no cambia la lógica externa)
    int sel = confirmSelection;
    if (sel < 0) sel = 0;
    if (sel >= kButtons) sel = kButtons - 1;

    for (int i = 0; i < kButtons; ++i) {
        const int rectX  = xBtn - 4;
        const int rectY  = (yBase + i * spacing) - 2;
        const int rectW  = boxW + 8;
        const int rectH  = boxH + 4;

        if (i == sel) {
            uiSprite.fillRoundRect(rectX, rectY, rectW, rectH, kCardRadius, CARD_COLOR);
            uiSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
        } else {
            uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
        }

        const int cx = rectX + rectW / 2;
        const int cy = rectY + rectH / 2;
        uiSprite.drawString(opciones[i], cx, cy);
    }

    // ── Volcado final ──────────────────────────────────────────────────
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Muestra un aviso de batería crítica con icono centrado.
 *
 * Rellena el fondo en negro, dibuja el texto de aviso en rojo (centrado en X,
 * a Y fija) y pinta un icono de batería centrado bajo el texto. Finalmente
 * vuelca el sprite a la pantalla.
 */
void showCriticalBatteryMessage()
{
    // ── Constantes de layout (mismos valores que el original, sin “números mágicos”) ──
    constexpr int kTextY   = 10;  // Y del mensaje
    constexpr int kIconW   = 32;  // Ancho icono batería
    constexpr int kIconH   = 61;  // Alto  icono batería
    constexpr int kIconY   = 40;  // Y superior del icono

    // ── Fondo ──
    uiSprite.fillSprite(TFT_BLACK);

    // ── Texto “Cargar bateria” centrado ──
    uiSprite.setTextDatum(MC_DATUM);           // centro horizontal, medio vertical
    uiSprite.setTextColor(TFT_RED, TFT_BLACK); // rojo sobre negro
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextSize(1);

    const int centerX = tft.width() / 2;       // centro dinámico según ancho real
    uiSprite.drawString(getTranslation("CARGAR_BATERIA"), centerX, kTextY);

    // ── Icono de batería centrado bajo el texto ──
    const int iconX = (tft.width() - kIconW) / 2;
    uiSprite.pushImage(iconX, kIconY, kIconW, kIconH,
                       reinterpret_cast<const uint16_t*>(vbat32x61_0));

    // ── Volcado a pantalla ──
    uiSprite.pushSprite(0, 0);
}


const int vbatallArray_LEN = 5;
const uint16_t* vbatallArray[4] = {
	vbat30x15_horizontal_10_25_1,
	vbat30x15_horizontal_10_25_2,
	vbat30x15_horizontal_25_66,
	vbat30x15_horizontal_66_100
};

const uint16_t* vbatallArrayCharging[5] = {
    vbat30x15_cargando_horizontal_10_25_1,
    vbat30x15_cargando_horizontal_10_25_2,
    vbat30x15_cargando_horizontal_25_66,
    vbat30x15_cargando_horizontal_66_100, // <— usa el nombre con rayo
    vbat30x15_horizontal_cargado
                                // no se usa en mini; lo dejamos igual
};

unsigned long lastBatteryToggleTime = 0;
bool batteryToggleState = false;
extern float batteryVoltagePercent;

/**
 * @brief Dibuja el icono de batería mini en la esquina superior derecha.
 *
 * Muestra un icono de 30×15 px indicando el nivel de batería:
 * - **Cargando**: si `percentage` supera el *sentinel* (umbral especial).
 * - **<25%**: parpadeo entre 0/1 barras cada 500 ms.
 * - **25–66%**: 2 barras.
 * - **≥66%**: 3 barras.
 *
 * @param percentage Porcentaje visual de batería (0..100). Si es mayor que el
 *        umbral de *sentinel*, se muestra el icono de “cargando”.
 */

// void drawBatteryIconMini(float /*percentage_ignored*/) {
//     // ── Constantes ─────────────────────────────────────────────────
//     constexpr int   kIconW                 = 30;
//     constexpr int   kIconH                 = 15;
//     constexpr int   kIconTopY              = 2;
//     constexpr float kLowThresholdPct       = 25.0f;  // 10–25% → toggle
//     constexpr float kMedThresholdPct       = 66.0f;  // 25–66%
//     constexpr float kChargedWhileCharging  = 98.0f;  // >98% y CARGANDO → “cargado”
//     constexpr unsigned long kBlinkMs       = 500UL;
//     constexpr float BAT_VIS_CHARGING       = 200.0f;

//     const int x = tft.width() - kIconW;
//     const int y = kIconTopY;

//     const bool isChargingVisual = (batteryVisualPercentage == BAT_VIS_CHARGING);

//     // Usamos el % VISUAL basado en tensión para los iconos
//     float p = batteryVoltagePercent;               // 0..100 según Vbat/4.22
//     if (p < 0.0f)   p = 0.0f;
//     if (p > 100.0f) p = 100.0f;

//     const uint16_t* icon = nullptr;

//     if (isChargingVisual) {
//         // ── CARGANDO ────────────────────────────────────────────────
//         if (p >= kChargedWhileCharging) {
//             // Sólo cuando está cargando y supera 98%
//             icon = vbat30x15_horizontal_cargado;
//         } else if (p < kLowThresholdPct) {
//             const unsigned long now = millis();
//             if ((now - lastBatteryToggleTime) > kBlinkMs) {
//                 batteryToggleState    = !batteryToggleState;
//                 lastBatteryToggleTime = now;
//             }
//             icon = batteryToggleState
//                 ? vbat30x15_cargando_horizontal_10_25_2
//                 : vbat30x15_cargando_horizontal_10_25_1;
//         } else if (p < kMedThresholdPct) {
//             icon = vbat30x15_cargando_horizontal_25_66;
//         } else {
//             icon = vbat30x15_cargando_horizontal_66_100; // 66–98%
//         }
//     } else {
//         // ── NO CARGANDO ─────────────────────────────────────────────
//         if (p < kLowThresholdPct) {
//             const unsigned long now = millis();
//             if ((now - lastBatteryToggleTime) > kBlinkMs) {
//                 batteryToggleState    = !batteryToggleState;
//                 lastBatteryToggleTime = now;
//             }
//             icon = batteryToggleState
//                 ? vbat30x15_horizontal_10_25_2
//                 : vbat30x15_horizontal_10_25_1;
//         } else if (p < kMedThresholdPct) {
//             icon = vbat30x15_horizontal_25_66;
//         } else {
//             icon = vbat30x15_horizontal_66_100;         // 66–100%
//         }
//     }

//     if (icon == nullptr) icon = vbat30x15_horizontal_10_25_1; // fallback
//     uiSprite.pushImage(x, y, kIconW, kIconH, icon);
// }

void drawBatteryIconMini(float /*percentage_ignored*/) {
    // ── Constantes ─────────────────────────────────────────────────
    constexpr int   kIconW                 = 30;
    constexpr int   kIconH                 = 15;
    constexpr int   kIconTopY              = 2;
    constexpr float kLowThresholdPct       = 25.0f;  // 10–25% → toggle
    constexpr float kMedThresholdPct       = 66.0f;  // 25–66%
    constexpr float kChargedWhileCharging  = 99.0f;  // vVis ≥98% → “cargado” SI está cargando
    constexpr unsigned long kBlinkMs       = 500UL;
    constexpr float BAT_VIS_CHARGING       = 200.0f;

    const int x = tft.width() - kIconW;
    const int y = kIconTopY;

    const bool isChargingVisual = (batteryVisualPercentage == BAT_VIS_CHARGING);

    // pVis: % por tensión absoluta (V/4.20*100)
    float pVis = batteryVoltagePercent;
    if (pVis < 0.0f)   pVis = 0.0f;
    if (pVis > 100.0f) pVis = 100.0f;

    // pIcon: % para icono con suelo 3.00V (ya calculado en updateBatteryStatus)
    float pIcon = batteryPercentage;
    if (pIcon < 0.0f)   pIcon = 0.0f;
    if (pIcon > 100.0f) pIcon = 100.0f;

    const uint16_t* icon = nullptr;

    if (isChargingVisual) {
        // ── CARGANDO ────────────────────────────────────────────────
        if (pVis >= kChargedWhileCharging) {
            // “cargado” SOLO si (visual cargando) y vVis ≥98%
            icon = vbat30x15_horizontal_cargado;
        } else if (pIcon < kLowThresholdPct) {
            const unsigned long now = millis();
            if ((now - lastBatteryToggleTime) > kBlinkMs) {
                batteryToggleState    = !batteryToggleState;
                lastBatteryToggleTime = now;
            }
            icon = batteryToggleState
                ? vbat30x15_cargando_horizontal_10_25_2
                : vbat30x15_cargando_horizontal_10_25_1;
        } else if (pIcon < kMedThresholdPct) {
            icon = vbat30x15_cargando_horizontal_25_66;
        } else {
            icon = vbat30x15_cargando_horizontal_66_100; // 66–98%
        }
    } else {
        // ── NO CARGANDO ─────────────────────────────────────────────
        if (pIcon < kLowThresholdPct) {
            const unsigned long now = millis();
            if ((now - lastBatteryToggleTime) > kBlinkMs) {
                batteryToggleState    = !batteryToggleState;
                lastBatteryToggleTime = now;
            }
            icon = batteryToggleState
                ? vbat30x15_horizontal_10_25_2
                : vbat30x15_horizontal_10_25_1;
        } else if (pIcon < kMedThresholdPct) {
            icon = vbat30x15_horizontal_25_66;
        } else {
            icon = vbat30x15_horizontal_66_100;         // 66–100%
        }
    }

    if (icon == nullptr) icon = vbat30x15_horizontal_10_25_1; // fallback
    uiSprite.pushImage(x, y, kIconW, kIconH, icon);
}


/**
 * @brief Dibuja el menú de actividades cognitivas con opción “Salir”.
 *
 * Muestra dos líneas de título centradas (“ACTIVIDADES” y “COGNITIVAS”) y un
 * botón centrado con la etiqueta “SALIR”. Usa `uiSprite` como *backbuffer* y
 * vuelca el resultado a la pantalla.
 */
void drawCognitiveMenu()
{
    // ── Constantes de layout (mantienen el diseño original) ──
    constexpr int kTitleY1       = 20;  // Y de la primera línea del título
    constexpr int kTitleY2       = 40;  // Y de la segunda línea del título
    constexpr int kExitY         = 80;  // Y del texto “SALIR”
    constexpr int kBtnY          = 70;  // Y del rectángulo del botón
    constexpr int kBtnW          = 60;  // ancho del botón
    constexpr int kBtnH          = 20;  // alto del botón
    constexpr int kBtnRadius     = 4;   // radio de la esquina
    constexpr int kTextSize      = 1;

    // Centro X dinámico (equivalente a 64 en pantallas de 128 px)
    const int centerX = tft.width() / 2;

    // ── Fondo ──
    uiSprite.fillSprite(BACKGROUND_COLOR);

    // ── Títulos ──
    uiSprite.setTextDatum(MC_DATUM);
    uiSprite.setFreeFont(&FreeSansBold9pt7b);
    uiSprite.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    uiSprite.setTextSize(kTextSize);

    uiSprite.drawString(getTranslation("ACTIVIDADES"), centerX, kTitleY1);
    uiSprite.drawString(getTranslation("COGNITIVAS"),  centerX, kTitleY2);

    // ── Botón SALIR ──
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    uiSprite.setTextSize(kTextSize);

    // Rectángulo centrado (en el original: x=34 con ancho 60 → centrado a 64)
    const int btnX = centerX - (kBtnW / 2);
    uiSprite.drawString(getTranslation("SALIR"), centerX, kExitY);
    uiSprite.drawRoundRect(btnX, kBtnY, kBtnW, kBtnH, kBtnRadius, TFT_LIGHTGREY);

    // ── Volcado a pantalla ──
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Dibuja el diálogo de confirmación para restaurar la sala.
 *
 * Muestra un título y dos opciones ("Si" / "No") en vertical. Resalta con un
 * rectángulo redondeado la opción indicada por `selection`. No modifica estado
 * global ni realiza navegación; solo pinta y hace push del sprite.
 *
 * @param selection Índice de la opción seleccionada: 0 = "Si", 1 = "No".
 *                  Valores fuera de rango se clampean a [0..1] para el dibujo.
 */

void drawConfirmRestoreMenu(int selection)
{
    // ── Constantes de layout (mismos valores visuales) ──
    constexpr int kTitleY      = 10;  // Y del título
    constexpr int kBtnTopY     = 40;  // Y de la primera opción
    constexpr int kBtnSpacing  = 40;  // separación vertical entre opciones
    constexpr int kBtnW        = 88;  // ancho del resaltado
    constexpr int kBtnH        = 30;  // alto del resaltado
    constexpr int kBtnRadius   = 5;   // radio de esquinas
    constexpr int kBtnPadX     = 4;   // padding horizontal extra del resaltado
    constexpr int kBtnPadY     = 5;   // padding vertical extra del resaltado
    constexpr int kTextSize    = 1;

    // Centro X dinámico (equivalente a 64 si ancho = 128)
    const int centerX = tft.width() / 2;

    // Clamp defensivo del índice para evitar accesos fuera de rango
    int sel = selection;
    if (sel < 0) sel = 0;
    if (sel > 1) sel = 1;

    // ── Fondo y título ──
    uiSprite.fillSprite(BACKGROUND_COLOR);

    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(kTextSize);
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.drawString(getTranslation("RESTORE_ROOM"), centerX, kTitleY);

    // ── Opciones ──
    const char* options[2] = {getTranslation("YES_DELETE"), getTranslation("NO_DELETE")};

    for (int i = 0; i < 2; ++i) {
        const int yText = kBtnTopY + i * kBtnSpacing;

        // Si está seleccionada, dibuja fondo resaltado centrado
        if (i == sel) {
            const int rectX = centerX - (kBtnW / 2) - (kBtnPadX - 0); // mantiene ancho visual
            const int rectY = yText - kBtnPadY;
            uiSprite.fillRoundRect(rectX, rectY, kBtnW + (kBtnPadX * 2), kBtnH,
                                   kBtnRadius, CARD_COLOR);
            uiSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
        } else {
            uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
        }

        uiSprite.setTextDatum(TC_DATUM);
        uiSprite.drawString(options[i], centerX, yText);
    }

    // ── Volcado a pantalla ──
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Dibuja el diálogo de confirmación para restaurar un elemento.
 *
 * Muestra un título (“¿Restaurar?”) centrado y dos opciones (“Si”/“No”) en vertical.
 * Resalta la opción indicada por `selection` con un rectángulo redondeado. No modifica
 * estado global ni realiza navegación; solo pinta y hace push del sprite.
 *
 * @param selection Índice de la opción seleccionada: 0 = "Si", 1 = "No".
 *                  Valores fuera de rango se clampean a [0..1] para el dibujo.
 */
void drawConfirmRestoreElementMenu(int selection)
{
    // ── Constantes de layout (mismos valores visuales) ──
    constexpr int kTitleY      = 10;  // Y del título
    constexpr int kOptTopY     = 40;  // Y de la primera opción
    constexpr int kOptSpacing  = 40;  // separación vertical entre opciones
    constexpr int kHLRectX     = 20;  // X del resaltado
    constexpr int kHLRectW     = 88;  // ancho del resaltado
    constexpr int kHLRectH     = 30;  // alto del resaltado
    constexpr int kHLRectPadY  = 5;   // padding vertical del resaltado
    constexpr int kHLRadius    = 5;   // radio esquinas
    constexpr int kTextSize    = 1;

    // Centro X dinámico (64 si el ancho es 128)
    const int centerX = tft.width() / 2;

    // Clamp defensivo de la selección
    int sel = selection;
    if (sel < 0) sel = 0;
    if (sel > 1) sel = 1;

    // ── Fondo ──
    uiSprite.fillSprite(BACKGROUND_COLOR);

    // ── Título (traducido) ──
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextDatum(TC_DATUM);
    uiSprite.setTextSize(kTextSize);
    uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
    uiSprite.drawString(getTranslation("RESTORE"), centerX, kTitleY);

    // ── Opciones ──
    const char* options[2] = {getTranslation("YES_DELETE"), getTranslation("NO_DELETE")};

    for (int i = 0; i < 2; ++i) {
        const int yText = kOptTopY + i * kOptSpacing;

        // Resaltado de la opción seleccionada
        if (i == sel) {
            uiSprite.fillRoundRect(
                kHLRectX, yText - kHLRectPadY,
                kHLRectW, kHLRectH,
                kHLRadius, CARD_COLOR
            );
            uiSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
        } else {
            uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
        }

        uiSprite.setTextDatum(TC_DATUM);
        uiSprite.drawString(options[i], centerX, yText);
    }

    // ── Volcado a pantalla ──
    uiSprite.pushSprite(0, 0);
}

/**
 * @brief Dibuja texto con ajuste de líneas (wrap por palabras) centrado en X.
 *
 * Parte el texto en varias líneas sin superar `maxWidth`, dibujando cada línea
 * centrada en X (`MC_DATUM`) a partir de `yInicio`. Limpia previamente un área
 * rectangular fija y restaura el datum y la fuente al finalizar.
 *
 * @param tft       Referencia al objeto `TFT_eSPI`.
 * @param texto     Cadena C (UTF-8/ASCII) con el contenido a mostrar. Si es `nullptr`, no dibuja.
 * @param xCentro   Coordenada X del centro para cada línea (px).
 * @param yInicio   Coordenada Y de la primera línea (px). Se incrementa en `fontHeight()` por línea.
 * @param maxWidth  Ancho máximo permitido para cada línea (px). Debe ser > 0.
 */
void mostrarTextoAjustado(TFT_eSPI& tft,
                          const char* texto,
                          uint16_t xCentro,
                          uint16_t yInicio,
                          uint16_t maxWidth)
{
    // ── Constantes (eliminan “números mágicos” manteniendo layout original) ──
    constexpr uint16_t kEraseX           = 5;
    constexpr uint16_t kEraseWidth       = 118;
    constexpr uint16_t kMaxTextHeight    = 55;
    constexpr uint8_t  kFontIndex        = 2;      // fuente pequeña integrada de TFT_eSPI
    constexpr size_t   kBufCap           = 128;    // límite de copia por línea (seguridad)

    // Entrada nula → no dibujar (defensivo sin cambiar la semántica normal)
    if (texto == nullptr) return;

    // Guardar estado previo (nota: `textfont` es miembro interno en TFT_eSPI)
    uint8_t prevFont  = tft.textfont;
    uint8_t prevDatum = tft.getTextDatum();

    // Configuración de estilo (igual que el original)
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(kFontIndex);

    // Calcular métricas de línea tras fijar la fuente
    const uint16_t lineHeight = tft.fontHeight();

    // Asegurar ancho válido (evita bucles patológicos con maxWidth=0)
    if (maxWidth == 0) maxWidth = 1;

    // Borrar el área de texto (mismo rectángulo que el original)
    tft.fillRect(kEraseX, (yInicio > (lineHeight / 2)) ? (yInicio - lineHeight / 2) : 0,
                 kEraseWidth, kMaxTextHeight, TFT_BLACK);

    // Punteros de avance y coordenada Y de dibujo
    const char* ptr = texto;
    uint16_t y = yInicio;

    // Dibujo línea a línea hasta agotar texto o altura máxima
    while (*ptr && (y + lineHeight) <= (yInicio + kMaxTextHeight)) {
        const char* scan = ptr;
        size_t lastSpace = 0;
        size_t charsCount = 0;
        uint16_t w = 0;

        // Explorar cuántos caracteres caben en la línea actual
        while (*scan) {
            ++charsCount;
            if (charsCount >= kBufCap - 1) break; // deja sitio para '\0'

            // Copiar fragmento a buffer temporal para medir
            char buf[kBufCap];
            const size_t copyLen = charsCount;
            memcpy(buf, ptr, copyLen);
            buf[copyLen] = '\0';

            w = tft.textWidth(buf);
            if (w > maxWidth) {
                // Retroceder un carácter y cortar
                --charsCount;
                break;
            }
            if (*scan == ' ') lastSpace = charsCount;
            ++scan;
        }

        // Elegir el corte: al espacio más cercano o por número de caracteres
        size_t lineLen;
        if (*scan == '\0') {
            lineLen = charsCount; // resto del texto
        } else {
            lineLen = (lastSpace > 0) ? lastSpace : charsCount;
        }

        // Progreso mínimo: si nada cabe (palabra más ancha que maxWidth),
        // dibujar al menos el primer carácter para evitar bucles de “línea vacía”.
        if (lineLen == 0 && *ptr != '\0') {
            lineLen = 1;
        }

        // Copiar y dibujar la línea
        char lineBuf[kBufCap];
        const size_t safeLen = (lineLen < (kBufCap - 1)) ? lineLen : (kBufCap - 1);
        memcpy(lineBuf, ptr, safeLen);
        lineBuf[safeLen] = '\0';
        tft.drawString(lineBuf, xCentro, y);

        // Avanzar el puntero global: saltar espacios consecutivos
        ptr += safeLen;
        while (*ptr == ' ') ++ptr;

        // Siguiente línea
        y += lineHeight;
    }

    // Restaurar estado previo
    tft.setTextFont(prevFont);
    tft.setTextDatum(prevDatum);
}

/**
 * @brief Muestra temporalmente la información de un elemento (ID y Serial) en pantalla.
 *
 * Borra la pantalla a negro, escribe el ID (en mayúsculas) y la etiqueta/valor de
 * “Serial” centrados y ajustados al ancho máximo, mantiene la vista durante
 * `delayTime` milisegundos (bloqueante) y, al finalizar, vuelve a dibujar el elemento actual.
 *
 * @param delayTime Tiempo de visualización en ms (bloqueante).
 * @param serialNumberRaw Número de serie (se mostrará en mayúsculas).
 * @param elementIDRaw    Identificador del elemento (se mostrará en mayúsculas).
 */
void showElemInfo(unsigned long delayTime,
                  const String& serialNumberRaw,
                  const String& elementIDRaw)
{
    // ── Constantes de layout ───────────────────────────────────────────
    constexpr uint16_t kMarginLR   = 5;   // margen izquierdo/derecho (px)
    constexpr uint16_t kIdY        = 20;  // Y de la línea de ID
    constexpr uint16_t kSerialLblY = 55;  // Y de la etiqueta "Serial"
    constexpr uint16_t kSerialValY = 85;  // Y del valor del serial

    // ── 1) Limpiar pantalla ────────────────────────────────────────────
    tft.fillScreen(TFT_BLACK);

    // ── 2) Normalizar a mayúsculas (sin recortar espacios) ────────────
    String elementID    = elementIDRaw;    elementID.toUpperCase();
    String serialNumber = serialNumberRaw; serialNumber.toUpperCase();

    // ── 3) Preparar textos ─────────────────────────────────────────────
    const String idLine      = String("ID: ") + elementID;
    const String serialLabel = "Serial";
    const String serialValue = serialNumber;

    // ── 4) Coordenadas/anchos comunes ──────────────────────────────────
    const uint16_t xCentro  = static_cast<uint16_t>(tft.width() / 2);
    const uint16_t maxWidth = static_cast<uint16_t>(tft.width() - (kMarginLR * 2));

    // ── 5..7) Mostrar ID, etiqueta y valor ─────────────────────────────
    mostrarTextoAjustado(tft, idLine.c_str(),      xCentro, kIdY,        maxWidth);
    mostrarTextoAjustado(tft, serialLabel.c_str(), xCentro, kSerialLblY, maxWidth);
    mostrarTextoAjustado(tft, serialValue.c_str(), xCentro, kSerialValY, maxWidth);

    // ── 8) Retención bloqueante (con cesión mínima para WDT) ──────────
    const unsigned long start = millis();
    while ((millis() - start) < delayTime) {
        // Ceder CPU sin alterar la temporización observada
    #if defined(ESP32)
        delay(0);   // alimenta el WDT en ESP32; no añade latencia perceptible
    #else
        yield();    // en plataformas que lo soporten
    #endif
    }

    // ── 9) Volver al menú/elemento actual ─────────────────────────────
    drawCurrentElement();
}

/**
 * @brief Selecciona la fuente libre adecuada según el idioma actual.
 *
 * Para idiomas con textos típicamente más largos (p. ej. alemán o italiano),
 * usa `FreeSans9pt7b` para mejorar el encaje en pantalla. Para el resto,
 * selecciona `FreeSans12pt7b`. No modifica `setTextSize()`.
 */
inline void setFontForCurrentLanguage() {
    // Punteros constantes: una sola instancia y cero coste en tiempo de ejecución.
    static constexpr const GFXfont* kFontCompact = &FreeSans9pt7b;
    static constexpr const GFXfont* kFontNormal  = &FreeSans12pt7b;

    const GFXfont* chosen = kFontNormal;
    switch (currentLanguage) {
        case Language::IT:
        case Language::DE:
            // case Language::CA:
            chosen = kFontCompact;
            break;
        default:
            break;
    }

    // Evita cambiar de fuente si ya está aplicada.
    static const GFXfont* last = nullptr;
    if (chosen != last) {
        uiSprite.setFreeFont(chosen);
        last = chosen;
    }
}

// /**
//  * @brief Ticker horizontal con rebote y pausa para el nombre de fichero.
//  *
//  * Desplaza el texto en vaivén dentro de un área fija. Hace una pausa breve en
//  * cada extremo y vuelve a moverse en sentido contrario. Dibuja únicamente el
//  * “ticker” en una banda concreta de la pantalla, empujando el sprite auxiliar
//  * directamente a la TFT (no al `uiSprite` principal).
//  */
// void scrollFileNameTickerBounce(const String& fileName)
// {
//     // ── Parámetros de animación (se mantienen valores visuales) ──
//     constexpr unsigned long kFrameMs      = 20;   // ~50 FPS
//     constexpr unsigned long kPauseMs      = 500;  // 500 ms de pausa en extremos
//     constexpr int           kAreaH        = 22;   // alto del ticker
//     constexpr int           kAnchorY      = 47;   // centro vertical de referencia (mantener)
//     constexpr int           kAreaX        = 0;    // lado izquierdo del área

//     // Área horizontal dinámica (sustituye el 128 mágico por el ancho real)
//     const int screenW  = tft.width();
//     const int areaW    = screenW;                 // ocupa todo el ancho visible
//     const int areaY    = kAnchorY - (kAreaH / 2); // banda vertical centrada sobre kAnchorY

//     // ── Estado persistente ──
//     static int            pixelOffset      = 0;
//     static int            scrollDirection  = 1;   // 1→izquierda (texto se mueve hacia -X), -1→derecha
//     static unsigned long  lastScrollTime   = 0;
//     static bool           inPause          = false;
//     static unsigned long  pauseStartTime   = 0;
//     static String         lastText;

//     // Si cambia el texto, reini ciamos offset/pausa para evitar saltos
//     if (fileName != lastText) {
//         lastText        = fileName;
//         pixelOffset     = 0;
//         scrollDirection = 1;
//         inPause         = false;
//         lastScrollTime  = 0;
//         pauseStartTime  = 0;
//     }

//     // Medición con la misma fuente que usará el ticker
//     uiSprite.setFreeFont(&FreeSans9pt7b);
//     uiSprite.setTextWrap(false);
//     uiSprite.setTextSize(1);

//     const int fullTextWidth = uiSprite.textWidth(fileName);
//     if (fullTextWidth <= areaW || fullTextWidth <= 0) return; // no hace falta ticker

//     const unsigned long now = millis();

//     // Pausa en extremos
//     if (inPause) {
//         if ((now - pauseStartTime) < kPauseMs) {
//             return; // aún en pausa: no redibujamos (evita trabajo y parpadeo)
//         }
//         inPause = false; // fin de pausa, retomamos movimiento
//     }

//     // Avance temporal (cadencia fija)
//     if ((now - lastScrollTime) >= kFrameMs) {
//         pixelOffset += scrollDirection;

//         // Límites de desplazamiento
//         const int maxOffset = fullTextWidth - areaW; // ≥1, ya que fullTextWidth > areaW
//         if (pixelOffset <= 0) {
//             pixelOffset     = 0;
//             scrollDirection = +1;
//             inPause         = true;
//             pauseStartTime  = now;
//         } else if (pixelOffset >= maxOffset) {
//             pixelOffset     = maxOffset;
//             scrollDirection = -1;
//             inPause         = true;
//             pauseStartTime  = now;
//         }

//         lastScrollTime = now;
//     }

//     // ── Sprite auxiliar del ticker (se crea/reutiliza según tamaño) ──
//     static TFT_eSprite ticker(&tft);
//     static int sprW = 0, sprH = 0;

//     if (!ticker.created() || sprW != areaW || sprH != kAreaH) {
//         if (ticker.created()) ticker.deleteSprite();
//         if (ticker.createSprite(areaW, kAreaH) == nullptr) {
//             return; // sin RAM, salir silenciosamente
//         }
//         sprW = areaW; sprH = kAreaH;
//     }

//     // Preparar y dibujar texto desplazado
//     ticker.fillSprite(BACKGROUND_COLOR);
//     ticker.setFreeFont(&FreeSans9pt7b);
//     ticker.setTextWrap(false);
//     ticker.setTextSize(1);
//     ticker.setTextDatum(TL_DATUM);
//     ticker.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);

//     ticker.drawString(fileName, -pixelOffset, 0);

//     // Componer directamente sobre la TFT (comportamiento original)
//     ticker.pushSprite(kAreaX, areaY);
// }

void drawLoadingModalFrame(const char* message, int frameCount) {
    uiSprite.fillSprite(BACKGROUND_COLOR);

    // === TEXTO MULTILÍNEA AJUSTADO (estilo mostrarTextoAjustado) ===
    uiSprite.setFreeFont(&FreeSansBold12pt7b);
    uiSprite.setTextSize(1);
    uiSprite.setTextColor(TFT_WHITE, BACKGROUND_COLOR);
    uiSprite.setTextDatum(MC_DATUM);

    const uint16_t xCentro        = 64;   // centro X de la pantalla
    const uint16_t yInicio        = 30;   // Y inicial
    const uint16_t maxWidth       = 120;  // ancho máximo
    const uint16_t lineHeight     = uiSprite.fontHeight();
    const uint16_t maxTextHeight  = 55;

    // Borrar área de texto (por si hay residuos)
    uiSprite.fillRect(5, yInicio - lineHeight / 2, 118, maxTextHeight, BACKGROUND_COLOR);

    const char* ptr = message;
    uint16_t y = yInicio;

    while (*ptr && y + lineHeight <= yInicio + maxTextHeight) {
        const char* scan = ptr;
        size_t lastSpace = 0, charsCount = 0;
        uint16_t w = 0;

        while (*scan) {
            ++charsCount;
            if (charsCount >= 128) break;

            char buf[128];
            memcpy(buf, ptr, charsCount);
            buf[charsCount] = '\0';

            w = uiSprite.textWidth(buf);
            if (w > maxWidth) { --charsCount; break; }
            if (*scan == ' ') lastSpace = charsCount;
            ++scan;
        }

        size_t lineLen = (*scan == '\0') ? charsCount
                          : (lastSpace > 0 ? lastSpace : charsCount);

        char lineBuf[128];
        memcpy(lineBuf, ptr, lineLen);
        lineBuf[lineLen] = '\0';
        uiSprite.drawString(lineBuf, xCentro, y);
        ptr += lineLen;
        while (*ptr == ' ') ++ptr;

        y += lineHeight;
    }

    // // === ANIMACIÓN DE PUNTOS GIRANDO ===
    // int centerX = uiSprite.width() / 2;
    // int centerY = uiSprite.height() - 50;
    // int radius  = 10;

    // for (int i = 0; i < 8; i++) {
    //     float angle = frameCount * 0.2 + i * PI / 4;
    //     int x = centerX + radius * cos(angle);
    //     int y = centerY + radius * sin(angle);
    //     uiSprite.fillCircle(x, y, 2, TEXT_COLOR);
    // }

    uiSprite.pushSprite(0, 0);
}

// /**
//  * @brief Muestra un mensaje en una "card" con barra de progreso animada (barber-pole).
//  * @param message Mensaje a mostrar (UTF-8). Si es `nullptr`, se muestra una línea vacía.
//  * @param delayTime Duración total en milisegundos (bloqueante).
//  * @note  Usa delay() (~30 FPS). Si se requiere no bloquear, migrar a bucle con millis().
//  * @warning Mantiene estilos: BACKGROUND_COLOR, TEXT_COLOR, FreeSansBold9pt7b.
//  */
// void showMessageWithProgress(const char* message, unsigned long delayTime) {
//     ignoreInputs = true;

//     const unsigned long startTime = millis();
//     unsigned long now = startTime;
//     int frameCount = 0;

//     // Lienzo
//     const int W = tft.width();   // 128
//     const int H = tft.height();  // 128
//     uiSprite.setTextDatum(TL_DATUM);
//     uiSprite.setFreeFont(&FreeSansBold9pt7b);
//     uiSprite.setTextSize(1);

//     // Card centrada
//     const int cardW = 112;
//     const int cardH = 96;
//     const int cardX = (W - cardW) / 2; // 8
//     const int cardY = (H - cardH) / 2; // 16

//     // Tipografía
//     const int lineHeight = 20;
//     const int textX = cardX + 12;
//     const int textY = cardY + 14;
//     const int textMaxW = cardW - 24;

//     // Barra de progreso
//     const int barX = cardX + 12;
//     const int barW = cardW - 24;
//     const int barH = 12;
//     const int barY = cardY + cardH - 18; // margen inferior

//     // Bucle de animación
//     while ((now = millis()) - startTime < delayTime) {
//         uiSprite.fillSprite(BACKGROUND_COLOR);

//         // —— Esquinas tipo “marcador” (toque moderno sin cambiar la paleta)
//         const int notch = 8;
//         // Esquina sup-izq
//         uiSprite.drawFastHLine(cardX, cardY, notch, TEXT_COLOR);
//         uiSprite.drawFastVLine(cardX, cardY, notch, TEXT_COLOR);
//         // sup-der
//         uiSprite.drawFastHLine(cardX + cardW - notch, cardY, notch, TEXT_COLOR);
//         uiSprite.drawFastVLine(cardX + cardW - notch + (notch-1), cardY, notch, TEXT_COLOR);
//         // inf-izq
//         uiSprite.drawFastHLine(cardX, cardY + cardH - 1, notch, TEXT_COLOR);
//         uiSprite.drawFastVLine(cardX, cardY + cardH - notch, notch, TEXT_COLOR);
//         // inf-der
//         uiSprite.drawFastHLine(cardX + cardW - notch, cardY + cardH - 1, notch, TEXT_COLOR);
//         uiSprite.drawFastVLine(cardX + cardW - 1, cardY + cardH - notch, notch, TEXT_COLOR);

//         // Marco de la card
//         uiSprite.drawRoundRect(cardX, cardY, cardW, cardH, 8, TEXT_COLOR);

//         // —— Texto con word-wrap y recorte a 3 líneas (con “…” si se desborda)
//         String remaining = (message ? String(message) : String(""));
//         String currentLine = "";
//         int cursorY = textY;
//         int linesDrawn = 0;
//         while (remaining.length() > 0 && linesDrawn < 3) {
//             int spaceIdx = remaining.indexOf(' ');
//             int nlIdx    = remaining.indexOf('\n');
//             int cut = -1; bool forcedNL = false;

//             // Elegir el separador más cercano (espacio o salto de línea)
//             if (spaceIdx == -1 && nlIdx == -1) {
//                 cut = remaining.length();
//             } else if (spaceIdx == -1) {
//                 cut = nlIdx; forcedNL = true;
//             } else if (nlIdx == -1) {
//                 cut = spaceIdx;
//             } else {
//                 cut = (spaceIdx < nlIdx) ? spaceIdx : nlIdx;
//                 forcedNL = (nlIdx != -1 && nlIdx <= spaceIdx);
//             }

//             String word = remaining.substring(0, cut);
//             String probe = currentLine + word + (forcedNL ? "" : " ");

//             if (uiSprite.textWidth(probe) <= textMaxW) {
//                 currentLine = probe;
//                 if (cut >= (int)remaining.length()) {
//                     // Última palabra del texto
//                     uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
//                     uiSprite.drawString(currentLine, textX, cursorY);
//                     linesDrawn++;
//                     break;
//                 } else {
//                     remaining = remaining.substring(cut + 1);
//                     if (forcedNL) {
//                         uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
//                         uiSprite.drawString(currentLine, textX, cursorY);
//                         cursorY += lineHeight;
//                         currentLine = "";
//                         linesDrawn++;
//                     }
//                 }
//             } else {
//                 // Línea completa -> pintar y pasar a la siguiente
//                 uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
//                 uiSprite.drawString(currentLine, textX, cursorY);
//                 cursorY += lineHeight;
//                 currentLine = "";
//                 linesDrawn++;

//                 // Si ya vamos a rebasar 3 líneas, pintar "…"
//                 if (linesDrawn >= 3) {
//                     uiSprite.drawString("...", textX + textMaxW - uiSprite.textWidth("..."), cursorY - lineHeight);
//                     break;
//                 }
//                 // no consumir la palabra, se reevalúa en la nueva línea
//             }
//             if (remaining.length() == 0 && currentLine.length() > 0 && linesDrawn < 3) {
//                 uiSprite.drawString(currentLine, textX, cursorY);
//                 linesDrawn++;
//             }
//         }

//         // Separador sutil bajo el texto
//         int sepY = textY + lineHeight * (linesDrawn == 0 ? 1 : linesDrawn) + 2;
//         if (sepY < barY - 14) {
//             uiSprite.drawFastHLine(cardX + 10, sepY, cardW - 20, TEXT_COLOR);
//         }

//         // —— Icono ecualizador (3 barras animadas) a la izquierda de la barra
//         const int eqX = barX - 8;    // pegado a la barra
//         const int eqY = barY - 16;   // por encima
//         const int eqW = 5;           // ancho de cada barra
//         const int eqGap = 4;
//         for (int i = 0; i < 3; i++) {
//             int period = 40;
//             int wave   = abs(((frameCount + i * 8) % period) - period / 2);
//             int h      = map(wave, 0, period / 2, 16, 6); // respira entre 6 y 16 px
//             int bx     = eqX + i * (eqW + eqGap);
//             int by     = eqY + (16 - h);
//             uiSprite.fillRect(bx, by, eqW, h, TEXT_COLOR);
//             // pequeña línea base
//             uiSprite.drawFastHLine(eqX, eqY + 16, eqW * 3 + eqGap * 2, TEXT_COLOR);
//         }

//         // —— Barra de progreso con animación barber-pole
//         float t = (delayTime == 0) ? 1.0f : (float)(now - startTime) / (float)delayTime;
//         if (t > 1.0f) t = 1.0f;
//         int progW = (int)(barW * t);

//         // Pista
//         uiSprite.drawRoundRect(barX - 1, barY - 1, barW + 2, barH + 2, 4, TEXT_COLOR);
//         // Relleno de progreso
//         if (progW > 0) {
//             uiSprite.fillRoundRect(barX, barY, progW, barH, 4, TEXT_COLOR);

//             // Rayas diagonales “barber-pole” recortadas al ancho de progreso
//             int stripeStep = 6;
//             int offset = frameCount % stripeStep;
//             for (int sx = -barH + offset; sx < progW; sx += stripeStep) {
//                 // Línea diagonal dentro del área de progreso
//                 int x1 = barX + sx;
//                 int y1 = barY;
//                 int x2 = barX + sx + barH;
//                 int y2 = barY + barH - 1;
//                 // recorte rudimentario a progW
//                 if (x2 > barX + progW) {
//                     int dx = x2 - (barX + progW);
//                     x2 -= dx; y2 -= dx; // desplazar fin para no salir
//                 }
//                 if (x1 < barX) {
//                     int dx = barX - x1;
//                     x1 += dx; y1 += dx;
//                 }
//                 if (x1 <= x2) {
//                     uiSprite.drawLine(x1, y1, x2, y2, BACKGROUND_COLOR);
//                 }
//             }
//         }

//         // Porcentaje a la derecha de la barra
//         char pct[8];
//         int percent = (int)(t * 100.0f + 0.5f);
//         snprintf(pct, sizeof(pct), "%d%%", percent);
//         int pctW = uiSprite.textWidth(pct);
//         int pctX = barX + barW - pctW;
//         int pctY = barY - 16;
//         uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
//         uiSprite.drawString(pct, pctX, pctY);

//         // Empujar al TFT
//         uiSprite.pushSprite(0, 0);

//         frameCount++;
//         delay(33); // ~30 FPS
//     }

//     ignoreInputs = false;
// }

bool extraElementsMenuActive = false;
int  extraElementsMenuSelection = 0;

bool confirmEnableDadoActive = false;
int  confirmEnableDadoSelection = 0;

void drawExtraElementsMenu(int selection)
{
    /*──────── Constantes/layout ────────*/
    const int screenW           = tft.width();
    constexpr int kTitleY       = 5;
    constexpr int kListStartY   = 30;
    constexpr int kCardRadius   = 3;
    constexpr int kHLPadX       = 3;
    constexpr int kHLPadY       = 1;
    constexpr int kRowTopInset  = (kHLPadY + 1);
    constexpr int kTickerFrameMs= 50;

    const int visibleOptions    = 2;   // Dado + Volver
    const int total             = 2;

    const int x              = 9;
    const int scrollBarW     = 6;
    const int scrollBarX     = screenW - scrollBarW - 3;
    const int textAreaW      = scrollBarX - x - 2;

    const int cardHeight     = CARD_HEIGHT;
    const int cardMargin     = CARD_MARGIN;
    const int stepY          = cardHeight + cardMargin;

    /*──────── Estado TICKER título (persistente) ────────*/
    static int  titleTickerOffset    = 0;
    static int  titleTickerDirection = 1; // 1→derecha, -1→izquierda
    static unsigned long titleLastFrameTime = 0;
    static TFT_eSprite titleTicker(&tft);
    static int titleW = 0, titleH = 0;

    /*──────── Estado TICKER fila (persistente) ────────*/
    static int lastSelectedIndex  = -1;
    static int optTickerOffset    = 0;
    static int optTickerDirection = 1; // 1→derecha, -1→izquierda
    static unsigned long optLastFrameTime = 0;
    static TFT_eSprite optTicker(&tft);
    static int tickerW = 0, tickerH = 0;

    /*──────── Preparar items (con traducción) ────────*/
    const bool enabled = isDadoEnabled();
    String dadoLine = String(getTranslation("DADO")) + " (" +
                      String(getTranslation(enabled ? "HABILITADO" : "DESHABILITADO")) + ")";

    const String items[] = {
        dadoLine,
        String(getTranslation("VOLVER"))
    };

    /*──────── Cabecera ────────*/
    uiSprite.fillSprite(BACKGROUND_COLOR);

    // Título con fuente grande, ticker si desborda
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextSize(1);
    const String title = String(getTranslation("ELEMENTOS_ADICIONALES"));

    const int titleTextW = uiSprite.textWidth(title);
    const int titleAreaW = screenW - 8;  // margen 4px por lado

    if (titleTextW > titleAreaW) {
        const unsigned long now = millis();
        if (now - titleLastFrameTime >= kTickerFrameMs) {
            titleTickerOffset += titleTickerDirection;
            if (titleTickerOffset < 0) { titleTickerOffset = 0; titleTickerDirection = 1; }
            const int maxOffsetX = titleTextW - titleAreaW;
            if (titleTickerOffset > maxOffsetX) { titleTickerOffset = maxOffsetX; titleTickerDirection = -1; }
            titleLastFrameTime = now;
        }

        if (!titleTicker.created() || titleW != titleAreaW || titleH != uiSprite.fontHeight()) {
            if (titleTicker.created()) titleTicker.deleteSprite();
            titleTicker.createSprite(titleAreaW, uiSprite.fontHeight());
            titleW = titleAreaW; titleH = uiSprite.fontHeight();
        }

        titleTicker.fillSprite(BACKGROUND_COLOR);
        titleTicker.setFreeFont(&FreeSans12pt7b);
        titleTicker.setTextSize(1);
        titleTicker.setTextDatum(TL_DATUM);
        titleTicker.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
        titleTicker.drawString(title, -titleTickerOffset, 0);
        // centrado horizontal usando un viewport
        const int titleX = (screenW - titleAreaW) / 2;
        titleTicker.pushToSprite(&uiSprite, titleX, kTitleY);
    } else {
        uiSprite.setTextDatum(TC_DATUM);
        uiSprite.drawString(title, screenW/2, kTitleY);
    }

    /*──────── Viewport listado con clipping ────────*/
    static TFT_eSprite listSprite(&tft);
    static int listW = 0, listH = 0;
    const int wantedListW = screenW;
    const int wantedListH = visibleOptions * stepY;

    if (!listSprite.created() || listW != wantedListW || listH != wantedListH) {
        if (listSprite.created()) listSprite.deleteSprite();
        listSprite.createSprite(wantedListW, wantedListH);
        listW = wantedListW; listH = wantedListH;
    }

    listSprite.fillSprite(BACKGROUND_COLOR);
    listSprite.setFreeFont(&FreeSans9pt7b);
    listSprite.setTextSize(1);
    listSprite.setTextDatum(TL_DATUM);
    listSprite.setTextColor(TEXT_COLOR);

    // No hay scroll vertical (2 items), pero dibujamos con el mismo layout
    const int listYOffset = 0;
    const int startIndex  = 0;
    const int firstVI     = 0;
    const int lastVI      = total - 1;

    for (int vi = firstVI; vi <= lastVI; ++vi)
    {
        const int y = kRowTopInset + listYOffset + (vi - startIndex) * stepY;
        const bool isSelected = (vi == selection);

        // Resaltado con clamp al viewport
        if (isSelected) {
            int rectY = y - kHLPadY;
            int rectH = cardHeight + (kHLPadY * 2);
            if (rectY < 0)            { rectH += rectY; rectY = 0; }
            if (rectY + rectH > listH){ rectH = listH - rectY; }
            if (rectH > 0) {
                listSprite.fillRoundRect(
                    x - kHLPadX, rectY,
                    textAreaW + (kHLPadX * 2),
                    rectH, kCardRadius,
                    CARD_COLOR
                );
            }
        }

        // Ticker horizontal SOLO en la fila seleccionada (igual que en Modos/Formato)
        String label = items[vi];

        if (isSelected) {
            if (selection != lastSelectedIndex) {
                optTickerOffset    = 0;
                optTickerDirection = 1;
                lastSelectedIndex  = selection;
            }

            const int fullW = listSprite.textWidth(label);
            if (fullW > textAreaW) {
                const unsigned long now = millis();
                if (now - optLastFrameTime >= kTickerFrameMs) {
                    optTickerOffset += optTickerDirection;
                    if (optTickerOffset < 0) { optTickerOffset = 0; optTickerDirection = 1; }
                    const int maxOffsetX = fullW - textAreaW;
                    if (optTickerOffset > maxOffsetX) { optTickerOffset = maxOffsetX; optTickerDirection = -1; }
                    optLastFrameTime = now;
                }

                if (!optTicker.created() || tickerW != textAreaW || tickerH != cardHeight) {
                    if (optTicker.created()) optTicker.deleteSprite();
                    optTicker.createSprite(textAreaW, cardHeight);
                    tickerW = textAreaW; tickerH = cardHeight;
                }

                optTicker.fillSprite(CARD_COLOR);
                optTicker.setFreeFont(&FreeSans9pt7b);
                optTicker.setTextSize(1);
                optTicker.setTextDatum(TL_DATUM);
                optTicker.setTextColor(TEXT_COLOR, CARD_COLOR);
                optTicker.drawString(label, -optTickerOffset, 0);
                optTicker.pushToSprite(&listSprite, x, y);
            } else {
                listSprite.drawString(label, x, y);
            }
        } else {
            listSprite.drawString(label, x, y);
        }
    }

    listSprite.pushToSprite(&uiSprite, 0, kListStartY);
    uiSprite.pushSprite(0, 0);
}

void drawConfirmEnableDadoMenu(int selection)
{
    // ── Constantes de layout (alineado a drawConfirmDelete) ───────────
    const int screenW         = tft.width();
    constexpr int kTitleY     = 5;
    constexpr int kCardRadius = 4;

    // Área centrada para el título (con ticker si desborda)
    constexpr int kTitleAreaW = 120;
    const int     kTitleAreaX = (screenW - kTitleAreaW) / 2;
    const int     kTitleAreaH = 20;   // alto aprox. para 12pt
    const int     kTitleAreaY = kTitleY;

    // Botones Sí/No centrados
    constexpr int kButtons      = 2;
    constexpr int boxW          = 88;
    constexpr int boxH          = 22;
    constexpr int spacing       = 28;
    const     int yBase         = 64; // misma pauta vertical
    const     int rectXCentered = (screenW - (boxW + 8)) / 2; // caja centrada

    // ── Estado del ticker del título ───────────────────────────────────
    constexpr int kTickerFrameMs = 50;
    static String lastTitle;
    static int titleTickerOffset    = 0;
    static int titleTickerDirection = 1;   // 1→derecha, -1→izquierda
    static unsigned long titleLastFrame = 0;
    static TFT_eSprite titleSprite(&tft);
    static int titleW = 0, titleH = 0;

    // ── Composición del título ─────────────────────────────────────────
    const bool enabled = isDadoEnabled();
    const String title = String(getTranslation(enabled ? "DESHABILITAR_DADO"
                                                       : "HABILITAR_DADO"));

    // ── Fondo + tipografías ───────────────────────────────────────────
    uiSprite.fillSprite(BACKGROUND_COLOR);
    uiSprite.setFreeFont(&FreeSans12pt7b);
    uiSprite.setTextColor(TEXT_COLOR);
    uiSprite.setTextSize(1);

    // (Re)crear sprite del título si cambian dimensiones
    if (!titleSprite.created() || titleW != kTitleAreaW || titleH != kTitleAreaH) {
        if (titleSprite.created()) titleSprite.deleteSprite();
        titleSprite.createSprite(kTitleAreaW, kTitleAreaH);
        titleW = kTitleAreaW; titleH = kTitleAreaH;
    }

    // Reset del ticker si cambia el texto del título
    if (title != lastTitle) {
        titleTickerOffset    = 0;
        titleTickerDirection = 1;
        titleLastFrame       = 0;
        lastTitle            = title;
    }

    // Medición con la fuente real del título
    titleSprite.fillSprite(BACKGROUND_COLOR);
    titleSprite.setFreeFont(&FreeSans12pt7b);
    titleSprite.setTextWrap(false);
    titleSprite.setTextSize(1);
    titleSprite.setTextDatum(ML_DATUM);
    titleSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);

    const int titleFullW = titleSprite.textWidth(title);
    if (titleFullW <= kTitleAreaW) {
        // Centrado visual cuando cabe
        const int xCentered = (kTitleAreaW - titleFullW) / 2;
        titleSprite.drawString(title, xCentered, kTitleAreaH / 2);
    } else {
        // Ticker “ping-pong”
        const unsigned long now = millis();
        if (titleLastFrame == 0 || (now - titleLastFrame) >= kTickerFrameMs) {
            titleTickerOffset += titleTickerDirection;
            if (titleTickerOffset < 0) {
                titleTickerOffset = 0; titleTickerDirection = 1;
            }
            const int maxOffset = titleFullW - kTitleAreaW;
            if (titleTickerOffset > maxOffset) {
                titleTickerOffset = maxOffset; titleTickerDirection = -1;
            }
            titleLastFrame = now;
        }
        titleSprite.drawString(title, -titleTickerOffset, kTitleAreaH / 2);
    }

    // Pintar el título centrado en pantalla
    titleSprite.pushToSprite(&uiSprite, kTitleAreaX, kTitleAreaY);

    // ── Botones Sí / No (cajas centradas, texto centrado) ─────────────
    uiSprite.setFreeFont(&FreeSans9pt7b);
    uiSprite.setTextSize(1);
    uiSprite.setTextDatum(MC_DATUM);

    const char* opciones[] = { getTranslation("YES_DELETE"), getTranslation("NO_DELETE") };

    // Clamp defensivo del índice seleccionado
    int sel = selection;
    if (sel < 0) sel = 0;
    if (sel >= kButtons) sel = kButtons - 1;

    for (int i = 0; i < kButtons; ++i) {
        const int rectX  = rectXCentered;
        const int rectY  = (yBase + i * spacing) - 2;
        const int rectW  = boxW + 8;
        const int rectH  = boxH + 4;

        if (i == sel) {
            uiSprite.fillRoundRect(rectX, rectY, rectW, rectH, kCardRadius, CARD_COLOR);
            uiSprite.setTextColor(TEXT_COLOR, CARD_COLOR);
        } else {
            uiSprite.setTextColor(TEXT_COLOR, BACKGROUND_COLOR);
        }

        const int cx = rectX + rectW / 2;
        const int cy = rectY + rectH / 2;
        uiSprite.drawString(opciones[i], cx, cy);
    }

    // ── Volcado final ──────────────────────────────────────────────────
    uiSprite.pushSprite(0, 0);
}
